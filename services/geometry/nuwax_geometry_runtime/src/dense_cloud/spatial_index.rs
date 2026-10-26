//! 逐原坐标的磁盘RTree（矩形空间树）索引；原坐标只从辅助二进制列读取。
//! 同次追加且同归档页内的完全相同坐标合并树项，页内位置列表保留全部重复观测。
use super::{decode_point, page_table::PageTable, Page, PAGE_POINTS};
use rusqlite::{ffi, params, Connection};
use std::{collections::BTreeMap, fs::File, os::unix::fs::FileExt, sync::Arc};

const RESIDENT_LIMIT: usize = 16 * 1024 * 1024;

pub(super) struct SpatialIndex {
    connection: Connection,
    pub(super) last_read_pages: usize,
    pub(super) last_candidate_points: usize,
    pub(super) last_index_read_pages: usize,
}

fn db_status(connection: &Connection, operation: i32) -> usize {
    let (mut current, mut maximum) = (0, 0);
    // 连接由完整源拥有方独占，句柄只在本调用期间使用。
    let code = unsafe { ffi::sqlite3_db_status(connection.handle(), operation, &mut current, &mut maximum, 0) };
    assert_eq!(code, ffi::SQLITE_OK, "已固定SQLite版本须支持连接状态计数");
    current.max(0) as usize
}

fn resident_bytes(connection: &Connection) -> usize {
    [ffi::SQLITE_DBSTATUS_CACHE_USED, ffi::SQLITE_DBSTATUS_SCHEMA_USED, ffi::SQLITE_DBSTATUS_STMT_USED]
        .into_iter().map(|operation| db_status(connection, operation)).sum()
}

fn check_resident(connection: &Connection) -> Result<(), String> {
    if resident_bytes(connection) <= RESIDENT_LIMIT { return Ok(()); }
    // cache_size只是缓存目标。释放未固定页后再核对实际连接占用，超出则回滚当前事务。
    connection.flush_prepared_statement_cache();
    unsafe { ffi::sqlite3_db_release_memory(connection.handle()); }
    if resident_bytes(connection) > RESIDENT_LIMIT {
        return Err("完整点云空间索引连接驻留量超过16MiB，当前提交未发布".into());
    }
    Ok(())
}

impl SpatialIndex {
    pub(super) fn new() -> Result<Self, String> {
        let connection = Connection::open("").map_err(|e| e.to_string())?;
        connection.set_prepared_statement_cache_capacity(4);
        let configured = unsafe { ffi::sqlite3_db_config(connection.handle(), ffi::SQLITE_DBCONFIG_LOOKASIDE,
            std::ptr::null_mut::<std::ffi::c_void>(), 0_i32, 0_i32) };
        if configured != ffi::SQLITE_OK { return Err("完整点云空间索引内存配置失败".into()); }
        connection.execute_batch("PRAGMA page_size=4096; PRAGMA cache_size=-8192;
            PRAGMA cache_spill=ON; PRAGMA mmap_size=0; PRAGMA temp_store=FILE;
            PRAGMA journal_mode=DELETE; PRAGMA synchronous=OFF; PRAGMA threads=0;
            CREATE VIRTUAL TABLE points USING rtree(id,min_x,max_x,min_y,max_y,min_z,max_z,+xyz,+positions);
            CREATE TEMP TABLE revoke_ids(id INTEGER PRIMARY KEY);
            PRAGMA temp.cache_size=-1024; PRAGMA temp.cache_spill=ON;")
            .map_err(|e| e.to_string())?;
        Ok(Self { connection, last_read_pages: 0, last_candidate_points: 0, last_index_read_pages: 0 })
    }

    pub(super) fn append(&mut self, xyz: &[u8], first: u64) -> Result<(), String> {
        // 合并只针对原12字节完全相同的点；既不量化坐标，也不跨归档页改变点身份。
        let mut groups: BTreeMap<(u64, [u8; 12]), (u64, Vec<u8>)> = BTreeMap::new();
        for (index, bytes) in xyz.chunks_exact(12).enumerate() {
            let position = first + index as u64;
            let raw: [u8; 12] = bytes.try_into().unwrap();
            let (_, locals) = groups.entry((position / PAGE_POINTS as u64, raw))
                .or_insert_with(|| (position + 1, Vec::new()));
            locals.extend_from_slice(&((position % PAGE_POINTS as u64) as u16).to_le_bytes());
        }
        let transaction = self.connection.transaction().map_err(|e| e.to_string())?;
        {
            let mut insert = transaction.prepare_cached("INSERT INTO points VALUES (?1,?2,?2,?3,?3,?4,?4,?5,?6)")
                .map_err(|e| e.to_string())?;
            for ((_, raw), (id, locals)) in groups {
                let point = decode_point(&raw).map(f64::from);
                // f32原坐标升为f64精确表示，RTree的向外舍入无需改变它；最终判定仍读取raw。
                insert.execute(params![id, point[0], point[1], point[2], raw.as_slice(), locals])
                    .map_err(|e| e.to_string())?;
            }
        }
        check_resident(&transaction)?;
        transaction.commit().map_err(|e| e.to_string())
    }

    pub(super) fn remove(
        &mut self, regions: impl Iterator<Item = ([f64; 3], [f64; 3])>,
        frame: &crate::dynamic_backend::MappedFrame, pages: &PageTable, file: &File,
    ) -> Result<Vec<(usize, Arc<Page>)>, String> {
        self.last_read_pages = 0;
        self.last_candidate_points = 0;
        self.last_index_read_pages = 0;
        let misses_before = db_status(&self.connection, ffi::SQLITE_DBSTATUS_CACHE_MISS);
        let transaction = self.connection.transaction().map_err(|e| e.to_string())?;
        transaction.execute("DELETE FROM temp.revoke_ids", []).map_err(|e| e.to_string())?;
        let mut changes: BTreeMap<usize, (Page, bool)> = BTreeMap::new();
        for (low, high) in regions {
            // 虚表自身完成三轴范围剪枝，不能先领取粗格内所有历史点再做坐标过滤。
            let mut select = transaction.prepare_cached("SELECT id,xyz,positions FROM points
                WHERE min_x<=?1 AND max_x>=?2 AND min_y<=?3 AND max_y>=?4 AND min_z<=?5 AND max_z>=?6")
                .map_err(|e| e.to_string())?;
            let mut rows = select.query(params![high[0], low[0], high[1], low[1], high[2], low[2]])
                .map_err(|e| e.to_string())?;
            while let Some(row) = rows.next().map_err(|e| e.to_string())? {
                let id: u64 = row.get(0).map_err(|e| e.to_string())?;
                let raw = row.get_ref(1).map_err(|e| e.to_string())?.as_blob().map_err(|e| e.to_string())?;
                let locals = row.get_ref(2).map_err(|e| e.to_string())?.as_blob().map_err(|e| e.to_string())?;
                if id == 0 || raw.len() != 12 || locals.len() > PAGE_POINTS * 2 || locals.len() % 2 != 0 {
                    return Err("完整点云RTree记录非法".into());
                }
                let index = ((id - 1) / PAGE_POINTS as u64) as usize;
                let old = pages.get(index).ok_or("完整点云空间索引页号越界")?;
                let point = decode_point(raw);
                self.last_candidate_points += locals.len() / 2;
                let clear = frame.clears(point.map(f64::from));
                let mut any_visible = false;
                for local in locals.chunks_exact(2) {
                    let local = u16::from_le_bytes(local.try_into().unwrap()) as usize;
                    if local >= old.length { return Err("完整点云空间索引页内位置非法".into()); }
                    if old.visible[local / 64] & (1 << (local % 64)) == 0 { continue; }
                    any_visible = true;
                    if !clear { continue; }
                    let (page, bounds_changed) = changes.entry(index).or_insert_with(|| ((**old).clone(), false));
                    if page.visible[local / 64] & (1 << (local % 64)) == 0 { continue; }
                    page.visible[local / 64] &= !(1 << (local % 64));
                    page.count -= 1;
                    *bounds_changed |= (0..3).any(|axis| point[axis] == old.low[axis] || point[axis] == old.high[axis]);
                }
                if clear || !any_visible {
                    // RTree游标存活时只写独立磁盘队列，不修改正在扫描的RTree；主键同时去重。
                    transaction.prepare_cached("INSERT OR IGNORE INTO temp.revoke_ids VALUES (?1)")
                        .map_err(|e| e.to_string())?.execute([id]).map_err(|e| e.to_string())?;
                }
            }
        }
        let mut updated = Vec::with_capacity(changes.len());
        for (index, (mut page, bounds_changed)) in changes {
            if bounds_changed {
                page.low = [f32::INFINITY; 3];
                page.high = [f32::NEG_INFINITY; 3];
                if page.count > 0 {
                    let mut bytes = vec![0; page.length * 12];
                    self.last_read_pages += 1;
                    file.read_exact_at(&mut bytes, (index * PAGE_POINTS * 12) as u64).map_err(|e| e.to_string())?;
                    for (local, bytes) in bytes.chunks_exact(12).enumerate() {
                        if page.visible[local / 64] & (1 << (local % 64)) == 0 { continue; }
                        let point = decode_point(bytes);
                        for axis in 0..3 { page.low[axis] = page.low[axis].min(point[axis]); page.high[axis] = page.high[axis].max(point[axis]); }
                    }
                }
            }
            updated.push((index, Arc::new(page)));
        }
        {
            // 此时所有RTree查询游标均已释放；队列按其主键流式读取，不在内存保存全部ID。
            let mut pending = transaction.prepare_cached("SELECT id FROM temp.revoke_ids").map_err(|e| e.to_string())?;
            let mut rows = pending.query([]).map_err(|e| e.to_string())?;
            let mut delete = transaction.prepare_cached("DELETE FROM points WHERE id=?1").map_err(|e| e.to_string())?;
            while let Some(row) = rows.next().map_err(|e| e.to_string())? {
                let id: u64 = row.get(0).map_err(|e| e.to_string())?;
                delete.execute([id]).map_err(|e| e.to_string())?;
            }
        }
        transaction.execute("DELETE FROM temp.revoke_ids", []).map_err(|e| e.to_string())?;
        check_resident(&transaction)?;
        transaction.commit().map_err(|e| e.to_string())?;
        self.last_index_read_pages = db_status(&self.connection, ffi::SQLITE_DBSTATUS_CACHE_MISS).saturating_sub(misses_before);
        Ok(updated)
    }

    #[cfg(test)]
    pub(super) fn memory_bytes(&self) -> usize { resident_bytes(&self.connection) }
    #[cfg(test)]
    pub(super) fn disk_bytes(&self) -> u64 {
        let pages: u64 = self.connection.query_row("PRAGMA page_count", [], |row| row.get(0)).unwrap();
        let temp_pages: u64 = self.connection.query_row("PRAGMA temp.page_count", [], |row| row.get(0)).unwrap();
        (pages + temp_pages) * 4096
    }
    #[cfg(test)]
    pub(super) fn maximum_memory_bytes() -> usize { RESIDENT_LIMIT }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rtree_candidates_preserve_archived_f32_and_auxiliary_coordinate_bits() {
        let mut index = SpatialIndex::new().unwrap();
        let values = [0.0_f32, -0.0, f32::from_bits(1), -f32::from_bits(1),
            f32::from_bits(0.15_f32.to_bits() - 1), 0.15, f32::from_bits(0.15_f32.to_bits() + 1), -100_000.125];
        let points: Vec<_> = values.into_iter().map(|x| [x, -0.3_f32, 1.025_f32]).collect();
        let bytes: Vec<_> = points.iter().flat_map(|point| point.iter().copied().flat_map(f32::to_le_bytes)).collect();
        index.append(&bytes, 0).unwrap();
        let mut statement = index.connection.prepare("SELECT id,min_x,max_x,xyz FROM points").unwrap();
        let mut rows = statement.query([]).unwrap();
        let mut count = 0;
        while let Some(row) = rows.next().unwrap() {
            let id: usize = row.get(0).unwrap();
            let (low, high): (f64, f64) = (row.get(1).unwrap(), row.get(2).unwrap());
            let raw: Vec<u8> = row.get(3).unwrap();
            assert_eq!(low, f64::from(points[id - 1][0]));
            assert_eq!(high, low, "已归档f32可由f64精确表示，索引不能额外扩张点框");
            assert_eq!(raw, bytes[(id - 1) * 12..id * 12], "最终判定用原字节，包括负零和次正规值");
            count += 1;
        }
        assert_eq!(count, points.len());
        drop(rows);
        drop(statement);
        eprintln!("DENSE_RTREE_SQLITE version={} measured_connection_bytes={} sqlite_process_allocated_bytes={}",
            rusqlite::version(), index.memory_bytes(), unsafe { ffi::sqlite3_memory_used() });
    }
}
