//! 将同机仿真采集时间用于真实新鲜度判断；不改动传感器时间或进程墙钟。
use std::{fs::File, os::unix::fs::FileExt, path::Path};

const CAPACITY: u64 = 16384;

#[derive(Debug)]
struct Acquisition {
    source_ns: u64,
    monotonic_ns: u64,
}

fn read<const N: usize>(file: &File, offset: u64) -> Result<[u8; N], String> {
    let mut bytes = [0; N];
    file.read_exact_at(&mut bytes, offset)
        .map_err(|e| format!("读取采集时间表失败：{e}"))?;
    Ok(bytes)
}

fn record(file: &File, sequence: u64) -> Result<Acquisition, String> {
    let offset = 16 + (sequence - 1) % CAPACITY * 32;
    let bytes = read::<32>(file, offset)?;
    let field = |start| u64::from_le_bytes(bytes[start..start + 8].try_into().unwrap());
    if field(0) != sequence || u64::from_le_bytes(read::<8>(file, offset)?) != sequence {
        return Err("采集时间表记录正在更新或已被覆盖".into());
    }
    Ok(Acquisition {
        source_ns: field(8),
        monotonic_ns: field(24),
    })
}

fn acquisition(path: &Path, source_ns: u64) -> Result<Acquisition, String> {
    let file = File::open(path).map_err(|e| format!("打开采集时间表失败：{e}"))?;
    let header = read::<16>(&file, 0)?;
    if &header[..8] != b"NEHTIME1" {
        return Err("采集时间表格式无效".into());
    }
    let end = u64::from_le_bytes(header[8..].try_into().unwrap());
    if end == 0 {
        return Err("尚无实际采集时间".into());
    }
    let start = end.saturating_sub(CAPACITY - 1).max(1);
    let last = record(&file, end)?;
    if source_ns > last.source_ns.saturating_add(1000) {
        return Err("源采样时间不在当前采集记录范围内".into());
    }
    // 使用前一个实际采集时刻，保守包含该物理步的耗时；不外推采集时间。
    let (mut lo, mut hi) = (start, end);
    while lo < hi {
        let middle = (lo + hi + 1) / 2;
        if record(&file, middle)?.source_ns <= source_ns {
            lo = middle;
        } else {
            hi = middle - 1;
        }
    }
    let selected = record(&file, lo)?;
    if selected.source_ns > source_ns {
        return Err("源采样时间早于当前采集记录范围".into());
    }
    Ok(selected)
}

fn monotonic_ns() -> Result<u64, String> {
    let mut value = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // CLOCK_MONOTONIC由同一Linux宿主提供，生产者与消费者共享其纪元。
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut value) } != 0 {
        return Err(std::io::Error::last_os_error().to_string());
    }
    Ok(value.tv_sec as u64 * 1_000_000_000 + value.tv_nsec as u64)
}

fn recorded_age(path: &Path, source_ns: u64, now_monotonic_ns: u64) -> Result<u64, String> {
    now_monotonic_ns
        .checked_sub(acquisition(path, source_ns)?.monotonic_ns)
        .ok_or_else(|| "采集单调时间晚于当前宿主时钟".into())
}

/// 默认按真实Unix时间计龄；显式同机仿真使用原采集单调时间，包含所有上游积压。
pub fn age_ns_at(source_ns: u64, wall_now_ns: u64) -> Result<u64, String> {
    match std::env::var_os("NUWAX_SOURCE_TIME_FILE") {
        Some(path) => recorded_age(Path::new(&path), source_ns, monotonic_ns()?),
        None => {
            if source_ns > wall_now_ns.saturating_add(50_000_000) {
                return Err("位姿源时间超前当前墙钟超过50毫秒".into());
            }
            Ok(wall_now_ns.saturating_sub(source_ns))
        }
    }
}

/// 仅给执行期限比较提供实际采集时刻；调用方仍保存原始源时间作为数据身份。
pub fn comparison_timestamp_ns_at(source_ns: u64, wall_now_ns: u64) -> Result<u64, String> {
    if std::env::var_os("NUWAX_SOURCE_TIME_FILE").is_none() {
        return Ok(source_ns);
    }
    wall_now_ns.checked_sub(age_ns_at(source_ns, wall_now_ns)?)
        .ok_or_else(|| "实际采集年龄超出当前Unix时刻".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn comparison_clock_preserves_capture_age_without_rejuvenating_old_frames() {
        use std::time::{Duration, SystemTime, UNIX_EPOCH};
        let wall = || SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as u64;
        if std::env::var_os("NEH_TEST_CAPTURE_CLOCK").is_some() {
            let source = std::env::var("NEH_TEST_CAPTURE_CLOCK").unwrap().parse::<u64>().unwrap();
            let first = comparison_timestamp_ns_at(source, wall()).unwrap();
            assert!(first > source + 50_000_000_000);
            std::thread::sleep(Duration::from_millis(20));
            let second = comparison_timestamp_ns_at(source, wall()).unwrap();
            assert!(first.abs_diff(second) < 5_000_000, "暂停旧帧不能随查询续期");
            assert!(comparison_timestamp_ns_at(source + 1_000_000, wall()).is_err());
            return;
        }
        let source = wall() - 60_000_000_000;
        let path = std::env::temp_dir().join(format!("neh-comparison-clock-{}", std::process::id()));
        let mut file = File::create(&path).unwrap();
        file.write_all(b"NEHTIME1").unwrap();
        for value in [1, 1, source, wall(), monotonic_ns().unwrap()] {
            file.write_all(&value.to_le_bytes()).unwrap();
        }
        file.sync_all().unwrap();
        let result = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "source_time::tests::comparison_clock_preserves_capture_age_without_rejuvenating_old_frames"])
            .env("NEH_TEST_CAPTURE_CLOCK", source.to_string())
            .env("NUWAX_SOURCE_TIME_FILE", &path).status().unwrap();
        std::fs::remove_file(path).unwrap();
        assert!(result.success());
    }

    #[test]
    fn newest_lookup_does_not_read_oldest_slot_being_reused() {
        let path = std::env::temp_dir().join(format!("neh-source-ring-{}", std::process::id()));
        let mut file = File::create(&path).unwrap();
        file.write_all(b"NEHTIME1").unwrap();
        file.write_all(&CAPACITY.to_le_bytes()).unwrap();
        for seq in 1..=CAPACITY {
            for value in [
                if seq == 1 { 0 } else { seq },
                seq * 5_000_000,
                42,
                seq * 10_000_000,
            ] {
                file.write_all(&value.to_le_bytes()).unwrap();
            }
        }
        assert_eq!(
            recorded_age(&path, CAPACITY * 5_000_000, CAPACITY * 10_000_000 + 20).unwrap(),
            20
        );
        assert!(recorded_age(&path, 5_000_000, CAPACITY * 10_000_000).is_err());
        drop(file);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn slow_paused_and_delayed_sources_use_actual_acquisition_age() {
        let path = std::env::temp_dir().join(format!("neh-source-time-{}", std::process::id()));
        let mut file = File::create(&path).unwrap();
        file.write_all(b"NEHTIME1").unwrap();
        file.write_all(&3_u64.to_le_bytes()).unwrap();
        // 逻辑采集间隔5ms，实际间隔10ms；日期故意与宿主相差很远。
        for (seq, source, mono) in [
            (1_u64, 100_000_000_u64, 1_000_000_000_u64),
            (2, 105_000_000, 1_010_000_000),
            (3, 110_000_000, 1_020_000_000),
        ] {
            for field in [seq, source, 42, mono] {
                file.write_all(&field.to_le_bytes()).unwrap();
            }
        }
        assert_eq!(
            recorded_age(&path, 110_000_000, 1_030_000_000).unwrap(),
            10_000_000
        );
        assert_eq!(
            recorded_age(&path, 110_000_000, 1_620_000_000).unwrap(),
            600_000_000
        );
        assert_eq!(
            recorded_age(&path, 105_000_000, 1_620_000_000).unwrap(),
            610_000_000
        );
        assert_eq!(
            recorded_age(&path, 107_000_000, 1_020_000_000).unwrap(),
            10_000_000
        );
        assert!(recorded_age(&path, 99_000_000, 1_030_000_000).is_err());
        assert!(recorded_age(&path, 111_000_000, 1_030_000_000).is_err());
        file.write_all(&0_u64.to_le_bytes()).unwrap();
        drop(file);
        std::fs::remove_file(path).unwrap();
    }
}
