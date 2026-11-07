//! 地图目录由 Rust 直接读写；整个 geometry 目录一次性原子安装。
use crate::{mapper::MapSnapshot, pointcloud::CloudSnapshot};
use prost::Message;
use serde::{Deserialize, Serialize};
use std::{
    ffi::CString,
    fs::{self, File},
    io::{Read, Write},
    os::unix::ffi::OsStrExt,
    path::{Path, PathBuf},
    sync::Arc,
};
use tonic::Status;
#[derive(Clone)]
pub struct MapStore {
    root: PathBuf,
}
#[derive(Serialize, Deserialize)]
struct Descriptor {
    map_from_odom: [f64; 16],
    last_pose: Option<Vec<u8>>,
    occupancy: bool,
    backend: bool,
    planar: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    planar_resume_unavailable_reason: Option<String>,
    fine: bool,
    dense: bool,
    scans: usize,
    voxel_size: f32,
    revision: u64,
    source_timestamp_ns: u64,
    final_poses: Vec<(u64, [f64; 16])>,
}
impl MapStore {
    pub fn new(root: PathBuf) -> Self {
        Self { root }
    }
    pub fn from_runtime() -> Self {
        Self::new(nuwax_config::resolve_path(
            &nuwax_config::geometry()
                .expect("geometry 配置")
                .map_catalog_root,
        ))
    }
    pub fn root(&self) -> &Path {
        &self.root
    }
    pub fn exists(&self, name: &str) -> bool {
        self.root.join(name).join("geometry").exists()
    }
    pub fn save(
        &self,
        name: &str,
        source: Option<&str>,
        mut snapshot: MapSnapshot,
    ) -> Result<bool, Status> {
        if let Some(checkpoint) = &snapshot.planar_checkpoint {
            let (metadata, data) = nuwax_slam_2d_runtime::archive::grid_structure(
                &mut std::io::Cursor::new(checkpoint),
            )
            .map_err(Status::internal)?;
            let (width, height) = metadata.dimensions();
            let origin = metadata.origin();
            let nuwax_slam_2d_runtime::archive::ArchiveMetadata::Assigned(assigned) = &metadata;
            snapshot.occupancy = Some(crate::proto::OccupancyGrid {
                map_name: name.into(),
                meta: Some(crate::proto::OccupancyGridMeta {
                    width,
                    height,
                    resolution: metadata.resolution() as f32,
                    origin_x: origin[0],
                    origin_y: origin[1],
                    frame_id: assigned.source.session.map_frame_id.clone(),
                    timestamp_ns: metadata.source_unix_timestamp_ns(),
                }),
                data: data.into_iter().map(|v| v as u8).collect(),
            });
            let estimate = metadata.estimate();
            snapshot.map_from_odom =
                crate::mapper::isometry_matrix(&nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(
                        estimate.map_from_odom[0],
                        estimate.map_from_odom[1],
                        0.,
                    ),
                    nalgebra::UnitQuaternion::from_euler_angles(0., 0., estimate.map_from_odom[2]),
                ));
            snapshot.last_pose = Some(crate::mapper::pose_from_isometry(
                nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(
                        estimate.map_from_base[0],
                        estimate.map_from_base[1],
                        0.,
                    ),
                    nalgebra::UnitQuaternion::from_euler_angles(0., 0., estimate.map_from_base[2]),
                ),
                &assigned.source.session.map_frame_id,
                metadata.source_unix_timestamp_ns(),
            ));
        }
        let map = self.root.join(name);
        let target = map.join("geometry");
        let replaced = target.exists();
        if replaced && source != Some(name) {
            return Err(Status::already_exists(format!("地图已存在：{name}")));
        }
        fs::create_dir_all(&map).map_err(write_error)?;
        let temporary = map.join(format!(".geometry-{}.tmp", uuid::Uuid::new_v4()));
        fs::create_dir(&temporary).map_err(write_error)?;
        let guard = TemporaryDirectory(temporary.clone());
        self.write_snapshot(&temporary, &snapshot)?;
        File::open(&temporary)
            .and_then(|f| f.sync_all())
            .map_err(write_error)?;
        if replaced {
            rename_flags(&temporary, &target, libc::RENAME_EXCHANGE).map_err(write_error)?;
        } else {
            rename_flags(&temporary, &target, libc::RENAME_NOREPLACE).map_err(|error| {
                if error.kind() == std::io::ErrorKind::AlreadyExists {
                    Status::already_exists(format!("地图已存在：{name}"))
                } else {
                    write_error(error)
                }
            })?;
        }
        File::open(&map)
            .and_then(|f| f.sync_all())
            .map_err(write_error)?;
        drop(guard);
        Ok(replaced)
    }
    fn write_snapshot(&self, path: &Path, snapshot: &MapSnapshot) -> Result<(), Status> {
        if let Some(grid) = &snapshot.occupancy {
            write_file(&path.join("occupancy.pb"), &grid.encode_to_vec())?;
        }
        if !snapshot.backend_checkpoint.is_empty() {
            write_file(&path.join("backend.bin"), &snapshot.backend_checkpoint)?;
        }
        if let Some(checkpoint) = &snapshot.planar_checkpoint {
            write_file(&path.join("planar.bin"), checkpoint)?;
        }
        write_file(&path.join("cloud.bin"), &snapshot.cloud.display)?;
        let fine = snapshot.cloud.fine.is_some() || snapshot.cloud.fine_archive.is_some();
        if let Some(root) = &snapshot.cloud.fine {
            let (mut file, _) = nuwax_map_chunks::archive_file(root.manifest(), &root.repository)
                .map_err(Status::internal)?;
            copy_file(&mut file, &path.join("fine.map"), None)?;
        } else if let Some(file) = &snapshot.cloud.fine_archive {
            copy_file(
                &mut file.try_clone().map_err(write_error)?,
                &path.join("fine.map"),
                None,
            )?;
        }
        if let Some(dense) = &snapshot.cloud.dense {
            let (mut file, _) = dense
                .archive(
                    snapshot
                        .occupancy
                        .as_ref()
                        .and_then(|g| g.meta.as_ref())
                        .map_or("map", |meta| meta.frame_id.as_str()),
                )
                .map_err(Status::internal)?;
            copy_file(&mut file, &path.join("dense.bin"), None)?;
        }
        for (index, (source, size)) in snapshot.scans.iter().enumerate() {
            let mut file = File::open(source).map_err(write_error)?;
            copy_file(
                &mut file,
                &path.join(format!("scans-{index}.bin")),
                Some(*size),
            )?;
        }
        let descriptor = Descriptor {
            map_from_odom: snapshot.map_from_odom,
            last_pose: snapshot.last_pose.as_ref().map(Message::encode_to_vec),
            occupancy: snapshot.occupancy.is_some(),
            backend: !snapshot.backend_checkpoint.is_empty(),
            planar: snapshot.planar_checkpoint.is_some(),
            planar_resume_unavailable_reason: snapshot.planar_resume_unavailable_reason.clone(),
            fine,
            dense: snapshot.cloud.dense.is_some(),
            scans: snapshot.scans.len(),
            voxel_size: snapshot.cloud.voxel_size,
            revision: snapshot.cloud.revision,
            source_timestamp_ns: snapshot.cloud.source_timestamp_ns,
            final_poses: snapshot.final_poses.clone(),
        };
        write_file(
            &path.join("map.json"),
            &serde_json::to_vec(&descriptor).map_err(|e| Status::internal(e.to_string()))?,
        )
    }
    fn read_descriptor(&self, name: &str) -> Result<Descriptor, Status> {
        let path = self.root.join(name).join("geometry");
        if !path.exists() {
            return Err(Status::not_found(format!("地图不存在：{name}")));
        }
        serde_json::from_slice(&read_file(&path.join("map.json"))?)
            .map_err(|e| data_error(&path.join("map.json"), e))
    }
    pub fn planar_resume_unavailable_reason(&self, name: &str) -> Result<Option<String>, Status> {
        self.read_descriptor(name)
            .map(|saved| saved.planar_resume_unavailable_reason)
    }
    pub fn load(&self, name: &str) -> Result<MapSnapshot, Status> {
        let path = self.root.join(name).join("geometry");
        let descriptor = self.read_descriptor(name)?;
        let occupancy = if descriptor.occupancy {
            Some(
                crate::proto::OccupancyGrid::decode(
                    read_file(&path.join("occupancy.pb"))?.as_slice(),
                )
                .map_err(|e| data_error(&path.join("occupancy.pb"), e))?,
            )
        } else {
            None
        };
        if let Some(grid) = &occupancy {
            let kernel = crate::mapper::proto_grid_to_kernel(grid)
                .map_err(|e| data_error(&path.join("occupancy.pb"), e))?;
            crate::persistent_occupancy::validate_checkpoint_grid(&kernel, &kernel.frame_id)
                .map_err(|e| data_error(&path.join("occupancy.pb"), e))?;
        }
        let backend_checkpoint = if descriptor.backend {
            read_file(&path.join("backend.bin"))?
        } else {
            Vec::new()
        };
        if descriptor.backend {
            crate::mapping_backend::validate_saved_checkpoint(&backend_checkpoint)
                .map_err(|e| data_error(&path.join("backend.bin"), e))?;
        }
        let planar_checkpoint = if descriptor.planar {
            let bytes = read_file(&path.join("planar.bin"))?;
            nuwax_slam_2d_runtime::archive::grid_structure(&mut std::io::Cursor::new(&bytes))
                .map_err(|e| data_error(&path.join("planar.bin"), e))?;
            Some(bytes)
        } else {
            None
        };
        let fine_archive = if descriptor.fine {
            let file = File::open(path.join("fine.map"))
                .map_err(|e| data_error(&path.join("fine.map"), e))?;
            let mut archive = nuwax_map_chunks::ArchiveReader::open(
                file.try_clone()
                    .map_err(|e| data_error(&path.join("fine.map"), e))?,
            )
            .map_err(|e| data_error(&path.join("fine.map"), e))?;
            let refs = match archive.root() {
                nuwax_map_chunks::ArchivedRoot::Current(root) => root.blocks.clone(),
            };
            for reference in refs {
                archive
                    .block(&reference)
                    .map_err(|e| data_error(&path.join("fine.map"), e))?;
            }
            Some(Arc::new(file))
        } else {
            None
        };
        let dense = if descriptor.dense {
            let mut file = File::open(path.join("dense.bin"))
                .map_err(|e| data_error(&path.join("dense.bin"), e))?;
            crate::dense_cloud::validate_archive(&mut file)
                .map_err(|e| data_error(&path.join("dense.bin"), e))?;
            let (metadata, _) = crate::dense_cloud::read_metadata(&mut file)
                .map_err(|e| data_error(&path.join("dense.bin"), e))?;
            let cloud = crate::dense_cloud::DenseCloud::from_archive(file, &metadata)
                .map_err(|e| data_error(&path.join("dense.bin"), e))?;
            cloud.snapshot(
                metadata.source_timestamp_ns,
                metadata.cloud_revision,
                &metadata.session_id,
            )
        } else {
            None
        };
        let display = read_file(&path.join("cloud.bin"))?;
        if display.len() % 12 != 0
            || display
                .chunks_exact(4)
                .any(|b| !f32::from_le_bytes(b.try_into().unwrap()).is_finite())
        {
            return Err(data_error(&path.join("cloud.bin"), "点云载荷无效"));
        }
        if descriptor.map_from_odom.iter().any(|v| !v.is_finite()) {
            return Err(data_error(&path.join("map.json"), "坐标变换无效"));
        }
        let last_pose = descriptor
            .last_pose
            .map(|bytes| {
                crate::proto::Pose::decode(bytes.as_slice())
                    .map_err(|e| data_error(&path.join("map.json"), e))
            })
            .transpose()?;
        let mut scans = Vec::new();
        for index in 0..descriptor.scans {
            let source = path.join(format!("scans-{index}.bin"));
            let mut reader = nuwax_cloud_cleanup::scan_log::ScanReader::open(&source)
                .map_err(|e| data_error(&source, e))?;
            while reader
                .next_scan()
                .map_err(|e| data_error(&source, e))?
                .is_some()
            {}
            scans.push((
                source.clone(),
                fs::metadata(&source)
                    .map_err(|e| data_error(&source, e))?
                    .len(),
            ));
        }

        Ok(MapSnapshot {
            occupancy,
            backend_checkpoint,
            planar_checkpoint,
            planar_resume_unavailable_reason: descriptor.planar_resume_unavailable_reason,
            map_from_odom: descriptor.map_from_odom,
            last_pose,
            cloud: CloudSnapshot {
                fine: None,
                fine_archive,
                dense,
                display: display.into(),
                voxel_size: descriptor.voxel_size,
                revision: descriptor.revision,
                source_timestamp_ns: descriptor.source_timestamp_ns,
            },
            scans,
            final_poses: descriptor.final_poses,
        })
    }
}
fn write_error(error: impl std::fmt::Display) -> Status {
    Status::internal(format!("地图写入失败：{error}"))
}
fn data_error(path: &Path, error: impl std::fmt::Display) -> Status {
    Status::data_loss(format!("地图文件损坏或不完整 {}：{error}", path.display()))
}
fn read_file(path: &Path) -> Result<Vec<u8>, Status> {
    fs::read(path).map_err(|e| data_error(path, e))
}
fn write_file(path: &Path, bytes: &[u8]) -> Result<(), Status> {
    let mut file = File::create(path).map_err(write_error)?;
    file.write_all(bytes)
        .and_then(|_| file.sync_all())
        .map_err(write_error)
}
fn copy_file(source: &mut File, path: &Path, size: Option<u64>) -> Result<(), Status> {
    use std::io::{Seek, SeekFrom};
    source.seek(SeekFrom::Start(0)).map_err(write_error)?;
    let mut target = File::create(path).map_err(write_error)?;
    let expected = size.unwrap_or(source.metadata().map_err(write_error)?.len());
    let copied = std::io::copy(&mut source.take(expected), &mut target).map_err(write_error)?;
    if copied != expected {
        return Err(Status::data_loss("地图快照文件截断"));
    }
    target.sync_all().map_err(write_error)
}
fn rename_flags(from: &Path, to: &Path, flags: u32) -> std::io::Result<()> {
    let from = CString::new(from.as_os_str().as_bytes())?;
    let to = CString::new(to.as_os_str().as_bytes())?;
    if unsafe {
        libc::renameat2(
            libc::AT_FDCWD,
            from.as_ptr(),
            libc::AT_FDCWD,
            to.as_ptr(),
            flags,
        )
    } == 0
    {
        Ok(())
    } else {
        Err(std::io::Error::last_os_error())
    }
}
struct TemporaryDirectory(PathBuf);
impl Drop for TemporaryDirectory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

#[cfg(test)]
#[path = "map_store_tests.rs"]
mod tests;
