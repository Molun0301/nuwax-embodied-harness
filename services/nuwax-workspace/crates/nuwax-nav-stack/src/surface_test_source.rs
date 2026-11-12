// 测试数据工具：保留点云配方，发布前由真实Writer生成同机精细根。
use super::*;
use std::collections::BTreeSet;
use nuwax_map_chunks::{Domain,Manifest,Repository,Writer,Transfer};

pub(super) struct FineFixtureSource {
    repository: Repository,
    geometry: nuwax_nav_mls_planner::mls_planner::Config,
    writer: Option<Writer>,
    previous: Option<(Arc<Manifest>,Vec<(f32,f32,f32)>)>,
}
impl FineFixtureSource {
    pub(super) fn new(geometry:nuwax_nav_mls_planner::mls_planner::Config) -> Self {
        Self {repository:Repository::temporary().unwrap(),geometry,writer:None,previous:None}
    }
    pub(super) fn message(&mut self, message: &schema::NavigationMap3D) -> schema::NavigationMap3D {
        if message.cleared || message.fine.is_some() {return message.clone();}
        let domain=Domain {session:message.session_id.clone()};
        let size=message.source_voxel_size;
        let points=admitted_cloud_points(&message.cloud);
        let keys=|points:&[(f32,f32,f32)]|points.iter().map(|p|[p.0,p.1,p.2].map(|v|(v/size).floor() as i32)).collect::<BTreeSet<_>>();
        let cells=keys(&points);
        let previous=self.previous.take().filter(|(root,_)|root.domain==domain && root.voxel_size_m.to_bits()==size.to_bits());
        if previous.is_none() {self.writer=Some(Writer::new(self.repository.clone(),size,4*1024*1024).unwrap());}
        let writer=self.writer.as_mut().unwrap();
        if previous.as_ref().is_none_or(|(_,old)|*old!=points) {
            let old=previous.as_ref().map_or_else(BTreeSet::new,|(_,old)|keys(old));
            writer.remove(old.difference(&cells).copied()).unwrap();
            writer.observe(cells.difference(&old).copied(),message.source_timestamp_ns as u64).unwrap();
            let surface=SurfaceSnapshot::from_points(&points,self.geometry.clone(),size).unwrap();
            let support: BTreeSet<_>=surface.cell_keys().collect();
            let coarse=self.geometry.voxel_size;
            let mut columns=std::collections::BTreeMap::<[i32;2],std::collections::BTreeMap<i32,std::collections::BTreeMap<i32,[u32;8]>>>::new();
            for p in &points {
                let p=[p.0,p.1,p.2];let key=p.map(|v|(v/size).floor() as i32);
                let column=columns.entry([key[0],key[1]]).or_default();
                let c=p.map(|v|(v/coarse).floor() as i32);
                if support.contains(&(c[0],c[1],c[2])) {
                    let bin=((f64::from(p[2])/f64::from(size)-f64::from(key[2]))*8.).floor() as usize;
                    column.entry(c[2]).or_default().entry(key[2]).or_default()[bin]+=1;
                }
            }
            for (xy,layers) in columns {
                let heightfields=layers.into_values().map(|counts|nuwax_map_chunks::HeightfieldSurface {
                    column:nuwax_map_chunks::heightfield_observations::HeightfieldColumn::from_counts(xy,size,counts).unwrap().unwrap(),samples:vec![]
                }).collect::<Vec<_>>();
                writer.replace_ground_column(xy,&nuwax_map_chunks::ColumnGroundRecords {classified_empty:heightfields.is_empty(),heightfields}).unwrap();
            }
        }
        let revision=previous.as_ref().map_or(1,|(root,_)|root.revision+1);
        let root=writer.snapshot(domain,revision,message.source_timestamp_ns as u64).unwrap();
        let mut fine=message.clone();
        fine.fine=Some(Transfer {root:root.encode().unwrap(),repository_path:self.repository.path().to_string_lossy().into_owned()});
        fine.cloud.width=0;fine.cloud.row_step=0;fine.cloud.data.clear();
        self.previous=Some((root,points));fine
    }
}

pub(super) struct FixtureSurfaceCache {
    pub(super) inner: crate::surface_map::SurfaceMapCache,
    pub(super) source: FineFixtureSource,
}
impl FixtureSurfaceCache {
    pub(super) fn new(config:nuwax_nav_mls_planner::mls_planner::Config)->Self {
        Self {inner:crate::surface_map::SurfaceMapCache::new(config.clone()),source:FineFixtureSource::new(config)}
    }
    pub(super) fn configured(config:&SurfaceNavigationConfig)->Self {
        Self {inner:crate::surface_map::SurfaceMapCache::configured(config),source:FineFixtureSource::new(config.geometry.clone())}
    }
    pub(super) fn build(&mut self,message:&schema::NavigationMap3D)->Result<Arc<SurfaceSnapshot>,String> {
        self.inner.build(&self.source.message(message))
    }
}
impl std::ops::Deref for FixtureSurfaceCache {
    type Target=crate::surface_map::SurfaceMapCache;
    fn deref(&self)->&Self::Target {&self.inner}
}
impl std::ops::DerefMut for FixtureSurfaceCache {
    fn deref_mut(&mut self)->&mut Self::Target {&mut self.inner}
}
/// 解码边界已确认字段、布局与有限坐标；此处只展开坐标，不重复校验协议。
pub(crate) fn admitted_cloud_points(cloud: &schema::PointCloud2) -> Vec<(f32, f32, f32)> {
    let axes = ["x", "y", "z"].map(|name| {
        let field = cloud.fields.iter().find(|field| field.name == name).unwrap();
        (field.offset as usize, field.datatype)
    });
    let mut points = Vec::with_capacity(cloud.width as usize * cloud.height as usize);
    for row in 0..cloud.height as usize {
        for column in 0..cloud.width as usize {
            let base = row * cloud.row_step as usize + column * cloud.point_step as usize;
            let xyz = axes.map(|(offset, datatype)| {
                let start = base + offset;
                if datatype == 7 {
                    let bytes = cloud.data[start..start + 4].try_into().unwrap();
                    if cloud.is_bigendian { f32::from_be_bytes(bytes) } else { f32::from_le_bytes(bytes) }
                } else {
                    let bytes = cloud.data[start..start + 8].try_into().unwrap();
                    (if cloud.is_bigendian { f64::from_be_bytes(bytes) } else { f64::from_le_bytes(bytes) }) as f32
                }
            });
            points.push((xyz[0], xyz[1], xyz[2]));
        }
    }
    points
}


#[test]
    fn admitted_coordinates_preserve_endianness_offsets_and_padded_rows() {
        for big_endian in [false, true] {
            for datatype in [7, 8] {
                let bytes = if datatype == 7 { 4 } else { 8 };
                let mut cloud = schema::PointCloud2 {
                    width: 2, height: 2, point_step: bytes * 4, row_step: bytes * 9,
                    is_bigendian: big_endian,
                    fields: ["z", "x", "y"].iter().enumerate().map(|(index, name)| schema::PointField {
                        name: (*name).into(), offset: index as i32 * bytes, datatype, count: 1,
                    }).collect(),
                    ..Default::default()
                };
                cloud.data = vec![0; (cloud.row_step * cloud.height) as usize];
                for index in 0..4 {
                    let base = index / 2 * cloud.row_step as usize + index % 2 * cloud.point_step as usize;
                    for (axis, offset) in [(0, bytes), (1, bytes * 2), (2, 0)] {
                        let value = (index * 3 + axis) as f64;
                        let encoded = if datatype == 7 {
                            if big_endian { (value as f32).to_be_bytes() } else { (value as f32).to_le_bytes() }.to_vec()
                        } else {
                            if big_endian { value.to_be_bytes() } else { value.to_le_bytes() }.to_vec()
                        };
                        let start = base + offset as usize;
                        cloud.data[start..start + bytes as usize].copy_from_slice(&encoded);
                    }
                }
                assert_eq!(admitted_cloud_points(&cloud), vec![(0.,1.,2.),(3.,4.,5.),(6.,7.,8.),(9.,10.,11.)]);
            }
        }
    }

// 既有图正确性断言仍在测试工具的完整产物上执行，生产导航只交付列。
pub(super) fn full_surface_fixture(surface:&SurfaceSnapshot)->SurfaceSnapshot {
    let (size,columns)=surface.navigation_source_columns(None);
    let mut points=Vec::new();
    for (column,bands) in columns {
        let xy=column.map(|v|(f64::from(v)+0.5)*size);
        for [low,high] in bands {
            let count=((high-low)/size).round() as usize;
            for i in 0..count {points.push((xy[0] as f32,xy[1] as f32,(low+(i as f64+0.5)*size) as f32));}
        }
    }
    let mut builder=nuwax_nav_mls_planner::SurfaceMapBuilder::configured(surface.config().clone(),surface.body_envelope().cloned());
    let graph=builder.build(&points,surface.source_voxel_size_m()).unwrap().snapshot
        .with_observations(surface.observations().cloned());
    graph.with_retained_virtual_obstacles_from(surface).map_or_else(||graph.clone(),Result::unwrap)
}
