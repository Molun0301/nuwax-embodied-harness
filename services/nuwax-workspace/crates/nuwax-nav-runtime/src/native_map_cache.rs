//! 全局静态列与临时叠加由原规划拥有方串行更新。
use super::*;
use nuwax_nav_pct_native::Planner;
use nuwax_pct_tomography::{ColumnCache, ColumnUpdate, TomographyConfig};

#[derive(Default, PartialEq)]
struct MlsColumnSource {
    bands: Vec<[f64; 2]>,
    points: Vec<[f64; 3]>,
    ground: Vec<(Option<f64>, Vec<i32>)>,
    classified_empty: bool,
}

/// 静态地图和原生结构同寿命，近期帧不撤销它们。
pub struct CachedPctPlanner {
    source: Option<SurfaceSnapshot>,
    terrain: PctTerrainSource,
    config: Option<Config>,
    tomography: ColumnCache,
    mls: BTreeMap<[i32; 2], MlsColumnSource>,
    planner: Option<Planner>,
}

impl CachedPctPlanner {
    pub fn new(terrain: PctTerrainSource) -> Self {
        Self {
            source: None,
            terrain,
            config: None,
            tomography: ColumnCache::default(),
            mls: BTreeMap::new(),
            planner: None,
        }
    }

    pub fn config(&self) -> &Config {
        self.config.as_ref().expect("全局输入已准备")
    }
    pub fn planner(&mut self) -> &mut Planner {
        self.planner.as_mut().expect("全局输入已准备")
    }

    /// 只替换变化列和当次叠加；调用方持有的搜索对象继续复用。
    pub fn update(
        &mut self,
        map: &SurfaceSnapshot,
        offset: f64,
        boundary: &[[f64; 2]],
    ) -> Result<(), String> {
        let (resolution, _) = map.navigation_source_columns(None);
        let config = pct_config(map, offset, boundary, resolution);
        let reset = self.config.as_ref() != Some(&config);
        if reset {
            self.tomography = ColumnCache::default();
            self.mls.clear();
            self.planner = Some(Planner::from_cells(config.clone(), &[], &[])?);
            self.source = None;
        }
        let static_changed = self
            .source
            .as_ref()
            .is_none_or(|old| !old.same_navigation_static_geometry(map));
        let (recent_ground, _) = map.navigation_recent_samples();
        let mut overlay_cells = Vec::new();
        let mut overlay_bands = Vec::new();
        let updates = match self.terrain {
            PctTerrainSource::Tomography => {
                if static_changed {
                    let changed = self.source.as_ref().and_then(|old| static_columns_since(old, map))
                        .and_then(|columns| tomography_column_points(map, &config, &columns)
                            .map(|points| (columns, points)));
                    match changed {
                        Some((columns, points)) => self.tomography.replace_static_columns(&columns, &points, resolution)?,
                        None => self.tomography.replace_static(&tomography_points(map, &config), resolution)?,
                    }
                }
                let ground: Vec<_> = recent_ground.iter().map(|p| p.map(|v| v as f32)).collect();
                self.tomography.update(
                    &ground,
                    &TomographyConfig {
                        resolution,
                        point_resolution: config.point_resolution,
                        slice_h0: 0.,
                        slice_dh: config.robot_height.max(config.point_resolution),
                    },
                )?
            }
            PctTerrainSource::Mls => {
                for p in &recent_ground {
                    let [x, y] = [p[0], p[1]].map(|v| (v / resolution).floor() as i32);
                    overlay_cells.push(Cell {
                        x,
                        y,
                        ground: p[2],
                        ceiling: f64::INFINITY,
                    });
                    overlay_bands.push(Band {
                        x,
                        y,
                        low: p[2],
                        high: p[2],
                    });
                }
                if static_changed {
                    let measured = map.ground_evidence()
                        == nuwax_nav_mls_planner::surface::GroundEvidence::MeasuredSurface;
                    let (_, source_columns) = map.navigation_source_columns(None);
                    let mut next: BTreeMap<_, _> = source_columns
                        .map(|(column, bands)| {
                            let xy = column.map(|v| (f64::from(v) + 0.5) * resolution);
                            let ground = map
                                .observations()
                                .filter(|_| measured)
                                .into_iter()
                                .flat_map(|o| o.ground_records(column))
                                .map(|r| (r.height_at(xy), r.members().collect()))
                                .collect();
                            (
                                column,
                                MlsColumnSource {
                                    bands,
                                    ground,
                                    points: Vec::new(),
                                    classified_empty: map
                                        .observations()
                                        .is_some_and(|o| o.classified_empty_ground(column)),
                                },
                            )
                        })
                        .collect();
                    if let Some(points) = map.navigation_measured_points().filter(|_| measured) {
                        for &point in points {
                            let key = [point[0], point[1]].map(|v| (v / resolution).floor() as i32);
                            next.entry(key).or_default().points.push(point);
                        }
                    }
                    let keys: BTreeSet<_> = self
                        .mls
                        .keys()
                        .chain(next.keys())
                        .copied()
                        .filter(|key| self.mls.get(key) != next.get(key))
                        .collect();
                    let input = from_surface_selected(map, None, Some(&keys))?;
                    let mut geometry: BTreeMap<_, (Vec<Cell>, Vec<Band>)> = keys
                        .iter()
                        .map(|&key| (key, (Vec::new(), Vec::new())))
                        .collect();
                    for cell in input.cells {
                        geometry.entry([cell.x, cell.y]).or_default().0.push(cell);
                    }
                    for band in input.bands {
                        geometry.entry([band.x, band.y]).or_default().1.push(band);
                    }
                    self.mls = next;
                    geometry
                        .into_iter()
                        .map(|(column, (cells, bands))| ColumnUpdate {
                            column,
                            cells,
                            bands,
                        })
                        .collect()
                } else {
                    Vec::new()
                }
            }
        };
        overlay_bands.extend(pct_overlay_bands(map, &config));
        let planner = self.planner.as_mut().expect("原生规划器已创建");
        // 先撤回旧叠加，变化列替换后再合入本次临时输入。
        planner.replace_overlay(&[], &[]);
        let mut columns = Vec::with_capacity(updates.len());
        let mut cells = Vec::new();
        let mut bands = Vec::new();
        for update in updates {
            columns.push(update.column);
            cells.extend(update.cells);
            bands.extend(update.bands);
        }
        planner.replace_columns(&columns, &cells, &bands);
        planner.replace_overlay(&overlay_cells, &overlay_bands);
        self.source = Some(map.clone());
        self.config = Some(config);
        Ok(())
    }
}

/// 表面图登记的、从上次静态几何到本次之间变化的层析列；登记记录不覆盖上次身份时为 `None`。
fn static_columns_since(old: &SurfaceSnapshot, map: &SurfaceSnapshot) -> Option<BTreeSet<[i32; 2]>> {
    map.static_columns()?.columns_since(old.static_columns()?.id())
}
