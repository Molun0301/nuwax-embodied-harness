use super::*;
use crate::heightfield_observations::HeightfieldColumn;

fn column(xy: [i32; 2], levels: &[i32], stamp: u64) -> ColumnGroundRecords {
    let source = Arc::new(LayerSource::Lidar {
        backend: 1, runtime_generation: 1, sensor_frame_id: "列保留测试".into(),
    });
    ColumnGroundRecords { classified_empty: false, heightfields: vec![HeightfieldSurface {
        column: HeightfieldColumn::from_counts(xy, 0.05,
            levels.iter().map(|&z| (z, [1, 0, 0, 0, 0, 0, 0, 0])).collect()).unwrap().unwrap(),
        samples: levels.iter().enumerate().map(|(i, &z)| LayerSample {
            source: source.clone(), point: [(f64::from(xy[0]) + 0.5) * f64::from(0.05_f32),
                (f64::from(xy[1]) + 0.5) * f64::from(0.05_f32), (f64::from(z) + 0.5) * f64::from(0.05_f32)],
            source_sequence: stamp, source_timestamp_ns: stamp, point_index: i as u32,
        }).collect(),
    }] }
}
fn compare(w: &mut Writer, rows: &[(i32, i32, i32)]) -> BTreeMap<[i32; 2], ColumnGroundRecords> {
    let columns: Vec<_> = rows.iter().flat_map(|&(x, a, b)| (a..=b).map(move |y| [x, y])).collect();
    let expected = w.ground_columns(columns).unwrap();
    let actual = w.ground_columns_in_rows(rows).unwrap().clone();
    assert_eq!(actual, expected);
    actual
}
#[test]
fn retained_columns_keep_samples_withdraw_members_restore_and_release_outside_rows() {
    let repo = Repository::temporary().unwrap();
    let mut writer = Writer::new(repo.clone(), 0.05, 4 * 1024 * 1024).unwrap();
    for xy in [[-1, 0], [0, 0], [32, 0]] {
        let value = column(xy, &[-1, 0, 33], 1);
        writer.observe([-1, 0, 33].map(|z| [xy[0], xy[1], z]), 1).unwrap();
        writer.replace_ground_column(xy, &value).unwrap();
    }
    let first = writer.snapshot(Domain { session: "列保留".into() }, 1, 1).unwrap();
    let rows = [(-1, 0, 1), (0, 0, 1), (32, 0, 1)];
    compare(&mut writer, &rows);
    let bins = writer.retained_ground_columns[&[0, 0]].heightfields[0].column.members().next().unwrap().1.as_ptr();
    writer.flush().unwrap();
    compare(&mut writer, &rows);
    assert_eq!(bins, writer.retained_ground_columns[&[0, 0]].heightfields[0].column.members().next().unwrap().1.as_ptr());
    assert_eq!(writer.retained_ground_columns[&[0, 0]].heightfields[0].samples.len(), 3);
    let replacement = column([0, 0], &[0, 33], 2);
    writer.replace_ground_column([0, 0], &replacement).unwrap();
    compare(&mut writer, &rows);
    writer.remove([[0, 0, 33]]).unwrap();
    let values = compare(&mut writer, &rows);
    assert_eq!(values[&[0, 0]].heightfields[0].column.members().map(|v| v.0).collect::<Vec<_>>(), vec![0]);
    writer.remove([[0, 0, 0]]).unwrap();
    compare(&mut writer, &rows);
    writer.restore(&first).unwrap();
    assert!(writer.retained_ground_columns.is_empty());
    compare(&mut writer, &rows);
    compare(&mut writer, &[(32, 0, 0)]);
    assert_eq!(writer.retained_ground_columns.keys().copied().collect::<Vec<_>>(), vec![[32, 0]]);
    compare(&mut writer, &rows);
}

#[test]
fn unmaterialized_empty_declaration_stays_absent_until_an_occupied_column_exists() {
    let mut writer = Writer::new(Repository::temporary().unwrap(), 0.05, 4 * 1024 * 1024).unwrap();
    let rows = [(0, 0, 0)];
    let empty = ColumnGroundRecords { classified_empty: true, heightfields: vec![] };
    compare(&mut writer, &rows);
    writer.replace_ground_column([0, 0], &empty).unwrap();
    assert!(compare(&mut writer, &rows).is_empty());
    writer.observe([[0, 0, 0]], 1).unwrap();
    assert!(compare(&mut writer, &rows).is_empty());
    writer.replace_ground_column([0, 0], &empty).unwrap();
    assert!(compare(&mut writer, &rows)[&[0, 0]].classified_empty);
}
