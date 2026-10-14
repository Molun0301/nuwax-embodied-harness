use super::*;

fn map(columns: impl IntoIterator<Item=([i32;2],Vec<[f64;2]>)>, width:f64, step:f64) -> Tomogram {
    Tomogram::from_columns_with_quantum(0.1,0.05,columns,0.4,width,step,step,0.0,width).unwrap()
}
#[test]
fn calibrated_height_is_carried_across_flat_known_and_unknown_motion() {
    let unknown=map([],0.1,0.0);
    let from=[0.05,0.05,0.12];
    assert_eq!(unknown.project_reference_motion(from,[0.45,0.05],&[]).unwrap(),[0.45,0.05,0.12]);
    let flat=map([([0,0],vec![[0.0,0.1]]),([4,0],vec![[0.0,0.1]])],0.0,0.0);
    let end=flat.project_reference_motion(from,[0.45,0.05],&[]).unwrap();
    assert!((end[2]-from[2]).abs()<1e-12,"{end:?}");
    assert!((flat.cells[0].ground-0.075).abs()<1e-12,"factual reference is unchanged");
}
#[test]
fn factual_steps_transport_only_the_terrain_delta() {
    let stepped=map((0..4).map(|x|([x,0],vec![[x as f64*0.05,x as f64*0.05+0.1]])),0.0,0.06);
    let end=stepped.project_reference_motion([0.05,0.05,0.12],[0.35,0.05],&[]).unwrap();
    assert!((end[2]-0.27).abs()<1e-12,"{end:?}");
    let steep=map([([0,0],vec![[0.0,0.1]]),([1,0],vec![[0.4,0.5]])],0.0,0.06);
    assert!(steep.project_reference_motion([0.05,0.05,0.12],[0.15,0.05],&[]).is_err());
    let drop=map([([0,0],vec![[0.0,0.1]]),([1,0],vec![[-0.4,-0.3]])],0.0,0.06);
    assert!(drop.project_reference_motion([0.05,0.05,0.12],[0.15,0.05],&[]).is_err());
}
#[test]
fn source_band_association_preserves_zero_step_frozen_style_calibration() {
    let floor=map([([0,1],vec![[-1.0,-0.9]])],0.1,0.0);
    let from=[0.05,0.05,-0.93715189];
    let end=floor.project_reference_motion(from,[0.08,0.05],&[]).unwrap();
    assert!((end[2]-from[2]).abs()<1e-12);
    assert!((floor.calibrated_reference_height(from).unwrap()+0.925).abs()<1e-12);
    assert!((floor.cells[0].ground+0.925).abs()<1e-12);
    let wall=map([([0,1],vec![[-1.0,0.2]])],0.1,0.0);
    assert!(wall.project_reference_motion(from,[0.08,0.05],&[]).is_err(),"band membership cannot exempt a tall wall");
    let unrelated=map([([0,1],vec![[-1.3,-1.2]])],0.1,0.0);
    assert_eq!(unrelated.calibrated_reference_height(from),Some(from[2]),"cannot acquire a neighboring lower layer");
}
#[test]
fn known_reference_motion_checks_the_whole_footprint_and_actual_carried_top() {
    let columns=(0..5).map(|x|([x,0],vec![[-0.1,0.0]]))
        .chain([([2,1],vec![[0.1,0.5]])]);
    let wall=map(columns,0.08,0.0);
    assert!(wall.segment_clear([0.05,0.05,0.0],[0.45,0.05,0.0],&[]),"global guide semantics are unchanged");
    assert!(wall.project_reference_motion([0.05,0.05,0.0],[0.45,0.05],&[]).is_err());
    let low_top=map((0..5).map(|x|([x,0],vec![[-0.1,0.0],[0.41,0.6]])),0.0,0.0);
    assert!(low_top.project_reference_motion([0.05,0.05,0.02],[0.45,0.05],&[]).is_err(),"the carried body top is 0.42, not the mean-reference top 0.4");
}

#[test]
fn observed_slope_adds_its_delta_without_losing_live_height_bias() {
    let ramp=Tomogram::from_columns_with_quantum(0.1,0.05,
        (0..5).map(|x|([x,0],vec![[x as f64*0.03,x as f64*0.03+0.1]])),
        0.4,0.0,0.0,0.0,20.0,0.0).unwrap();
    let end=ramp.project_reference_motion([0.05,0.05,0.12],[0.45,0.05],&[]).unwrap();
    assert!((end[2]-0.24).abs()<1e-12,"{end:?}");
}
