#!/usr/bin/env python3
"""Generate a standalone test-only experiment from real relocalization modules.

No production file is changed. Uses frozen copied source and exact replacements,
with an uninstrumented timing binary and a separately instrumented attribution one.
Compile commands and input bounds are documented in the associated audit report.
"""
import argparse
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[2]
src = root / 'services/nuwax-workspace/crates/nuwax-mapping/src/relocalization'
a.output.mkdir(parents=True, exist_ok=True)

def replace_once(text, old, new):
    assert text.count(old) == 1, (old[:100], text.count(old))
    return text.replace(old, new)

for name in ['nearest', 'features', 'ransac', 'relocalize']:
    content = (src / (name + '.rs')).read_text()
    (a.output / ('original_' + name + '.rs')).write_text(content)
    if name == 'ransac':
        content = replace_once(content, 'fn evaluate_with_tree(', 'pub(crate) fn evaluate_with_tree(')
    if name == 'nearest':
        content = replace_once(content, '    points: Vec<IndexedPoint>,\n}', '    points: Vec<IndexedPoint>,\n    #[cfg(probe_counts)]\n    probe_bytes: usize,\n}')
        content = replace_once(content, '    pub(crate) fn new(points: &[Point3<f64>]) -> Self {', '''    pub(crate) fn new(points: &[Point3<f64>]) -> Self {
        #[cfg(probe_counts)] let started = std::time::Instant::now();''')
        content = replace_once(content, '        Self { points: indexed }', '''        #[cfg(probe_counts)] let probe_bytes = indexed.capacity() * std::mem::size_of::<IndexedPoint>();
        #[cfg(probe_counts)] crate::stats::built(points.as_ptr() as usize, started.elapsed().as_nanos(), probe_bytes);
        Self { points: indexed, #[cfg(probe_counts)] probe_bytes }''')
        content += '''\n#[cfg(probe_counts)] impl Drop for NearestPointIndex { fn drop(&mut self) { crate::stats::freed(self.probe_bytes); } }\n'''
    if name == 'relocalize':
        content = replace_once(content, '    let mut maximum_feature_matches = 0;', '''    #[cfg(probe_counts)] crate::stats::fine(fine_target.as_ptr() as usize);
    let mut maximum_feature_matches = 0;''')
        (a.output / 'baseline.rs').write_text(content)
        candidate = replace_once(content, 'use super::ransac::{evaluate_alignment,', 'use super::ransac::{evaluate_with_tree, evaluate_alignment,')
        candidate = replace_once(candidate, '''        consider_candidate(
            &fine_source,
            &fine_target,
            ransac.transformation,''', '''        // Test-only candidate: one tree for this exact pair, dropped at scale end.
        let pair_tree = NearestPointIndex::new(&fine_target);
        consider_candidate(
            &fine_source,
            &pair_tree,
            ransac.transformation,''')
        candidate = replace_once(candidate, '''        consider_candidate(
            &fine_source,
            &fine_target,
            ransac.transformation * centroid_yaw_flip(&fine_source),''', '''        consider_candidate(
            &fine_source,
            &pair_tree,
            ransac.transformation * centroid_yaw_flip(&fine_source),''')
        candidate = replace_once(candidate, '''fn consider_candidate(
    source: &[Point3<f64>],
    target: &[Point3<f64>],''', '''fn consider_candidate(
    source: &[Point3<f64>],
    target: &NearestPointIndex,''')
        candidate = replace_once(candidate, '    let score = evaluate_alignment(source, target, transformation, threshold);', '    let score = evaluate_with_tree(source, target, transformation, threshold);')
        (a.output / 'candidate.rs').write_text(candidate)
    else:
        (a.output / (name + '.rs')).write_text(content)

# Keep production compute orchestration intact except selecting the candidate
# global pipeline and adapting its structurally identical private result type.
coordinator = (root / 'services/geometry/nuwax_geometry_runtime/src/correction_coordinator.rs').read_text()
start = coordinator.index('    fn run_relocalization_compute(')
end = coordinator.index('    type CorrectionWorkerJoin', start)
compute = coordinator[start:end]
(a.output / 'original_compute.rs').write_text(compute)
compute = replace_once(compute, '        source: &[Point3<f64>],', '        reuse: bool,\n        source: &[Point3<f64>],')
compute = replace_once(compute, '        let result = relocation_pipeline_with_config(source, target, config);', '''        let result = if reuse {
            let r = candidate::relocation_pipeline_with_config(source, target, candidate::RelocalizationConfig::default());
            RelocalizationResult { registration: IcpResult { transformation:r.registration.transformation,
                mse:r.registration.mse, iterations:r.registration.iterations, converged:r.registration.converged },
                feature_matches:r.feature_matches, ransac_inliers:r.ransac_inliers, fitness:r.fitness }
        } else { relocation_pipeline_with_config(source, target, config) };''')
(a.output / 'compute.rs').write_text(compute)
(a.output / 'main.rs').write_text((Path(__file__).with_name('relocalization_pair_reuse_probe.rs')).read_text())
template=(a.output / 'main.rs').read_text()
contracts=template.replace('fn main() {', 'fn unused_timing_main() {')
contracts += r"""
fn main() {
    let base=base_source();
    let mut cases=vec![(Vec::new(),Vec::new()),(base.clone(),Vec::new()),
        (vec![Point3::origin()],base.clone()),(base.clone(),vec![Point3::origin()]),
        (vec![Point3::origin();8],vec![Point3::origin();8])];
    let mut duplicate=base.clone();duplicate.extend(base.iter().copied());
    cases.push((duplicate.clone(),duplicate));
    let mut signed=base.clone();signed.extend([Point3::new(-0.0,0.0,0.0),Point3::new(0.0,-0.0,0.0)]);
    cases.push((signed.clone(),signed));
    for v in [f64::NAN,f64::INFINITY,f64::NEG_INFINITY] {
        let mut invalid=base.clone();invalid.push(Point3::new(v,0.0,1.0));
        cases.push((invalid.clone(),base.clone()));cases.push((base.clone(),invalid));
    }
    let mut reversed=base.clone();reversed.reverse();cases.push((base.clone(),reversed));
    let symmetric=(0..4).flat_map(|x|(0..4).flat_map(move|y|(0..3).map(move|z|Point3::new(x as f64*0.5,y as f64*0.5,z as f64*0.5)))).collect::<Vec<_>>();
    cases.push((symmetric.clone(),symmetric));
    for (i,(source,target)) in cases.iter().enumerate() {
        let a=run_relocalization_compute(false,source,target,Isometry3::identity(),Isometry3::identity());
        let b=run_relocalization_compute(true,source,target,Isometry3::identity(),Isometry3::identity());
        assert_eq!(fingerprint(&a),fingerprint(&b),"case {i}");
        // Also force the exact global functions, bypassing local success.
        let a=baseline::relocation_pipeline_with_config(source,target,baseline::RelocalizationConfig::default());
        let r=candidate::relocation_pipeline_with_config(source,target,candidate::RelocalizationConfig::default());
        let b=RelocalizationResult {registration:IcpResult {transformation:r.registration.transformation,mse:r.registration.mse,iterations:r.registration.iterations,converged:r.registration.converged},feature_matches:r.feature_matches,ransac_inliers:r.ransac_inliers,fitness:r.fitness};
        assert_eq!(fingerprint(&RelocalizationComputeOutput{result:a,correction:None}),fingerprint(&RelocalizationComputeOutput{result:b,correction:None}),"global case {i}");
    }
    println!("{{\"contract_cases\":{},\"path_comparisons\":{},\"exact\":true}}",cases.len(),cases.len()*2);
}
"""
(a.output / 'contracts.rs').write_text(contracts)
print(a.output)
