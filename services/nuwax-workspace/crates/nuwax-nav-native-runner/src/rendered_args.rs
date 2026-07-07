//! `rendered_args()` 实现(严格按方案 §3.3 第 3 小节规则)
//!
//! 规则:
//! 1. `topics` 中每对 `(port, topic)` 渲染为 `--{port} {topic}`
//! 2. `args` 原样追加(但禁止空字符串,validate 已检查)
//! 3. 启动前对 spec.validate 校验
//! 4. 参数顺序稳定:`BTreeMap` topic 按 port 字典序,然后追加 `args`

use crate::spec::NativeProgramSpec;

/// 渲染最终 CLI 参数(不含 argv[0])
///
/// 调用方需先 `spec.validate()`。
#[must_use]
pub fn render_args(spec: &NativeProgramSpec) -> Vec<String> {
    let mut out = Vec::with_capacity(spec.topics.len() * 2 + spec.args.len());
    for (port, topic) in &spec.topics {
        out.push(format!("--{port}"));
        out.push(topic.clone());
    }
    out.extend(spec.args.iter().cloned());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_base::native::RestartPolicy;
    use std::collections::BTreeMap;
    use std::path::PathBuf;

    fn make_spec() -> NativeProgramSpec {
        let mut topics = BTreeMap::new();
        topics.insert("odometry".into(), "/odom".into());
        topics.insert("registered_scan".into(), "/scan".into());
        topics.insert("goal".into(), "/goal".into());
        NativeProgramSpec {
            name: "far_planner".into(),
            executable: PathBuf::from("/bin/true"),
            topics,
            args: vec!["--rate".into(), "10".into(), "--debug".into()],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        }
    }

    #[test]
    fn topics_are_lexicographic_by_port() {
        let s = make_spec();
        let args = render_args(&s);
        // 第一个 topic 应是 goal (字典序最小)
        assert_eq!(args[0], "--goal");
        assert_eq!(args[1], "/goal");
        assert_eq!(args[2], "--odometry");
        assert_eq!(args[3], "/odom");
        assert_eq!(args[4], "--registered_scan");
        assert_eq!(args[5], "/scan");
    }

    #[test]
    fn args_appear_after_topics_in_order() {
        let s = make_spec();
        let args = render_args(&s);
        let after_topics = &args[6..];
        assert_eq!(
            after_topics,
            &[
                "--rate".to_string(),
                "10".to_string(),
                "--debug".to_string()
            ]
        );
    }

    #[test]
    fn empty_topics_only_args() {
        let mut s = make_spec();
        s.topics.clear();
        let args = render_args(&s);
        assert_eq!(
            args,
            vec![
                "--rate".to_string(),
                "10".to_string(),
                "--debug".to_string()
            ]
        );
    }

    #[test]
    fn empty_spec_renders_empty() {
        let mut s = make_spec();
        s.topics.clear();
        s.args.clear();
        let args = render_args(&s);
        assert!(args.is_empty());
    }
}
