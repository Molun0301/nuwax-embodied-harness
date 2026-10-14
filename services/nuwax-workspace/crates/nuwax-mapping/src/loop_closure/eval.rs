//! 回环检测评估
//! 提供 precision / recall / F1 评估指标。

/// 回环检测评估结果
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EvalResult {
    /// 真阳性
    pub true_positive: usize,
    /// 假阳性
    pub false_positive: usize,
    /// 假阴性
    pub false_negative: usize,
}

impl EvalResult {
    /// 精度
    #[must_use]
    pub fn precision(&self) -> f64 {
        if self.true_positive + self.false_positive == 0 {
            return 0.0;
        }
        self.true_positive as f64 / (self.true_positive + self.false_positive) as f64
    }

    /// 召回
    #[must_use]
    pub fn recall(&self) -> f64 {
        if self.true_positive + self.false_negative == 0 {
            return 0.0;
        }
        self.true_positive as f64 / (self.true_positive + self.false_negative) as f64
    }

    /// F1 分数
    #[must_use]
    pub fn f1(&self) -> f64 {
        let p = self.precision();
        let r = self.recall();
        if p + r == 0.0 {
            return 0.0;
        }
        2.0 * p * r / (p + r)
    }
}

/// 回环评估器
pub struct EvalLoopClosure;

impl EvalLoopClosure {
    /// 比较预测回环边与真值回环边，计算 TP/FP/FN
    ///
    /// `predicted` 与 `ground_truth` 都是 `(from_id, to_id)` 元组列表（无向，回环天然对称）。
    #[must_use]
    pub fn evaluate(predicted: &[(u64, u64)], ground_truth: &[(u64, u64)]) -> EvalResult {
        let pred_set: std::collections::HashSet<(u64, u64)> = predicted
            .iter()
            .map(|&(a, b)| if a < b { (a, b) } else { (b, a) })
            .collect();

        let truth_set: std::collections::HashSet<(u64, u64)> = ground_truth
            .iter()
            .map(|&(a, b)| if a < b { (a, b) } else { (b, a) })
            .collect();

        let tp = pred_set.intersection(&truth_set).count();
        let fp = pred_set.difference(&truth_set).count();
        let fn_ = truth_set.difference(&pred_set).count();

        EvalResult {
            true_positive: tp,
            false_positive: fp,
            false_negative: fn_,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn perfect_score() {
        let pred = vec![(1, 5), (2, 7)];
        let truth = vec![(5, 1), (7, 2)]; // 无序，eval 会归一化
        let r = EvalLoopClosure::evaluate(&pred, &truth);
        assert_eq!(r.true_positive, 2);
        assert_eq!(r.false_positive, 0);
        assert_eq!(r.false_negative, 0);
        assert!((r.precision() - 1.0).abs() < 1e-9);
        assert!((r.recall() - 1.0).abs() < 1e-9);
        assert!((r.f1() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn false_positive_count() {
        let pred = vec![(1, 5), (2, 7), (3, 9)]; // 3 预测
        let truth = vec![(1, 5)]; // 1 真值
        let r = EvalLoopClosure::evaluate(&pred, &truth);
        assert_eq!(r.true_positive, 1);
        assert_eq!(r.false_positive, 2);
        assert_eq!(r.false_negative, 0);
        assert!((r.precision() - 1.0 / 3.0).abs() < 1e-9);
        assert!((r.recall() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn false_negative_count() {
        let pred = vec![(1, 5)];
        let truth = vec![(1, 5), (2, 7)];
        let r = EvalLoopClosure::evaluate(&pred, &truth);
        assert_eq!(r.true_positive, 1);
        assert_eq!(r.false_positive, 0);
        assert_eq!(r.false_negative, 1);
    }

    #[test]
    fn empty_inputs() {
        let r = EvalLoopClosure::evaluate(&[], &[]);
        assert_eq!(r.precision(), 0.0);
        assert_eq!(r.recall(), 0.0);
        assert_eq!(r.f1(), 0.0);
    }
}
