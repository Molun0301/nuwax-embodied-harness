//! Typed resource outcomes for owners; compatibility APIs retain their text.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlanningFailure {
    /// The configured state/expansion allowance ended before a route was proved.
    BudgetExhausted(String),
    /// Materialized/search domain ended before physical reachability was settled.
    SearchDomainLimited(String),
    /// Existing validation, cancellation or geometric diagnostic.
    Other(String),
}
impl std::fmt::Display for PlanningFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BudgetExhausted(message) => write!(formatter, "BudgetExhausted: {message}"),
            Self::SearchDomainLimited(message) => {
                write!(formatter, "SearchDomainLimited: {message}")
            }
            Self::Other(message) => formatter.write_str(message),
        }
    }
}
impl std::error::Error for PlanningFailure {}
impl From<String> for PlanningFailure {
    fn from(message: String) -> Self {
        Self::Other(message)
    }
}
impl From<&str> for PlanningFailure {
    fn from(message: &str) -> Self {
        Self::Other(message.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{SearchLimits, Tomogram};
    #[test]
    fn producer_status_survives_without_parsing_and_string_wrappers_keep_diagnostics() {
        let map = Tomogram::from_columns(0.1, [], 0.4, 0.0, 0.1, 0.1, 20.0).unwrap();
        let start = [0.05, 0.05, 0.0];
        let goal = [0.55, 0.05, 0.0];
        let reject = |_: [f64; 3], _: [f64; 3]| false;
        let typed = map
            .plan_bounded_with_segment_filter_and_limits_typed(
                start,
                goal,
                0.0,
                &[],
                None,
                None,
                &[],
                &reject,
                SearchLimits { max_states: 0 },
            )
            .unwrap_err();
        assert!(matches!(typed, PlanningFailure::BudgetExhausted(_)));
        let text = map
            .plan_bounded_with_segment_filter_and_limits(
                start,
                goal,
                0.0,
                &[],
                None,
                None,
                &[],
                &reject,
                SearchLimits { max_states: 0 },
            )
            .unwrap_err();
        assert_eq!(typed.to_string(), text);
        let mut cropped = map;
        cropped.window = Some(([0.0, 0.0], 0.2));
        let outside = [0.5, 0.05, 0.0];
        let typed = cropped
            .plan_bounded_with_limits_typed(
                outside,
                goal,
                0.0,
                &[],
                None,
                None,
                &[],
                SearchLimits::default(),
            )
            .unwrap_err();
        assert!(matches!(typed, PlanningFailure::SearchDomainLimited(_)));
        let text = cropped
            .plan_bounded_with_limits(
                outside,
                goal,
                0.0,
                &[],
                None,
                None,
                &[],
                SearchLimits::default(),
            )
            .unwrap_err();
        assert_eq!(typed.to_string(), text);
        assert!(
            matches!(
                PlanningFailure::from("BudgetExhausted: an unrelated validation message"),
                PlanningFailure::Other(_)
            ),
            "a diagnostic prefix can never manufacture a resource outcome"
        );
    }
}
