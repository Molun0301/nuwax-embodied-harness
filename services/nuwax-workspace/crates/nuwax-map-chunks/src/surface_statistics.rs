//! Pure observation statistics math, reused by bounded capture-layer records.
//!
//! The caller supplies samples from one explicitly associated population.
//! Occupancy fill and legacy voxel centres are not surface measurements.
//! These types provide no classification, source tracking, scan admission,
//! deduplication, storage budget, revision ownership or persistence. Those belong
//! to the runtime capture owner and the versioned layer-record codec.

const PAIRS: [(usize, usize); 6] = [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatisticsError {
    NonFinite,
    CountOverflow,
    InvalidRotation,
}

/// Empirical scatter of returns, not calibrated sensor/pose covariance.
/// M2 order is xx, xy, xz, yy, yz, zz. Correlation is not assumed away.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CenteredMoments3 {
    count: u64,
    mean: [f64; 3],
    m2: [f64; 6],
}

impl CenteredMoments3 {
    pub fn count(&self) -> u64 {
        self.count
    }
    pub fn mean(&self) -> [f64; 3] {
        self.mean
    }
    pub fn m2(&self) -> [f64; 6] {
        self.m2
    }

    /// Sample scatter M2/(n-1). For one point it is unknown, not zero.
    /// This is NOT covariance of the mean; no M2/n uncertainty is provided.
    pub fn sample_scatter(&self) -> Option<[f64; 6]> {
        (self.count > 1).then(|| self.m2.map(|v| v / (self.count - 1) as f64))
    }

    fn singleton(point: [f64; 3]) -> Result<Self, StatisticsError> {
        if !point.iter().all(|v| v.is_finite()) {
            return Err(StatisticsError::NonFinite);
        }
        Ok(Self {
            count: 1,
            mean: point,
            m2: [0.0; 6],
        })
    }

    /// Parallel Welford merge. Both inputs must contain disjoint samples.
    /// The caller owns sample admission; this math type does not deduplicate.
    pub fn merged(&self, other: &Self) -> Result<Self, StatisticsError> {
        let count = self
            .count
            .checked_add(other.count)
            .filter(|n| *n <= 1_u64 << 53)
            .ok_or(StatisticsError::CountOverflow)?;
        let fraction = other.count as f64 / count as f64;
        let delta: [f64; 3] = std::array::from_fn(|i| other.mean[i] - self.mean[i]);
        let mean = std::array::from_fn(|i| self.mean[i] + delta[i] * fraction);
        let m2 = std::array::from_fn(|k| {
            let (i, j) = PAIRS[k];
            self.m2[k] + other.m2[k] + delta[i] * delta[j] * (self.count as f64 * fraction)
        });
        if !mean.iter().chain(m2.iter()).all(|v| v.is_finite()) {
            return Err(StatisticsError::NonFinite);
        }
        Ok(Self { count, mean, m2 })
    }

    /// Rigid-frame transformation of the sufficient moments: R M2 R^T.
    /// Reprojection of captured samples is owned by the runtime correction path.
    /// An exact observed AABB cannot be recovered after rotation from its old
    /// extrema alone, so this deliberately returns no transformed extent.
    pub fn rigid_transform(
        &self,
        rotation: [[f64; 3]; 3],
        translation: [f64; 3],
    ) -> Result<Self, StatisticsError> {
        if !rotation
            .iter()
            .flatten()
            .chain(translation.iter())
            .all(|v| v.is_finite())
        {
            return Err(StatisticsError::NonFinite);
        }
        for i in 0..3 {
            for j in 0..3 {
                let dot: f64 = (0..3).map(|k| rotation[k][i] * rotation[k][j]).sum();
                if (dot - if i == j { 1.0 } else { 0.0 }).abs() > 1e-10 {
                    return Err(StatisticsError::InvalidRotation);
                }
            }
        }
        let r = rotation;
        let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
        if (det - 1.0).abs() > 1e-10 {
            return Err(StatisticsError::InvalidRotation);
        }
        let mut matrix = [[0.0; 3]; 3];
        for (k, &(i, j)) in PAIRS.iter().enumerate() {
            matrix[i][j] = self.m2[k];
            matrix[j][i] = self.m2[k];
        }
        let mean = std::array::from_fn(|i| {
            translation[i] + (0..3).map(|j| r[i][j] * self.mean[j]).sum::<f64>()
        });
        let m2 = std::array::from_fn(|k| {
            let (i, j) = PAIRS[k];
            (0..3)
                .flat_map(|a| (0..3).map(move |b| r[i][a] * matrix[a][b] * r[j][b]))
                .sum()
        });
        if !mean.iter().chain(m2.iter()).all(|v| v.is_finite()) {
            return Err(StatisticsError::NonFinite);
        }
        Ok(Self {
            count: self.count,
            mean,
            m2,
        })
    }
}

/// Exact extrema of the actually supplied returns in their capture frame.
/// These are not occupancy bounds, a uniform distribution or a noise estimate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObservationStatistics {
    moments: CenteredMoments3,
    minimum: [f64; 3],
    maximum: [f64; 3],
}

impl ObservationStatistics {
    pub fn from_point(point: [f64; 3]) -> Result<Self, StatisticsError> {
        Ok(Self {
            moments: CenteredMoments3::singleton(point)?,
            minimum: point,
            maximum: point,
        })
    }
    pub fn moments(&self) -> CenteredMoments3 {
        self.moments
    }
    pub fn observed_extent(&self) -> ([f64; 3], [f64; 3]) {
        (self.minimum, self.maximum)
    }
    /// On a returned Err, self is unchanged.
    pub fn push(&mut self, point: [f64; 3]) -> Result<(), StatisticsError> {
        *self = self.merged(&Self::from_point(point)?)?;
        Ok(())
    }
    pub fn merged(&self, other: &Self) -> Result<Self, StatisticsError> {
        Ok(Self {
            moments: self.moments.merged(&other.moments)?,
            minimum: std::array::from_fn(|i| self.minimum[i].min(other.minimum[i])),
            maximum: std::array::from_fn(|i| self.maximum[i].max(other.maximum[i])),
        })
    }
}

#[cfg(test)]
#[path = "surface_statistics_tests.rs"]
mod tests;
