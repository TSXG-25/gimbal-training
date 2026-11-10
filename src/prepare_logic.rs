pub const PREPARE_MAX_RPM: f32 = 15.0;
pub const ANGLE_TOL_RAD: f32 = 0.03;
pub const SPEED_TOL_RPM: f32 = 5.0;

pub const SETTLE_TICKS: u16 = 200;
pub const TIMEOUT_TICKS: u16 = 5000;

#[derive(Debug, PartialEq, Eq)]
pub enum PrepareOutcome {
    Active,
    Done,
    Timeout,
}

#[derive(Default)]
pub struct PrepareProgress {
    pub elapsed_ticks: u16,
    pub stable_ticks: u16,
}

impl PrepareProgress {
    pub fn reset(&mut self) {
        self.elapsed_ticks = 0;
        self.stable_ticks = 0;
    }

    pub fn advance(&mut self, settled: bool) -> PrepareOutcome {
        self.elapsed_ticks = self.elapsed_ticks.saturating_add(1);

        if settled {
            self.stable_ticks = self.stable_ticks.saturating_add(1);
        } else {
            self.stable_ticks = 0;
        }

        if self.stable_ticks >= SETTLE_TICKS {
            PrepareOutcome::Done
        } else if self.elapsed_ticks >= TIMEOUT_TICKS {
            PrepareOutcome::Timeout
        } else {
            PrepareOutcome::Active
        }
    }
}

pub fn axes_settled(yaw_angle: f32, yaw_rpm: f32, pitch_angle: f32, pitch_rpm: f32) -> bool {
    [yaw_angle, yaw_rpm, pitch_angle, pitch_rpm]
        .iter()
        .all(|x| x.is_finite())
        && yaw_angle.abs() <= ANGLE_TOL_RAD
        && pitch_angle.abs() <= ANGLE_TOL_RAD
        && yaw_rpm.abs() <= SPEED_TOL_RPM
        && pitch_rpm.abs() <= SPEED_TOL_RPM
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_for_200_ticks() {
        let mut progress = PrepareProgress::default();

        for _ in 0..199 {
            assert_eq!(progress.advance(true), PrepareOutcome::Active);
        }

        assert_eq!(progress.advance(true), PrepareOutcome::Done);
    }

    #[test]
    fn interrupted_stability_resets_counter() {
        let mut progress = PrepareProgress::default();

        for _ in 0..100 {
            progress.advance(true);
        }

        progress.advance(false);
        assert_eq!(progress.stable_ticks, 0);
    }

    #[test]
    fn timeout_after_5000_ticks() {
        let mut progress = PrepareProgress::default();

        for _ in 0..4999 {
            assert_eq!(progress.advance(false), PrepareOutcome::Active);
        }

        assert_eq!(progress.advance(false), PrepareOutcome::Timeout);
    }

    #[test]
    fn both_axes_must_be_settled() {
        assert!(axes_settled(0.0, 0.0, 0.0, 0.0));

        // Pitch 角度超出容差
        assert!(!axes_settled(0.0, 0.0, 0.2, 0.0));

        // Yaw 转速过高
        assert!(!axes_settled(0.0, 10.0, 0.0, 0.0));

        // 非法反馈
        assert!(!axes_settled(f32::NAN, 0.0, 0.0, 0.0));
    }
}
