pub const STALL_DEMAND_RPM: f32 = 5.0;
pub const STALL_SPEED_RPM: f32 = 2.0;
pub const STALL_OUTPUT_RAW: f32 = 200.0;
pub const STALL_TICKS: u16 = 500;

pub const OSC_AMPLITUDE_RAD: f32 = 0.03;
pub const OSC_MAX_PERIOD_TICKS: u16 = 100;
pub const OSC_REQUIRED_CYCLES: u8 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultKind {
    Stall,
    Oscillation,
}

#[derive(Default)]
pub struct FaultMonitor {
    pub stall_ticks: u16,
    pub fast_cycles: u8,
    pub last_period_ticks: u16,

    anchor_side: i8,
    saw_opposite: bool,
    period_ticks: u16,
}

impl FaultMonitor {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    fn reset_oscillation(&mut self) {
        self.fast_cycles = 0;
        self.last_period_ticks = 0;
        self.anchor_side = 0;
        self.saw_opposite = false;
        self.period_ticks = 0;
    }

    pub fn step(
        &mut self,
        target_rpm: f32,
        actual_rpm: f32,
        output_abs: f32,
        angle_error: f32,
        check_oscillation: bool,
    ) -> Option<FaultKind> {
        if !target_rpm.is_finite()
            || !actual_rpm.is_finite()
            || !output_abs.is_finite()
            || !angle_error.is_finite()
        {
            self.reset();
            return None;
        }

        // 堵转
        let stalled = target_rpm.abs() >= STALL_DEMAND_RPM
            && actual_rpm.abs() <= STALL_SPEED_RPM
            && output_abs >= STALL_OUTPUT_RAW;

        if stalled {
            self.stall_ticks = self.stall_ticks.saturating_add(1);
        } else {
            self.stall_ticks = 0;
        }

        if self.stall_ticks >= STALL_TICKS {
            return Some(FaultKind::Stall);
        }

        // 震荡
        if !check_oscillation {
            self.reset_oscillation();
            return None;
        }

        // 误差跨越正负两个幅度阈值
        let side = if angle_error >= OSC_AMPLITUDE_RAD {
            1
        } else if angle_error <= -OSC_AMPLITUDE_RAD {
            -1
        } else {
            0
        };

        if self.anchor_side == 0 {
            if side != 0 {
                self.anchor_side = side;
            }
            return None;
        }

        self.period_ticks = self.period_ticks.saturating_add(1);

        // 一个完整周期必须在 100 ticks 内结束
        if self.period_ticks > OSC_MAX_PERIOD_TICKS {
            self.reset_oscillation();
            if side != 0 {
                self.anchor_side = side;
            }
            return None;
        }

        if !self.saw_opposite {
            if side == -self.anchor_side {
                self.saw_opposite = true;
            }
        } else if side == self.anchor_side {
            self.last_period_ticks = self.period_ticks;
            self.fast_cycles = self.fast_cycles.saturating_add(1);

            self.period_ticks = 0;
            self.saw_opposite = false;

            if self.fast_cycles >= OSC_REQUIRED_CYCLES {
                return Some(FaultKind::Oscillation);
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stall_requires_sustained_demand() {
        let mut m = FaultMonitor::default();

        for _ in 0..499 {
            assert_eq!(m.step(10.0, 0.0, 500.0, 0.0, false), None);
        }

        assert_eq!(m.step(10.0, 0.0, 500.0, 0.0, false), Some(FaultKind::Stall));
    }

    #[test]
    fn stationary_hold_is_not_stall() {
        let mut m = FaultMonitor::default();

        for _ in 0..600 {
            assert_eq!(m.step(0.0, 0.0, 500.0, 0.0, false), None);
        }
    }

    #[test]
    fn repeated_fast_oscillation_triggers() {
        let mut m = FaultMonitor::default();
        let mut result = None;

        for tick in 0..200 {
            // 每 20 ms 换方向，完整周期 40 ms
            let error = if (tick / 20) % 2 == 0 { 0.08 } else { -0.08 };

            result = m.step(0.0, 0.0, 0.0, error, true);

            if result.is_some() {
                break;
            }
        }

        assert_eq!(result, Some(FaultKind::Oscillation));
        assert_eq!(m.last_period_ticks, 40);
    }

    #[test]
    fn slow_oscillation_is_not_fault() {
        let mut m = FaultMonitor::default();

        for tick in 0..900 {
            let error = if (tick / 80) % 2 == 0 { 0.08 } else { -0.08 };

            assert_eq!(m.step(0.0, 0.0, 0.0, error, true), None);
        }
    }

    #[test]
    fn moving_target_disables_oscillation_check() {
        let mut m = FaultMonitor::default();

        for tick in 0..200 {
            let error = if (tick / 20) % 2 == 0 { 0.08 } else { -0.08 };

            assert_eq!(m.step(0.0, 0.0, 0.0, error, false), None);
        }
    }
}
