// Stall: >=5 rpm demand, <=2 rpm feedback, >=200 voltage counts for 500 ms.
pub const STALL_DEMAND_RPM: f32 = 5.0;
pub const STALL_SPEED_RPM: f32 = 2.0;
pub const STALL_OUTPUT_RAW: f32 = 200.0;
pub const STALL_TICKS: u16 = 500;

// Oscillation: errors reach both +/-0.03 rad, >10 Hz for 4 complete cycles.
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
    /// Total milliseconds in completed consecutive high-frequency cycles.
    pub fast_ticks: u16,
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
        self.fast_ticks = 0;
        self.last_period_ticks = 0;
        self.anchor_side = 0;
        self.saw_opposite = false;
        self.period_ticks = 0;
    }

    pub fn frequency_hz(&self) -> f32 {
        if self.last_period_ticks == 0 {
            0.0
        } else {
            1000.0 / self.last_period_ticks as f32
        }
    }

    pub fn step(
        &mut self,
        target_rpm: f32,
        actual_rpm: f32,
        output_abs: f32,
        angle_error: f32,
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

        // A complete cycle must be shorter than 100 ms (>10 Hz).
        if self.period_ticks >= OSC_MAX_PERIOD_TICKS {
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
            self.fast_ticks = self.fast_ticks.saturating_add(self.period_ticks);

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
            assert_eq!(m.step(10.0, 0.0, 500.0, 0.0), None);
        }

        assert_eq!(m.step(10.0, 0.0, 500.0, 0.0), Some(FaultKind::Stall));
    }

    #[test]
    fn stationary_hold_is_not_stall() {
        let mut m = FaultMonitor::default();

        for _ in 0..600 {
            assert_eq!(m.step(0.0, 0.0, 500.0, 0.0), None);
        }
    }

    #[test]
    fn repeated_fast_oscillation_triggers() {
        let mut m = FaultMonitor::default();
        let mut result = None;

        for tick in 0..200 {
            // 每 20 ms 换方向，完整周期 40 ms
            let error = if (tick / 20) % 2 == 0 { 0.08 } else { -0.08 };

            result = m.step(0.0, 0.0, 0.0, error);

            if result.is_some() {
                break;
            }
        }

        assert_eq!(result, Some(FaultKind::Oscillation));
        assert_eq!(m.last_period_ticks, 40);
        assert_eq!(m.fast_ticks, 160);
    }

    #[test]
    fn slow_oscillation_is_not_fault() {
        let mut m = FaultMonitor::default();

        for tick in 0..900 {
            let error = if (tick / 80) % 2 == 0 { 0.08 } else { -0.08 };

            assert_eq!(m.step(0.0, 0.0, 0.0, error), None);
        }
    }

    #[test]
    fn motion_demand_does_not_disable_oscillation_detection() {
        let mut m = FaultMonitor::default();
        let mut result = None;
        for tick in 0..200 {
            let error = if (tick / 20) % 2 == 0 { 0.08 } else { -0.08 };
            result = m.step(3.0, 3.0, 500.0, error);
            if result.is_some() {
                break;
            }
        }
        assert_eq!(result, Some(FaultKind::Oscillation));
        assert!((m.frequency_hz() - 25.0).abs() < 0.01);
    }

    #[test]
    fn exact_10_hz_does_not_exceed_threshold() {
        let mut m = FaultMonitor::default();
        for tick in 0..1000 {
            let error = if (tick / 50) % 2 == 0 { 0.08 } else { -0.08 };
            assert_eq!(m.step(0.0, 0.0, 0.0, error), None);
        }
    }

    #[test]
    fn noise_and_single_overshoot_are_not_sustained_oscillation() {
        let mut m = FaultMonitor::default();
        for tick in 0..500 {
            let error = if tick == 0 {
                0.08
            } else if tick == 20 {
                -0.08
            } else {
                0.01
            };
            assert_eq!(m.step(0.0, 0.0, 0.0, error), None);
        }
        assert_eq!(m.fast_cycles, 0);
    }
}
