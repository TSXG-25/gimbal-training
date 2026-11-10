pub const STICK_MAX: i16 = 660;
pub const STICK_DEADZONE: i16 = 30;

#[derive(Clone, Copy, Debug)]
pub struct AxisMotionConfig {
    pub min_angle_rad: f32,
    pub max_angle_rad: f32,
    pub max_target_rate_rad_s: f32,
    pub max_motor_rpm: f32,
    pub stick_direction: f32,
}

impl AxisMotionConfig {
    pub fn is_valid(&self) -> bool {
        self.min_angle_rad.is_finite()
            && self.max_angle_rad.is_finite()
            && self.max_target_rate_rad_s.is_finite()
            && self.max_motor_rpm.is_finite()
            && self.stick_direction.is_finite()
            && self.min_angle_rad > -core::f32::consts::PI
            && self.max_angle_rad < core::f32::consts::PI
            && self.min_angle_rad <= 0.0
            && self.max_angle_rad >= 0.0
            && self.min_angle_rad < self.max_angle_rad
            && self.max_target_rate_rad_s > 0.0
            && self.max_motor_rpm > 0.0
            && (self.stick_direction == 1.0 || self.stick_direction == -1.0)
    }

    pub fn contains(&self, angle: f32) -> bool {
        self.is_valid()
            && angle.is_finite()
            && angle >= self.min_angle_rad
            && angle <= self.max_angle_rad
    }
}

// covert controller to 0-1
pub fn normalize_stick(raw: i16) -> Option<f32> {
    if !(-STICK_MAX..=STICK_MAX).contains(&raw) {
        return None;
    }

    let magnitude = raw.abs() as f32;
    let deadzone = STICK_DEADZONE as f32;
    let max = STICK_MAX as f32;

    if magnitude <= deadzone {
        return Some(0.0);
    }

    let value = (magnitude - deadzone) / (max - deadzone);

    Some(if raw < 0 { -value } else { value })
}

pub fn next_target_angle(
    current_target: f32,
    stick_raw: i16,
    dt_s: f32,
    config: AxisMotionConfig,
) -> Option<f32> {
    if !config.is_valid() || !config.contains(current_target) || !dt_s.is_finite() || dt_s <= 0.0 {
        return None;
    }

    let stick = normalize_stick(stick_raw)?;

    let target_rate = stick * config.stick_direction * config.max_target_rate_rad_s;

    let new_target = current_target + target_rate * dt_s;

    if !new_target.is_finite() {
        return None;
    }

    Some(new_target.clamp(config.min_angle_rad, config.max_angle_rad))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> AxisMotionConfig {
        AxisMotionConfig {
            // 以下仅为离线测试值，不是实机配置
            min_angle_rad: -0.8,
            max_angle_rad: 0.6,
            max_target_rate_rad_s: 1.0,
            max_motor_rpm: 50.0,
            stick_direction: 1.0,
        }
    }

    #[test]
    fn deadzone_works() {
        assert_eq!(normalize_stick(0), Some(0.0));
        assert_eq!(normalize_stick(20), Some(0.0));
        assert_eq!(normalize_stick(-30), Some(0.0));

        assert_eq!(normalize_stick(660), Some(1.0));
        assert_eq!(normalize_stick(-660), Some(-1.0));

        assert!((normalize_stick(345).unwrap() - 0.5).abs() < 0.00001);
    }

    #[test]
    fn invalid_stick_rejected() {
        assert!(normalize_stick(700).is_none());
        assert!(normalize_stick(-700).is_none());
    }

    #[test]
    fn target_moves_with_stick() {
        let next = next_target_angle(0.0, 660, 0.001, test_config()).unwrap();

        assert!((next - 0.001).abs() < 0.000001);
    }

    #[test]
    fn releasing_stick_holds_target() {
        let next = next_target_angle(0.2, 0, 0.001, test_config()).unwrap();

        assert_eq!(next, 0.2);
    }

    #[test]
    fn angle_limit_works() {
        let next = next_target_angle(0.5995, 660, 0.001, test_config()).unwrap();

        assert_eq!(next, 0.6);
    }

    #[test]
    fn direction_can_be_reversed() {
        let mut config = test_config();
        config.stick_direction = -1.0;

        let next = next_target_angle(0.0, 660, 0.001, config).unwrap();

        assert!((next + 0.001).abs() < 0.000001);
    }

    #[test]
    fn invalid_limits_rejected() {
        let mut config = test_config();

        config.min_angle_rad = 1.0;
        assert!(!config.is_valid());

        assert!(next_target_angle(0.0, 660, 0.001, config).is_none());
    }

    #[test]
    fn measured_angle_outside_limits_rejected() {
        let config = test_config();

        assert!(!config.contains(0.9));
        assert!(config.contains(0.2));

        assert!(next_target_angle(0.9, 0, 0.001, config).is_none());
    }
}
