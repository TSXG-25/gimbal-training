#[path = "src/motor_protocol.rs"]
mod motor_protocol;

#[derive(Default)]
pub struct Pid {
    //p
    pub kp: f32,
    pub maxout: f32,
    pub kd: f32,
    //d
    pub previous_error: f32,
    pub has_previous: bool,
    //i
    pub ki: f32,
    pub integral: f32,
    pub integral_limit: f32,
}

impl Pid {
    // TODO: clear the integral and history.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.has_previous = false;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        // TODO: compute the PID with dt_s, and limit the integral and output.
        //p
        let error = target - measured;
        let p_output = self.kp * error;
        //i
        if dt_s > 0.0{
            self.integral = self.integral + error * dt_s;
            let integral_limit = self.integral_limit.abs();
            self.integral = self.integral.clamp(-integral_limit, integral_limit);
        }
        let i_output = self.ki * self.integral;

        //d
        let d_output = if self.has_previous && dt_s > 0.0 {
            self.kd * (error - self.previous_error) / dt_s
        } else{
            0.0
        };
        self.previous_error = error;
        self.has_previous = true;

        let output = p_output + i_output + d_output;
        
        let limit = self.maxout.abs();
        output.clamp(-limit, limit)
    }
}

fn main() {
    // Test 2: Integral
    let mut pid_i = Pid {
        kp: 0.0,
        ki: 4.0,
        kd: 0.0,
        maxout: 100.0,
        integral_limit: 100.0,
        ..Default::default()
    };

    let i1 = pid_i.calculate(2.0, 0.0, 0.5);
    let i2 = pid_i.calculate(2.0, 0.0, 0.5);

    println!("I test: {}, {}", i1, i2);

    assert!((i1 - 4.0).abs() < 0.0001);
    assert!((i2 - 8.0).abs() < 0.0001);

    // Test 3: Derivative
    let mut pid_d = Pid {
        kp: 0.0,
        ki: 0.0,
        kd: 1.0,
        maxout: 100.0,
        integral_limit: 100.0,
        ..Default::default()
    };

    let d1 = pid_d.calculate(10.0, 0.0, 0.1);
    let d2 = pid_d.calculate(10.0, 2.0, 0.1);

    println!("D test: {}, {}", d1, d2);

    assert!((d1 - 0.0).abs() < 0.0001);
    assert!((d2 - (-20.0)).abs() < 0.0001);

    // Test 4: Reset
    pid_i.reset();

    let i_after_reset = pid_i.calculate(2.0, 0.0, 0.5);

    println!("I after reset: {}", i_after_reset);

    assert!((i_after_reset - 4.0).abs() < 0.0001);

    // Test 5: Cascade PID

    // 位置环
    let mut position_pid = Pid {
        kp: 100.0,
        ki: 0.0,
        kd: 0.0,
        maxout: 60.0,
        integral_limit: 100.0,
        ..Default::default()
    };

    // 速度环
    let mut speed_pid = Pid {
        kp: 100.0,
        ki: 0.0,
        kd: 0.0,
        maxout: 2000.0,
        integral_limit: 100.0,
        ..Default::default()
    };

    let target_angle = 1.0;
    let current_angle = 0.8;
    let current_rpm = 5.0;
    let dt_s = 0.001;

    // TODO 1: 使用位置 PID 计算目标转速
    let target_rpm = position_pid.calculate(target_angle, current_angle, dt_s);  

    // TODO 2: 使用速度 PID 计算最终输出
    let motor_output = speed_pid.calculate(target_rpm, current_rpm, dt_s);

    println!("Target RPM: {}", target_rpm);
    println!("Motor output: {}", motor_output);

    // Test 6: Encoder angle conversion

    let encoder_raw: u16 = 2048;

    // TODO：将编码器数值换算为弧度
    let angle_rad = encoder_raw as f32 / 8192.0  * 2.0 * std::f32::consts::PI;

    println!("Encoder angle: {} rad", angle_rad);

    fn shortest_delta_counts(current: u16, zero: u16) -> i32 {
        let mut diff = current as i32 - zero as i32;

        if diff > 4096 {
            diff = diff - 8192
        } else if diff < -4096 {
            diff = diff + 8192
        }

        diff
    }

    // Test 7: Angle wrapping

    let diff1 = shortest_delta_counts(5120, 4096);
    let diff2 = shortest_delta_counts(2, 8190);
    let diff3 = shortest_delta_counts(8190, 2);

    println!("Angle differences: {}, {}, {}", diff1, diff2, diff3);

    assert_eq!(diff1, 1024);
    assert_eq!(diff2, 4);
    assert_eq!(diff3, -4);

    let angle = joint_angle_rad(5120, 4096);
    fn joint_angle_rad(current: u16, zero: u16) -> f32 {
        let delta = shortest_delta_counts(current, zero);

        delta as f32 / 8192.0 * 2.0 * core::f32::consts::PI
    }

    println!("Joint angle: {}", angle);

    assert!(
        (angle - std::f32::consts::FRAC_PI_4).abs() < 0.00001
    );

    // Test 8: CAN voltage encoding
    let mut data = [0u8; 8];

    motor_protocol::write_voltage_slot(
        &mut data,
        1,
        1000.0,
        2000.0,
    );

    motor_protocol::write_voltage_slot(
        &mut data,
        2,
        -500.0,
        2000.0,
    );

    println!("CAN data: {:02X?}", data);

    assert_eq!(
        data,
        [0x03, 0xE8, 0xFE, 0x0C, 0, 0, 0, 0]
    );

    // 测试限幅
    let mut limited = [0u8; 8];
    motor_protocol::write_voltage_slot(&mut limited, 1, 5000.0, 1000.0);
    assert_eq!(&limited[0..2], &[0x03, 0xE8]);

    // 测试零输出锁定
    let mut disabled = [0u8; 8];
    motor_protocol::write_voltage_slot(&mut disabled, 1, 1000.0, 0.0);
    assert_eq!(disabled, [0u8; 8]);

    // 测试无效浮点数
    let mut invalid = [0u8; 8];
    motor_protocol::write_voltage_slot(&mut invalid, 1, f32::NAN, 2000.0);
    assert_eq!(invalid, [0u8; 8]);

    println!("CAN encoding tests passed!");
}