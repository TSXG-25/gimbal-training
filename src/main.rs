#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![forbid(unsafe_code)]
#[derive(Clone, Copy)]
enum RecoveryPhase {
    NeedRelease,
    NeedDown,
    NeedUp,
}

#[cfg(not(test))]
mod dm_mc02;
#[cfg(test)]
#[path = "../tests/support.rs"]
mod dm_mc02;
#[cfg(test)]
#[path = "../tests/control.rs"]
mod tests;
#[cfg(test)]
extern crate self as rtt_target;
#[cfg(test)]
#[macro_export]
macro_rules! rprintln {
    ($($arg:tt)*) => { println!($($arg)*); };
}
mod fault_logic;
mod prepare_logic;
mod running_logic;

use fault_logic::FaultMonitor;
use running_logic::STICK_DEADZONE;

use running_logic::{AxisMotionConfig, next_target_angle};

use prepare_logic::{PREPARE_MAX_RPM, PrepareOutcome, PrepareProgress, axes_settled};

const DT_S: f32 = 0.001;

// Adjust: whether can raise verified flag
const HOMING_CONFIG_VERIFIED: bool = true;

mod motor_protocol;
use motor_protocol::write_voltage_slot;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
#[cfg(test)]
use dm_mc02::{Frame, Id};
#[cfg(not(test))]
use embassy_executor::Spawner;
#[cfg(not(test))]
use embassy_stm32::can::Frame;
#[cfg(not(test))]
use embassy_time::{Duration, Instant, Ticker};
#[cfg(not(test))]
use embedded_can::{Frame as _, Id};

// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
#[cfg(not(test))]
const TICK: Duration = Duration::from_millis(1);

const FEEDBACK_TIMEOUT_TICKS: u16 = 100;

// Assumption：motor 1 = Yaw，motor 2 = Pitch
const YAW_MOTOR_ID: u8 = 1;
const PITCH_MOTOR_ID: u8 = 2;

const YAW_FEEDBACK_ID: u16 = 0x204 + YAW_MOTOR_ID as u16;
const PITCH_FEEDBACK_ID: u16 = 0x204 + PITCH_MOTOR_ID as u16;

// Adjust: locked inut
const BENCH_MAX_ABS_RAW: f32 = 16000.0;
// Measured horizontal holding voltage (+4773); zero_encoder_count must be horizontal.
const PITCH_GRAVITY_GAIN_RAW: f32 = 4773.0;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);
pub static DEBUG_TICKS: AtomicU32 = AtomicU32::new(0);
pub static DEBUG_STATE: AtomicU32 = AtomicU32::new(0);

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
    pub encoder_raw: u16,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    pub fn read(&mut self, data: &[u8; 8]) {
        let angle_raw = u16::from_be_bytes([data[0], data[1]]);

        // Covert encoder count to angle
        self.angle_rad = angle_raw as f32 / 8192.0 * 2.0 * core::f32::consts::PI;

        // get rpm
        let speed_raw = i16::from_be_bytes([data[2], data[3]]);
        self.speed_rpm = speed_raw as f32;

        // get current raw value
        self.current_raw = i16::from_be_bytes([data[4], data[5]]);

        self.encoder_raw = angle_raw;
    }
}

fn shortest_delta_counts(current: u16, zero: u16) -> i32 {
    let mut diff = current as i32 - zero as i32;

    if diff > 4096 {
        diff = diff - 8192
    } else if diff < -4096 {
        diff = diff + 8192
    }

    diff
}

fn joint_angle_rad(current: u16, zero: u16) -> f32 {
    let delta = shortest_delta_counts(current, zero);

    delta as f32 / 8192.0 * 2.0 * core::f32::consts::PI
}

fn pid_params_valid(pid: &Pid) -> bool {
    [pid.kp, pid.ki, pid.kd, pid.maxout, pid.integral_limit]
        .iter()
        .all(|value| value.is_finite())
}

/// PID gains, plus the integral and history kept between ticks.
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
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.has_previous = false;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        //p
        let error = target - measured;
        let p_output = self.kp * error;
        //i
        if dt_s > 0.0 {
            self.integral = self.integral + error * dt_s;
            let integral_limit = self.integral_limit.abs();
            self.integral = self.integral.clamp(-integral_limit, integral_limit);
        }
        let i_output = self.ki * self.integral;

        //d
        let d_output = if self.has_previous && dt_s > 0.0 {
            self.kd * (error - self.previous_error) / dt_s
        } else {
            0.0
        };
        self.previous_error = error;
        self.has_previous = true;

        let output = p_output + i_output + d_output;

        let limit = self.maxout.abs();
        output.clamp(-limit, limit)
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,
    /// Signed horizontal gravity compensation in voltage counts; zero for yaw.
    pub gravity_gain_raw: f32,
    pub zero_encoder_count: Option<u16>, //Adjustment: 0 position adjust
    pub motion_config: Option<AxisMotionConfig>,
}
impl Axis {
    pub fn measured_joint_angle_rad(&self) -> Option<f32> {
        self.zero_encoder_count
            .map(|zero| joint_angle_rad(self.motor.encoder_raw, zero))
    }
    pub fn begin_prepare(&mut self) {
        // return to zero for machine
        self.target_angle_rad = 0.0;
        self.target_rpm = 0.0;

        self.position_pid.reset();
        self.speed_pid.reset();

        self.motor.disable();
    }

    pub fn prepare_step(&mut self) -> Option<f32> {
        let measured_angle = self.measured_joint_angle_rad()?;

        let config = self.motion_config?;
        let max_rpm = PREPARE_MAX_RPM.min(config.max_motor_rpm);

        if !config.contains(measured_angle) {
            self.motor.disable();
            return None;
        }

        if !measured_angle.is_finite() || !self.motor.speed_rpm.is_finite() {
            return None;
        }

        //position pid
        self.target_rpm = self
            .position_pid
            .calculate(self.target_angle_rad, measured_angle, DT_S)
            .clamp(-max_rpm, max_rpm);

        //speed pid
        // Add gravity compensation after the speed loop, using the measured angle.
        let feedback = self
            .speed_pid
            .calculate(self.target_rpm, self.motor.speed_rpm, DT_S);
        let gravity_feedforward = self.gravity_gain_raw * libm::cosf(measured_angle);
        let output_limit = self.speed_pid.maxout.abs();
        self.motor.command_voltage_raw =
            (feedback + gravity_feedforward).clamp(-output_limit, output_limit);

        if !self.target_rpm.is_finite() || !self.motor.command_voltage_raw.is_finite() {
            self.motor.disable();
            return None;
        }

        Some(measured_angle)
    }

    pub fn running_step(&mut self, stick_raw: i16) -> Option<()> {
        let config = self.motion_config?;
        let measured_angle = self.measured_joint_angle_rad()?;

        // avoid out move
        if !config.contains(measured_angle) || !self.motor.speed_rpm.is_finite() {
            return None;
        }

        if !pid_params_valid(&self.position_pid) || !pid_params_valid(&self.speed_pid) {
            return None;
        }

        self.target_angle_rad = next_target_angle(self.target_angle_rad, stick_raw, DT_S, config)?;

        self.target_rpm = self
            .position_pid
            .calculate(self.target_angle_rad, measured_angle, DT_S);

        if !self.target_rpm.is_finite() {
            self.motor.disable();
            return None;
        }

        self.target_rpm = self
            .target_rpm
            .clamp(-config.max_motor_rpm, config.max_motor_rpm);

        // Add gravity compensation after the speed loop, using the measured angle.
        let feedback = self
            .speed_pid
            .calculate(self.target_rpm, self.motor.speed_rpm, DT_S);
        let gravity_feedforward = self.gravity_gain_raw * libm::cosf(measured_angle);
        let output_limit = self.speed_pid.maxout.abs();
        self.motor.command_voltage_raw =
            (feedback + gravity_feedforward).clamp(-output_limit, output_limit);

        if !self.motor.command_voltage_raw.is_finite() {
            self.motor.disable();
            return None;
        }

        Some(())
    }
}

/// State shared by every control tick.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    pub yaw_feedback_age_ticks: u16,
    pub pitch_feedback_age_ticks: u16,
    pub enable_armed: bool,
    pub prepare_progress: PrepareProgress,
    pub yaw_fault: FaultMonitor,
    pub pitch_fault: FaultMonitor,
    /// Most recent group command prepared for CAN transmission (including failed attempts).
    pub last_command_data: [u8; 8],

    recovery_phase: RecoveryPhase,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            // ========== YAW PID ==========
            yaw: Axis {
                zero_encoder_count: Some(3564),
                position_pid: Pid {
                    kp: 45.0, // Yaw 位置环 Kp
                    ki: 0.0,
                    kd: 0.01,
                    maxout: 12.0, // 输出最大目标 rpm
                    integral_limit: 0.0,
                    ..Pid::default()
                },

                speed_pid: Pid {
                    kp: 400.0, // Yaw 速度环 Kp
                    ki: 100.0,
                    kd: 0.0,
                    maxout: 2500.0, // 输出最大电压原始值
                    integral_limit: 4.0,
                    ..Pid::default()
                },
                motion_config: Some(AxisMotionConfig {
                    min_angle_rad: -2.618,
                    max_angle_rad: 2.618,
                    max_target_rate_rad_s: 0.8,
                    max_motor_rpm: 15.0,
                    stick_direction: -1.0,
                }),

                ..Axis::default()
            },

            // ========== PITCH PID ==========
            pitch: Axis {
                zero_encoder_count: Some(4110),
                gravity_gain_raw: PITCH_GRAVITY_GAIN_RAW,
                position_pid: Pid {
                    kp: 30.0, // Pitch 位置环 Kp
                    ki: 0.0,
                    kd: 0.0,
                    maxout: 6.0,
                    integral_limit: 0.0,
                    ..Pid::default()
                },

                speed_pid: Pid {
                    kp: 150.0, // Pitch 速度环 Kp
                    ki: 1000.0,
                    kd: 0.0,
                    maxout: 10000.0,
                    integral_limit: 20.0,
                    ..Pid::default()
                },

                motion_config: Some(AxisMotionConfig {
                    min_angle_rad: -0.50,
                    max_angle_rad: 0.50,
                    max_target_rate_rad_s: 0.3,
                    max_motor_rpm: 6.0,
                    stick_direction: 1.0,
                }),

                ..Axis::default()
            },
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            yaw_feedback_age_ticks: FEEDBACK_TIMEOUT_TICKS,
            pitch_feedback_age_ticks: FEEDBACK_TIMEOUT_TICKS,
            enable_armed: false,
            prepare_progress: PrepareProgress::default(),
            yaw_fault: FaultMonitor::default(),
            pitch_fault: FaultMonitor::default(),
            last_command_data: [0; 8],

            recovery_phase: RecoveryPhase::NeedRelease,
        }
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.check_faults();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                self.state = State::Waiting;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        let receiver = can.reader();
        self.yaw_feedback_age_ticks = self.yaw_feedback_age_ticks.saturating_add(1);
        self.pitch_feedback_age_ticks = self.pitch_feedback_age_ticks.saturating_add(1);
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;
                    // Only accept CAN frame
                    if frame.is_remote_frame() {
                        continue;
                    }

                    let can_id = match frame.id() {
                        Id::Standard(id) => id.as_raw(),
                        Id::Extended(_) => continue,
                    };

                    // check 8 bytes or not
                    let data: &[u8; 8] = match frame.data().try_into() {
                        Ok(bytes) => bytes,
                        Err(_) => continue,
                    };
                    let encoder = u16::from_be_bytes([data[0], data[1]]);

                    if encoder >= 8192 {
                        continue;
                    }

                    // Assign to motors based on CAN id
                    match can_id {
                        YAW_FEEDBACK_ID => {
                            self.yaw.motor.read(data);
                            self.yaw_feedback_age_ticks = 0;
                        }
                        PITCH_FEEDBACK_ID => {
                            self.pitch.motor.read(data);
                            self.pitch_feedback_age_ticks = 0;
                        }
                        _ => {}
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    pub fn update_mode(&mut self) {
        let yaw_fresh = self.yaw_feedback_age_ticks < FEEDBACK_TIMEOUT_TICKS;
        let pitch_fresh = self.pitch_feedback_age_ticks < FEEDBACK_TIMEOUT_TICKS;
        let motion_ready = self
            .yaw
            .motion_config
            .map(|c| c.is_valid())
            .unwrap_or(false)
            && self
                .pitch
                .motion_config
                .map(|c| c.is_valid())
                .unwrap_or(false);
        let zero_ready = HOMING_CONFIG_VERIFIED
            && motion_ready
            && self.yaw.zero_encoder_count.is_some()
            && self.pitch.zero_encoder_count.is_some();
        if !zero_ready {
            self.enable_armed = false;
            if self.state != State::Error {
                self.state = State::Waiting;
            }
            self.recovery_phase = RecoveryPhase::NeedRelease;
            return;
        }
        if self.remote.is_none() || !yaw_fresh || !pitch_fresh {
            self.enable_armed = false;
            if self.state != State::Error {
                self.state = State::Waiting;
            }
            self.recovery_phase = RecoveryPhase::NeedRelease;
            return;
        }
        let remote = self.remote.unwrap();
        match self.state {
            State::Waiting => {
                if remote.right_switch == 2 {
                    self.enable_armed = true;
                } else if remote.right_switch == 3 && self.enable_armed {
                    self.enter_prepare();
                } else if remote.right_switch == 1 {
                    self.enable_armed = false;
                }
            }
            State::Prepare => {
                if remote.right_switch != 3 {
                    self.state = State::Waiting;
                    self.enable_armed = false;
                }
            }
            State::Running => {
                if remote.right_switch != 3 {
                    self.state = State::Waiting;
                    self.enable_armed = false;
                }
            }
            State::Error => {
                self.enable_armed = false;

                if remote.right_switch != 3 {
                    self.recovery_phase = RecoveryPhase::NeedRelease;
                } else {
                    match self.recovery_phase {
                        RecoveryPhase::NeedRelease => {
                            if remote.left_switch != 2 {
                                self.recovery_phase = RecoveryPhase::NeedDown;
                            }
                        }

                        RecoveryPhase::NeedDown => {
                            if remote.left_switch == 2 {
                                self.recovery_phase = RecoveryPhase::NeedUp;
                            }
                        }

                        RecoveryPhase::NeedUp => {
                            let sticks_centered = remote.channels[2].abs() <= STICK_DEADZONE
                                && remote.channels[3].abs() <= STICK_DEADZONE;

                            if remote.left_switch == 1 && sticks_centered {
                                self.recovery_phase = RecoveryPhase::NeedRelease;
                                self.recover_from_error();
                            }
                        }
                    }
                }
            }
        }
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    pub fn update(&mut self) {
        match self.state {
            State::Waiting | State::Error => {
                self.disable_all();
            }

            State::Prepare => {
                let yaw_angle = self.yaw.prepare_step();
                let pitch_angle = self.pitch.prepare_step();

                let (Some(yaw_angle), Some(pitch_angle)) = (yaw_angle, pitch_angle) else {
                    self.state = State::Error;
                    self.disable_all();
                    rtt_target::rprintln!("Prepare: invalid angle or PID output");
                    return;
                };

                let settled = axes_settled(
                    yaw_angle,
                    self.yaw.motor.speed_rpm,
                    pitch_angle,
                    self.pitch.motor.speed_rpm,
                );

                match self.prepare_progress.advance(settled) {
                    PrepareOutcome::Active => {}

                    PrepareOutcome::Done => {
                        self.yaw.target_angle_rad = yaw_angle;
                        self.pitch.target_angle_rad = pitch_angle;

                        self.yaw.target_rpm = 0.0;
                        self.pitch.target_rpm = 0.0;

                        self.yaw.position_pid.reset();
                        self.yaw.speed_pid.reset();
                        self.pitch.position_pid.reset();
                        self.pitch.speed_pid.reset();

                        self.disable_all();
                        self.state = State::Running;

                        rtt_target::rprintln!("Prepare completed");
                    }

                    PrepareOutcome::Timeout => {
                        self.state = State::Error;
                        self.disable_all();

                        rtt_target::rprintln!("Prepare timeout");
                    }
                }
            }

            State::Running => {
                let Some(remote) = self.remote else {
                    self.state = State::Waiting;
                    self.enable_armed = false;
                    self.disable_all();
                    return;
                };

                // left channels[2] for Yaw
                let yaw_result = self.yaw.running_step(remote.channels[2]);

                // left channels[3] for Pitch
                let pitch_result = self.pitch.running_step(remote.channels[3]);

                if yaw_result.is_none() || pitch_result.is_none() {
                    self.state = State::Error;
                    self.enable_armed = false;
                    self.disable_all();

                    rtt_target::rprintln!("Running: invalid feedback, limits or controller");
                }
            }
        }
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        if YAW_MOTOR_ID != PITCH_MOTOR_ID && matches!(self.state, State::Prepare | State::Running) {
            write_voltage_slot(
                &mut data,
                YAW_MOTOR_ID,
                self.yaw.motor.command_voltage_raw,
                BENCH_MAX_ABS_RAW,
            );
            write_voltage_slot(
                &mut data,
                PITCH_MOTOR_ID,
                self.pitch.motor.command_voltage_raw,
                BENCH_MAX_ABS_RAW,
            );
        }
        self.last_command_data = data;
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn enter_prepare(&mut self) {
        self.yaw.begin_prepare();
        self.pitch.begin_prepare();

        self.prepare_progress.reset();
        self.enable_armed = false;
        self.state = State::Prepare;
        rtt_target::rprintln!("Prepare started: targets=0 rad, PID reset");
    }

    fn check_faults(&mut self) {
        if self.state == State::Waiting {
            self.yaw_fault.reset();
            self.pitch_fault.reset();
        }
        if !matches!(self.state, State::Prepare | State::Running) {
            return;
        }

        let Some(yaw_angle) = self.yaw.measured_joint_angle_rad() else {
            return;
        };
        let Some(pitch_angle) = self.pitch.measured_joint_angle_rad() else {
            return;
        };

        let yaw_fault = self.yaw_fault.step(
            self.yaw.target_rpm,
            self.yaw.motor.speed_rpm,
            self.yaw
                .motor
                .command_voltage_raw
                .abs()
                .min(BENCH_MAX_ABS_RAW),
            self.yaw.target_angle_rad - yaw_angle,
        );

        let pitch_fault = self.pitch_fault.step(
            self.pitch.target_rpm,
            self.pitch.motor.speed_rpm,
            self.pitch
                .motor
                .command_voltage_raw
                .abs()
                .min(BENCH_MAX_ABS_RAW),
            self.pitch.target_angle_rad - pitch_angle,
        );

        let fault = if let Some(kind) = yaw_fault {
            Some(("Yaw", kind, self.yaw_fault.last_period_ticks))
        } else if let Some(kind) = pitch_fault {
            Some(("Pitch", kind, self.pitch_fault.last_period_ticks))
        } else {
            None
        };

        if let Some((axis, kind, period)) = fault {
            self.state = State::Error;
            self.enable_armed = false;
            self.recovery_phase = RecoveryPhase::NeedRelease;
            self.disable_all();

            let frequency_hz = if period > 0 {
                1000.0 / period as f32
            } else {
                0.0
            };

            rtt_target::rprintln!(
                "FAULT: {} {:?}, period={} ms, estimated={:.1} Hz",
                axis,
                kind,
                period,
                frequency_hz
            );
        }
    }
    fn recover_from_error(&mut self) -> bool {
        let (Some(yaw_angle), Some(pitch_angle)) = (
            self.yaw.measured_joint_angle_rad(),
            self.pitch.measured_joint_angle_rad(),
        ) else {
            return false;
        };

        let (Some(yaw_config), Some(pitch_config)) =
            (self.yaw.motion_config, self.pitch.motion_config)
        else {
            return false;
        };

        if !yaw_config.contains(yaw_angle) || !pitch_config.contains(pitch_angle) {
            return false;
        }

        // 不回到旧目标：从当前位置开始
        self.yaw.target_angle_rad = yaw_angle;
        self.pitch.target_angle_rad = pitch_angle;

        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;

        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();

        self.yaw_fault.reset();
        self.pitch_fault.reset();
        self.prepare_progress.reset();

        self.disable_all();
        self.enable_armed = false;
        self.recovery_phase = RecoveryPhase::NeedRelease;
        self.state = State::Running;

        rtt_target::rprintln!(
            "Recovery: yaw={} pitch={}, targets held, PID reset",
            yaw_angle,
            pitch_angle
        );

        true
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[cfg(not(test))]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[cfg(not(test))]
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut debug_ticks = 0u16;
    let mut pid_log_ticks = 0u16;
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        pid_log_ticks += 1;
        if pid_log_ticks >= 200 {
            pid_log_ticks = 0;

            rtt_target::rprintln!(
                "state={:?} prepare_ms={} stable_ms={} yaw: stall_ms={} osc_hz={:.1} osc_ms={} cycles={} pitch: stall_ms={} osc_hz={:.1} osc_ms={} cycles={}",
                control.state,
                control.prepare_progress.elapsed_ticks,
                control.prepare_progress.stable_ticks,
                control.yaw_fault.stall_ticks,
                control.yaw_fault.frequency_hz(),
                control.yaw_fault.fast_ticks,
                control.yaw_fault.fast_cycles,
                control.pitch_fault.stall_ticks,
                control.pitch_fault.frequency_hz(),
                control.pitch_fault.fast_ticks,
                control.pitch_fault.fast_cycles,
            );
            if matches!(control.state, State::Prepare | State::Running) {
                rtt_target::rprintln!(
                    "YAW target_angle={:.3} actual_angle={:?} target_rpm={:.1} rpm={:.1} out={:.0}",
                    control.yaw.target_angle_rad,
                    control.yaw.measured_joint_angle_rad(),
                    control.yaw.target_rpm,
                    control.yaw.motor.speed_rpm,
                    control.yaw.motor.command_voltage_raw,
                );

                rtt_target::rprintln!(
                    "PITCH target={:.3} actual={:?} target_rpm={:.1} rpm={:.1} out={:.0} current_raw={} integral={:.3} gravity_gain={:.0}",
                    control.pitch.target_angle_rad,
                    control.pitch.measured_joint_angle_rad(),
                    control.pitch.target_rpm,
                    control.pitch.motor.speed_rpm,
                    control.pitch.motor.command_voltage_raw,
                    control.pitch.motor.current_raw,
                    control.pitch.speed_pid.integral,
                    control.pitch.gravity_gain_raw,
                );
            }
        }
        debug_ticks += 1;

        if debug_ticks >= 1000 {
            debug_ticks = 0;
            rtt_target::rprintln!(
                "CAN command id=0x{:03X} data={:02X?} tx_failed={}",
                MOTOR_COMMAND_ID,
                control.last_command_data,
                TX_FAILED.load(Ordering::Relaxed),
            );
            rtt_target::rprintln!(
                "PID CONFIG: pos_kp={:.1} pos_max={:.1} speed_kp={:.1} speed_ki={:.1} speed_max={:.1} integral={:.3} i_limit={:.1} rate={:.3} bench={:.0}",
                control.yaw.position_pid.kp,
                control.yaw.position_pid.maxout,
                control.yaw.speed_pid.kp,
                control.yaw.speed_pid.ki,
                control.yaw.speed_pid.maxout,
                control.yaw.speed_pid.integral,
                control.yaw.speed_pid.integral_limit,
                control
                    .yaw
                    .motion_config
                    .map(|c| c.max_target_rate_rad_s)
                    .unwrap_or(0.0),
                BENCH_MAX_ABS_RAW,
            );
        }
        DEBUG_TICKS.fetch_add(1, Ordering::Relaxed);

        let state_code = match control.state {
            State::Waiting => 0,
            State::Prepare => 1,
            State::Running => 2,
            State::Error => 3,
        };

        DEBUG_STATE.store(state_code, Ordering::Relaxed);
        watchdog.pet();
    }
}
