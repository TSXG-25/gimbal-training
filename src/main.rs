#![no_std]
#![no_main]
#![forbid(unsafe_code)]
#[derive(Clone, Copy)]
enum RecoveryPhase {
    NeedRelease,
    NeedDown,
    NeedUp,
}

mod dm_mc02;
mod fault_logic;
mod prepare_logic;
mod running_logic;

use fault_logic::FaultMonitor;
use running_logic::STICK_DEADZONE;

use running_logic::{AxisMotionConfig, next_target_angle};

use prepare_logic::{PREPARE_MAX_RPM, PrepareOutcome, PrepareProgress, axes_settled};

const DT_S: f32 = 0.001;
const PID_TUNING_MODE: bool = true;

// Adjust: whether can raise verified flag
const HOMING_CONFIG_VERIFIED: bool = true;

mod motor_protocol; // delete_required
use motor_protocol::write_voltage_slot;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::{Frame as _, Id};

// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);

const FEEDBACK_TIMEOUT_TICKS: u16 = 100;

// Assumption：motor 1 = Yaw，motor 2 = Pitch
const YAW_MOTOR_ID: u8 = 1;
const PITCH_MOTOR_ID: u8 = 2;

const YAW_FEEDBACK_ID: u16 = 0x204 + YAW_MOTOR_ID as u16;
const PITCH_FEEDBACK_ID: u16 = 0x204 + PITCH_MOTOR_ID as u16;

// Adjust: locked inut
const BENCH_MAX_ABS_RAW: f32 = 10.0;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

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
        // TODO: decode angle, speed and current as the manual describes.
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
// TODO: add gains, history, an integral limit and a symmetric output limit `maxout`.
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
        self.motor.command_voltage_raw =
            self.speed_pid
                .calculate(self.target_rpm, self.motor.speed_rpm, DT_S);

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

        self.motor.command_voltage_raw =
            self.speed_pid
                .calculate(self.target_rpm, self.motor.speed_rpm, DT_S);

        if !self.motor.command_voltage_raw.is_finite() {
            self.motor.disable();
            return None;
        }

        Some(())
    }
}

/// State shared by every control tick.
// TODO: add feedback ages and a fault record.
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
                    kp: 0.0, // Yaw 位置环 Kp
                    ki: 0.0,
                    kd: 0.0,
                    maxout: 0.0, // 输出最大目标 rpm
                    integral_limit: 0.0,
                    ..Pid::default()
                },

                speed_pid: Pid {
                    kp: 2.0, // Yaw 速度环 Kp
                    ki: 0.0,
                    kd: 0.0,
                    maxout: 10.0, // 输出最大电压原始值
                    integral_limit: 0.0,
                    ..Pid::default()
                },

                motion_config: Some(AxisMotionConfig {
                    min_angle_rad: -2.618,
                    max_angle_rad: 2.618,
                    max_target_rate_rad_s: 0.3,
                    max_motor_rpm: 15.0,
                    stick_direction: 1.0,
                }),

                ..Axis::default()
            },

            // ========== PITCH PID ==========
            pitch: Axis {
                zero_encoder_count: Some(4110),
                position_pid: Pid {
                    kp: 0.0, // Pitch 位置环 Kp
                    ki: 0.0,
                    kd: 0.0,
                    maxout: 0.0,
                    integral_limit: 0.0,
                    ..Pid::default()
                },

                speed_pid: Pid {
                    kp: 0.0, // Pitch 速度环 Kp
                    ki: 0.0,
                    kd: 0.0,
                    maxout: 0.0,
                    integral_limit: 0.0,
                    ..Pid::default()
                },

                motion_config: Some(AxisMotionConfig {
                    min_angle_rad: -0.21,
                    max_angle_rad: 0.24,
                    max_target_rate_rad_s: 0.2,
                    max_motor_rpm: 10.0,
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
                    // TODO: check the ID and length, pass the data to the matching axis's
                    // `Motor::read`, and record its feedback age.
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
    // TODO: the transitions, and initialising targets and PIDs on entering a state.
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
                    if PID_TUNING_MODE {
                        self.enter_pid_tuning();
                    } else {
                        self.enter_prepare();
                    }
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
                                self.recover_from_error();
                            }
                        }
                    }
                }
            }
        }
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    // TODO: limited recentering in Prepare, remote cascade control in Running, and
    // `disable_all` in Waiting and Error. Each tick is 0.001 s.
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
        // TODO: clamp each voltage count output within the bench limit (protocol +/-25000),
        // encode signed i16 big-endian and write it into the
        // bytes for its motor ID as the manual describes. Other motor IDs stay zero.
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
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn enter_pid_tuning(&mut self) {
        let (Some(yaw_angle), Some(pitch_angle)) = (
            self.yaw.measured_joint_angle_rad(),
            self.pitch.measured_joint_angle_rad(),
        ) else {
            self.enable_armed = false;
            return;
        };

        // 检查当前角度是否在配置的活动范围内
        let (Some(yaw_config), Some(pitch_config)) =
            (self.yaw.motion_config, self.pitch.motion_config)
        else {
            self.enable_armed = false;
            return;
        };

        if !yaw_config.contains(yaw_angle) || !pitch_config.contains(pitch_angle) {
            self.enable_armed = false;
            return;
        }

        // 不自动回零：当前位置就是初始目标
        self.yaw.target_angle_rad = yaw_angle;
        self.pitch.target_angle_rad = pitch_angle;

        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;

        // 清除四套 PID 的历史值
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();

        self.prepare_progress.reset();
        self.yaw_fault.reset();
        self.pitch_fault.reset();

        self.disable_all();

        self.enable_armed = false;
        self.state = State::Running;

        rtt_target::rprintln!(
            "PID tuning started: yaw={:.3}, pitch={:.3}",
            yaw_angle,
            pitch_angle
        );
    }

    fn enter_prepare(&mut self) {
        self.yaw.begin_prepare();
        self.pitch.begin_prepare();

        self.prepare_progress.reset();
        self.enable_armed = false;
        self.state = State::Prepare;
    }

    fn check_faults(&mut self) {
        if !matches!(self.state, State::Prepare | State::Running) {
            self.yaw_fault.reset();
            self.pitch_fault.reset();
            return;
        }

        let Some(yaw_angle) = self.yaw.measured_joint_angle_rad() else {
            return;
        };
        let Some(pitch_angle) = self.pitch.measured_joint_angle_rad() else {
            return;
        };

        let yaw_osc_check = self.state == State::Prepare
            || self
                .remote
                .map(|r| r.channels[2].abs() <= STICK_DEADZONE)
                .unwrap_or(false);

        let pitch_osc_check = self.state == State::Prepare
            || self
                .remote
                .map(|r| r.channels[3].abs() <= STICK_DEADZONE)
                .unwrap_or(false);

        let yaw_fault = self.yaw_fault.step(
            self.yaw.target_rpm,
            self.yaw.motor.speed_rpm,
            self.yaw
                .motor
                .command_voltage_raw
                .abs()
                .min(BENCH_MAX_ABS_RAW),
            self.yaw.target_angle_rad - yaw_angle,
            yaw_osc_check,
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
            pitch_osc_check,
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

            let frequency_hz = if period > 0 { 1000 / period as u32 } else { 0 };

            rtt_target::rprintln!(
                "FAULT: {} {:?}, period={} ms, estimated={} Hz",
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
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
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
        watchdog.pet();
    }
}
