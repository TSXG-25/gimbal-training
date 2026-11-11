use super::*;
use dm_mc02::{Dr16, Envelope};

fn remote(right_switch: u8, left_switch: u8, yaw: i16, pitch: i16) -> Dr16 {
    Dr16 {
        channels: [0, 0, yaw, pitch, 0],
        right_switch,
        left_switch,
    }
}
fn feedback(can: &mut Can1, id: u16, encoder: u16, rpm: i16, current: i16) {
    let e = encoder.to_be_bytes();
    let s = rpm.to_be_bytes();
    let c = current.to_be_bytes();
    let frame = Frame::new_standard(id, &[e[0], e[1], s[0], s[1], c[0], c[1], 0, 0]).unwrap();
    can.received.push_back(Ok(Envelope { frame }));
}
fn tick(c: &mut Control, can: &mut Can1, r: Option<Dr16>, yaw: u16, pitch: u16) {
    dm_mc02::REMOTE.signal(r);
    feedback(can, YAW_FEEDBACK_ID, yaw, 0, 0);
    feedback(can, PITCH_FEEDBACK_ID, pitch, 0, 0);
    c.tick(can);
}
fn ready() -> Control {
    let mut c = Control::default();
    c.yaw.motor.encoder_raw = 3564;
    c.pitch.motor.encoder_raw = 4110;
    c.yaw_feedback_age_ticks = 0;
    c.pitch_feedback_age_ticks = 0;
    c.remote = Some(remote(3, 3, 0, 0));
    c
}
fn assert_zero(c: &Control, can: &Can1) {
    assert_eq!(c.yaw.motor.command_voltage_raw, 0.0);
    assert_eq!(c.pitch.motor.command_voltage_raw, 0.0);
    assert_eq!(can.sent.last().unwrap().data, vec![0; 8]);
}

#[test]
fn boot_requires_down_then_middle_and_enters_prepare() {
    let mut c = Control::default();
    let mut can = Can1::default();
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3664, 4210);
    assert_eq!(c.state, State::Waiting);
    assert_zero(&c, &can);
    tick(&mut c, &mut can, Some(remote(2, 3, 0, 0)), 3664, 4210);
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3664, 4210);
    assert_eq!(c.state, State::Prepare);
    assert_eq!(c.yaw.target_angle_rad, 0.0);
    assert_eq!(c.pitch.target_angle_rad, 0.0);
}

#[test]
fn running_detects_fast_oscillation_even_while_stick_is_held() {
    let mut c = ready();
    c.state = State::Running;
    c.remote = Some(remote(3, 3, 100, 0));
    for tick in 0..200 {
        c.yaw.motor.encoder_raw = if (tick / 20) % 2 == 0 { 3664 } else { 3464 };
        c.check_faults();
        if c.state == State::Error {
            break;
        }
    }
    assert_eq!(c.state, State::Error);
    assert_eq!(c.yaw.motor.command_voltage_raw, 0.0);
    assert_eq!(c.pitch.motor.command_voltage_raw, 0.0);
}

#[test]
fn prepare_ignores_sticks_and_requires_continuous_settling() {
    let mut c = ready();
    let mut can = Can1::default();
    c.enter_prepare();
    for _ in 0..100 {
        tick(&mut c, &mut can, Some(remote(3, 3, 660, -660)), 3564, 4110);
        assert_eq!(c.state, State::Prepare);
        assert_eq!(c.yaw.target_angle_rad, 0.0);
        assert_eq!(c.pitch.target_angle_rad, 0.0);
    }
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3664, 4110);
    assert_eq!(c.prepare_progress.stable_ticks, 0);
    for _ in 0..199 {
        tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
        assert_eq!(c.state, State::Prepare);
    }
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
    assert_eq!(c.state, State::Running);
    assert_zero(&c, &can);
}

#[test]
fn prepare_timeout_latches_error_and_zeroes_can() {
    let mut c = ready();
    let mut can = Can1::default();
    c.enter_prepare();
    // Small unresolved error stays below the chosen stall-demand threshold.
    for _ in 0..prepare_logic::TIMEOUT_TICKS {
        tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3624, 4170);
    }
    assert_eq!(c.state, State::Error);
    assert_zero(&c, &can);
}

#[test]
fn both_prepare_axes_respect_speed_and_voltage_limits() {
    let mut c = ready();
    let mut can = Can1::default();
    c.enter_prepare();
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 5000, 4700);
    assert_eq!(c.state, State::Prepare);
    for axis in [&c.yaw, &c.pitch] {
        assert!(axis.target_rpm.abs() <= prepare_logic::PREPARE_MAX_RPM);
        assert!(axis.target_rpm.abs() <= axis.motion_config.unwrap().max_motor_rpm);
        assert!(axis.motor.command_voltage_raw.abs() <= axis.speed_pid.maxout);
    }
}

#[test]
fn left_stick_updates_each_target_and_release_holds_it() {
    let mut c = ready();
    let mut can = Can1::default();
    c.state = State::Running;
    tick(&mut c, &mut can, Some(remote(3, 3, 660, 0)), 3564, 4110);
    assert!((c.yaw.target_angle_rad + 0.0008).abs() < 0.000001);
    assert_eq!(c.pitch.target_angle_rad, 0.0);
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 660)), 3564, 4110);
    assert!((c.pitch.target_angle_rad - 0.0003).abs() < 0.000001);
    let held = (c.yaw.target_angle_rad, c.pitch.target_angle_rad);
    let mut r = remote(3, 3, 0, 0);
    r.channels[0] = 660;
    r.channels[1] = -660;
    tick(&mut c, &mut can, Some(r), 3564, 4110);
    assert_eq!(held, (c.yaw.target_angle_rad, c.pitch.target_angle_rad));
}

#[test]
fn up_and_down_disable_normal_operation_in_the_same_tick() {
    for switch in [1, 2] {
        let mut c = ready();
        let mut can = Can1::default();
        c.state = State::Running;
        tick(&mut c, &mut can, Some(remote(switch, 3, 0, 0)), 3564, 4110);
        assert_eq!(c.state, State::Waiting);
        assert_zero(&c, &can);
    }
}

#[test]
fn remote_invalid_frame_stops_immediately_and_reconnection_needs_new_enable() {
    let mut c = ready();
    let mut can = Can1::default();
    c.state = State::Running;
    tick(&mut c, &mut can, None, 3564, 4110);
    assert_eq!(c.state, State::Waiting);
    assert_zero(&c, &can);
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
    assert_eq!(c.state, State::Waiting);
    tick(&mut c, &mut can, Some(remote(2, 3, 0, 0)), 3564, 4110);
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
    assert_eq!(c.state, State::Prepare);
}

#[test]
fn remote_silence_times_out_after_100_ticks() {
    let mut c = ready();
    let mut can = Can1::default();
    c.state = State::Running;
    c.remote_silent_ticks = 0;
    for age in 1..=100 {
        feedback(&mut can, YAW_FEEDBACK_ID, 3564, 0, 0);
        feedback(&mut can, PITCH_FEEDBACK_ID, 4110, 0, 0);
        c.tick(&mut can);
        assert_eq!(
            c.state,
            if age < 100 {
                State::Running
            } else {
                State::Waiting
            }
        );
    }
    assert_zero(&c, &can);
}

#[test]
fn either_motor_feedback_timeout_stops_both_axes_and_disarms() {
    for missing_id in [YAW_FEEDBACK_ID, PITCH_FEEDBACK_ID] {
        let mut c = ready();
        let mut can = Can1::default();
        c.state = State::Running;
        c.enable_armed = true;
        for _ in 0..100 {
            dm_mc02::REMOTE.signal(Some(remote(3, 3, 0, 0)));
            if missing_id != YAW_FEEDBACK_ID {
                feedback(&mut can, YAW_FEEDBACK_ID, 3564, 0, 0);
            }
            if missing_id != PITCH_FEEDBACK_ID {
                feedback(&mut can, PITCH_FEEDBACK_ID, 4110, 0, 0);
            }
            c.tick(&mut can);
        }
        assert_eq!(c.state, State::Waiting);
        assert!(!c.enable_armed);
        assert_zero(&c, &can);
    }
}

#[test]
fn feedback_router_rejects_wrong_id_length_type_and_encoder() {
    let mut c = Control::default();
    let mut can = Can1::default();
    feedback(&mut can, 0x201, 3564, 10, 20);
    feedback(&mut can, YAW_FEEDBACK_ID, 8192, 10, 20);
    let mut f = Frame::new_standard(YAW_FEEDBACK_ID, &[0; 7]).unwrap();
    can.received.push_back(Ok(Envelope { frame: f.clone() }));
    f.data.push(0);
    f.id = Id::Extended(YAW_FEEDBACK_ID as u32);
    can.received.push_back(Ok(Envelope { frame: f.clone() }));
    f.id = Id::Standard(dm_mc02::StandardId(YAW_FEEDBACK_ID));
    f.remote = true;
    can.received.push_back(Ok(Envelope { frame: f }));
    can.received.push_back(Err(dm_mc02::BusError));
    c.read_feedback(&mut can);
    assert!(c.yaw_feedback_age_ticks >= FEEDBACK_TIMEOUT_TICKS);
    assert!(c.pitch_feedback_age_ticks >= FEEDBACK_TIMEOUT_TICKS);
    assert_eq!(c.yaw.motor.encoder_raw, 0);
    feedback(&mut can, YAW_FEEDBACK_ID, 8191, -123, -456);
    feedback(&mut can, PITCH_FEEDBACK_ID, 4110, 234, 567);
    c.read_feedback(&mut can);
    assert_eq!(c.yaw_feedback_age_ticks, 0);
    assert_eq!(c.pitch_feedback_age_ticks, 0);
    assert_eq!(c.yaw.motor.encoder_raw, 8191);
    assert_eq!(c.yaw.motor.speed_rpm, -123.0);
    assert_eq!(c.yaw.motor.current_raw, -456);
    assert_eq!(c.pitch.motor.speed_rpm, 234.0);
    assert_eq!(c.pitch.motor.current_raw, 567);
}

#[test]
fn stall_on_either_axis_zeroes_both_can_slots_on_fault_tick() {
    for pitch in [false, true] {
        let mut c = ready();
        let mut can = Can1::default();
        c.state = State::Running;
        if pitch {
            c.pitch.target_angle_rad = 0.3;
        } else {
            c.yaw.target_angle_rad = 0.3;
        }
        for tick_no in 1..=fault_logic::STALL_TICKS {
            tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
            assert_eq!(
                c.state,
                if tick_no < fault_logic::STALL_TICKS {
                    State::Running
                } else {
                    State::Error
                }
            );
        }
        assert_zero(&c, &can);
        let m = if pitch { &c.pitch_fault } else { &c.yaw_fault };
        assert_eq!(m.stall_ticks, fault_logic::STALL_TICKS);
        tick(&mut c, &mut can, None, 3564, 4110);
        tick(&mut c, &mut can, Some(remote(3, 1, 0, 0)), 3564, 4110);
        assert_eq!(c.state, State::Error);
        assert_zero(&c, &can);
        let m = if pitch { &c.pitch_fault } else { &c.yaw_fault };
        assert_eq!(m.stall_ticks, fault_logic::STALL_TICKS);
    }
}

#[test]
fn manual_recovery_resets_histories_and_holds_current_joint_angles() {
    let mut c = ready();
    let mut can = Can1::default();
    c.state = State::Error;
    c.yaw.target_angle_rad = 1.0;
    c.pitch.target_angle_rad = -0.4;
    c.yaw.position_pid.integral = 7.0;
    c.pitch.speed_pid.integral = 8.0;
    for _ in 0..3 {
        tick(&mut c, &mut can, Some(remote(3, 1, 0, 0)), 3664, 4210);
    }
    assert_eq!(c.state, State::Error);
    tick(&mut c, &mut can, Some(remote(3, 2, 0, 0)), 3664, 4210);
    assert_eq!(c.state, State::Error);
    tick(&mut c, &mut can, Some(remote(3, 1, 0, 0)), 3664, 4210);
    assert_eq!(c.state, State::Running);
    assert_eq!(
        c.yaw.target_angle_rad,
        c.yaw.measured_joint_angle_rad().unwrap()
    );
    assert_eq!(
        c.pitch.target_angle_rad,
        c.pitch.measured_joint_angle_rad().unwrap()
    );
    assert_eq!(c.yaw.target_rpm, 0.0);
    assert_eq!(c.pitch.target_rpm, 0.0);
    assert_eq!(c.yaw.position_pid.integral, 0.0);
    assert_eq!(c.pitch.speed_pid.integral, 0.0);
    assert_eq!(c.yaw_fault.stall_ticks, 0);
    assert_eq!(c.pitch_fault.fast_cycles, 0);
}

#[test]
fn failed_recovery_requires_a_new_gesture_after_angle_is_corrected() {
    let mut c = ready();
    let mut can = Can1::default();
    c.state = State::Error;
    for left in [3, 2, 1] {
        tick(&mut c, &mut can, Some(remote(3, left, 0, 0)), 3564, 5110);
    }
    assert_eq!(c.state, State::Error);
    tick(&mut c, &mut can, Some(remote(3, 1, 0, 0)), 3564, 4110);
    assert_eq!(c.state, State::Error);
    tick(&mut c, &mut can, Some(remote(3, 2, 0, 0)), 3564, 4110);
    tick(&mut c, &mut can, Some(remote(3, 1, 0, 0)), 3564, 4110);
    assert_eq!(c.state, State::Running);
}

#[test]
fn noncentered_sticks_do_not_allow_recovery() {
    let mut c = ready();
    let mut can = Can1::default();
    c.state = State::Error;
    for left in [3, 2, 1] {
        tick(&mut c, &mut can, Some(remote(3, left, 100, 0)), 3564, 4110);
    }
    assert_eq!(c.state, State::Error);
    assert_zero(&c, &can);
}

#[test]
fn gravity_uses_measured_angle_on_both_control_paths_and_can_is_bounded() {
    for state in [State::Prepare, State::Running] {
        for delta in [-650, 0, 650] {
            let mut c = ready();
            let mut can = Can1::default();
            c.state = state;
            c.pitch.position_pid.kp = 0.0;
            tick(
                &mut c,
                &mut can,
                Some(remote(3, 3, 0, 0)),
                3564,
                (4110 + delta) as u16,
            );
            let angle = c.pitch.measured_joint_angle_rad().unwrap();
            assert!((c.pitch.motor.command_voltage_raw - 4773.0 * angle.cos()).abs() < 0.01);
            assert_eq!(c.yaw.motor.command_voltage_raw, 0.0);
            assert_eq!(can.sent.last().unwrap().data[4..], [0; 4]);
            assert_eq!(
                c.last_command_data.as_slice(),
                can.sent.last().unwrap().data
            );
        }
    }
}

#[test]
fn zero_speed_loop_limit_disables_feedforward_and_total_output_is_clamped() {
    for limit in [0.0, 1000.0] {
        let mut c = ready();
        let mut can = Can1::default();
        c.state = State::Running;
        c.pitch.speed_pid.maxout = limit;
        tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
        assert_eq!(c.pitch.motor.command_voltage_raw, limit);
    }
}

#[test]
fn pid_proportional_integral_derivative_reset_and_limits_match_readme() {
    let mut p = Pid {
        kp: 2.0,
        maxout: 100.0,
        ..Pid::default()
    };
    assert_eq!(p.calculate(10.0, 7.0, DT_S), 6.0);
    p.maxout = 5.0;
    assert_eq!(p.calculate(10.0, 7.0, DT_S), 5.0);
    assert_eq!(p.calculate(7.0, 10.0, DT_S), -5.0);
    p.maxout = 0.0;
    assert_eq!(p.calculate(10.0, 7.0, DT_S), 0.0);
    let mut i = Pid {
        ki: 4.0,
        integral_limit: 1.5,
        maxout: 100.0,
        ..Pid::default()
    };
    assert_eq!(i.calculate(2.0, 0.0, 0.5), 4.0);
    assert_eq!(i.calculate(2.0, 0.0, 0.5), 6.0);
    i.reset();
    assert_eq!(i.integral, 0.0);
    assert!(!i.has_previous);
    let mut d = Pid {
        kd: 1.0,
        maxout: 100.0,
        ..Pid::default()
    };
    assert_eq!(d.calculate(10.0, 0.0, 0.1), 0.0);
    assert_eq!(d.calculate(10.0, 2.0, 0.1), -20.0);
    d.reset();
    assert_eq!(d.calculate(10.0, 2.0, 0.1), 0.0);
}

#[test]
fn encoder_wrap_and_signed_voltage_encoding_match_protocol() {
    assert_eq!(shortest_delta_counts(2, 8190), 4);
    assert_eq!(shortest_delta_counts(8190, 2), -4);
    let mut data = [0; 8];
    write_voltage_slot(&mut data, 1, 1000.0, 2000.0);
    write_voltage_slot(&mut data, 2, -500.0, 2000.0);
    assert_eq!(data, [0x03, 0xe8, 0xfe, 0x0c, 0, 0, 0, 0]);
    write_voltage_slot(&mut data, 1, 30000.0, 30000.0);
    write_voltage_slot(&mut data, 2, -30000.0, 30000.0);
    assert_eq!(&data[..4], &[0x61, 0xa8, 0x9e, 0x58]);
    write_voltage_slot(&mut data, 1, 123.0, 0.0);
    assert_eq!(&data[..2], &[0, 0]);
}

#[test]
fn full_can_transmit_queue_is_counted_without_panicking() {
    let mut c = ready();
    let mut can = Can1 {
        fail_transmit: true,
        ..Can1::default()
    };
    let before = TX_FAILED.load(Ordering::Relaxed);
    c.transmit(&mut can);
    assert!(TX_FAILED.load(Ordering::Relaxed) > before);
    assert!(can.sent.is_empty());
}

#[test]
fn offline_session_start_home_move_hold_stop_fault_and_recover() {
    let mut c = Control::default();
    let mut can = Can1::default();
    for right in [2, 3] {
        tick(&mut c, &mut can, Some(remote(right, 3, 0, 0)), 3664, 4210);
    }
    println!(
        "SESSION enable: {:?}, targets=({:.3},{:.3}), CAN={:02X?}",
        c.state, c.yaw.target_angle_rad, c.pitch.target_angle_rad, c.last_command_data
    );
    assert_eq!(c.state, State::Prepare);
    for _ in 0..200 {
        tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
    }
    assert_eq!(c.state, State::Running);
    tick(&mut c, &mut can, Some(remote(3, 3, -660, 660)), 3564, 4110);
    let target = (c.yaw.target_angle_rad, c.pitch.target_angle_rad);
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
    assert_eq!(target, (c.yaw.target_angle_rad, c.pitch.target_angle_rad));
    println!(
        "SESSION release: {:?}, targets=({:.4},{:.4}), out=({:.1},{:.1})",
        c.state,
        target.0,
        target.1,
        c.yaw.motor.command_voltage_raw,
        c.pitch.motor.command_voltage_raw
    );
    tick(&mut c, &mut can, Some(remote(2, 3, 0, 0)), 3564, 4110);
    assert_zero(&c, &can);
    println!(
        "SESSION disable: {:?}, CAN={:02X?}",
        c.state, c.last_command_data
    );
    c.state = State::Running;
    c.pitch.target_angle_rad = 0.3;
    for _ in 0..500 {
        tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
    }
    assert_eq!(c.state, State::Error);
    assert_zero(&c, &can);
    println!(
        "SESSION fault: {:?}, stall_ms={}, CAN={:02X?}",
        c.state, c.pitch_fault.stall_ticks, c.last_command_data
    );
    for left in [3, 2, 1] {
        tick(&mut c, &mut can, Some(remote(3, left, 0, 0)), 3664, 4210);
    }
    assert_eq!(c.state, State::Running);
    println!(
        "SESSION recover: {:?}, targets=({:.3},{:.3}), actual=({:.3},{:.3})",
        c.state,
        c.yaw.target_angle_rad,
        c.pitch.target_angle_rad,
        c.yaw.measured_joint_angle_rad().unwrap(),
        c.pitch.measured_joint_angle_rad().unwrap()
    );
}

#[test]
fn oscillation_on_either_axis_stops_prepare_and_running_through_tick() {
    for state in [State::Prepare, State::Running] {
        for pitch_axis in [false, true] {
            let mut c = ready();
            let mut can = Can1::default();
            c.state = state;
            for n in 0..200 {
                let delta = if (n / 20) % 2 == 0 { 100 } else { -100 };
                let yaw = if pitch_axis {
                    3564
                } else {
                    (3564 + delta) as u16
                };
                let pitch = if pitch_axis {
                    (4110 + delta) as u16
                } else {
                    4110
                };
                tick(&mut c, &mut can, Some(remote(3, 3, 100, 100)), yaw, pitch);
                if c.state == State::Error {
                    break;
                }
            }
            assert_eq!(c.state, State::Error);
            assert_zero(&c, &can);
            let monitor = if pitch_axis {
                &c.pitch_fault
            } else {
                &c.yaw_fault
            };
            assert_eq!(monitor.fast_cycles, 4);
            assert_eq!(monitor.fast_ticks, 160);
            assert!((monitor.frequency_hz() - 25.0).abs() < 0.01);
        }
    }
}

#[test]
fn out_of_range_feedback_and_missing_calibration_never_leave_active_output() {
    for state in [State::Prepare, State::Running] {
        let mut c = ready();
        let mut can = Can1::default();
        c.state = state;
        tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 5110);
        assert_eq!(c.state, State::Error);
        assert_zero(&c, &can);
    }
    let mut c = ready();
    let mut can = Can1::default();
    c.state = State::Running;
    c.pitch.zero_encoder_count = None;
    tick(&mut c, &mut can, Some(remote(3, 3, 0, 0)), 3564, 4110);
    assert_eq!(c.state, State::Waiting);
    assert_zero(&c, &can);
}
