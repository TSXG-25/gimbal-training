pub fn write_voltage_slot(data: &mut [u8; 8], motor_id: u8, raw: f32, bench_limit: f32) {
    if !(1..=4).contains(&motor_id)
        || !raw.is_finite()
        || !bench_limit.is_finite()
        || bench_limit < 0.0
    {
        return;
    }

    let limit = bench_limit.min(25000.0);
    let clamped = raw.clamp(-limit, limit);

    let value = if clamped >= 0.0 {
        (clamped + 0.5) as i16
    } else {
        (clamped - 0.5) as i16
    };

    let bytes = value.to_be_bytes();

    let index = (motor_id as usize - 1) * 2;

    data[index] = bytes[0];
    data[index + 1] = bytes[1];
}
