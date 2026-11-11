//! Host replacements for UART input, CAN queues and RTT; control logic remains production code.
use std::cell::RefCell;
use std::collections::VecDeque;

#[derive(Clone, Copy)]
pub struct Dr16 {
    pub channels: [i16; 5],
    pub right_switch: u8,
    pub left_switch: u8,
}

thread_local! {
    static INPUT: RefCell<Option<Option<Dr16>>> = const { RefCell::new(None) };
}
pub struct RemoteSignal;
pub static REMOTE: RemoteSignal = RemoteSignal;
impl RemoteSignal {
    pub fn signal(&self, value: Option<Dr16>) {
        INPUT.with(|input| *input.borrow_mut() = Some(value));
    }
    pub fn try_take(&self) -> Option<Option<Dr16>> {
        INPUT.with(|input| input.borrow_mut().take())
    }
}

#[derive(Clone, Copy)]
pub struct StandardId(pub u16);
impl StandardId {
    pub fn as_raw(self) -> u16 {
        self.0
    }
}
#[derive(Clone, Copy)]
pub enum Id {
    Standard(StandardId),
    Extended(u32),
}
#[derive(Clone)]
pub struct Frame {
    pub id: Id,
    pub data: Vec<u8>,
    pub remote: bool,
}
impl Frame {
    pub fn new_standard(id: u16, data: &[u8]) -> Result<Self, ()> {
        assert!(id <= 0x7ff && data.len() <= 8);
        Ok(Self {
            id: Id::Standard(StandardId(id)),
            data: data.to_vec(),
            remote: false,
        })
    }
    pub fn is_remote_frame(&self) -> bool {
        self.remote
    }
    pub fn id(&self) -> Id {
        self.id
    }
    pub fn data(&self) -> &[u8] {
        &self.data
    }
}
pub struct Envelope {
    pub frame: Frame,
}
#[derive(Debug)]
pub struct BusError;
#[derive(Default)]
pub struct Can1 {
    pub received: VecDeque<Result<Envelope, BusError>>,
    pub sent: Vec<Frame>,
    pub fail_transmit: bool,
}
impl Can1 {
    pub fn reader(&mut self) -> &mut Self {
        self
    }
    pub fn writer(&mut self) -> &mut Self {
        self
    }
    pub fn try_receive(&mut self) -> Result<Result<Envelope, BusError>, ()> {
        self.received.pop_front().ok_or(())
    }
    pub fn try_write(&mut self, frame: Frame) -> Result<(), ()> {
        if self.fail_transmit {
            return Err(());
        }
        self.sent.push(frame);
        Ok(())
    }
}
pub fn log_can_error(error: BusError) {
    eprintln!("CAN error: {error:?}");
}
