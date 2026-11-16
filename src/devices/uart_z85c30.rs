//! Zilog Z85C30 ESCC inside the SGI Indy's IOC2.
//!
//! The port starts polled, which is all the early boot can use. Once the
//! INT3 controller is set up, `enable_interrupts` turns on the receive
//! interrupt, and optionally a transmit queue drained by the transmit
//! interrupt for a port that only carries a copy of the console.
//!
//! Registers are indirect: write the register number to the control port,
//! then the value to the same port. The data register has its own port.
//! Channel B is the Indy's rear-panel "Serial Port 1" (the PROM's `serial0`).

use core::ptr::{read_volatile, write_volatile};

/// RR0 bit 0: a received character is available.
const RR0_RX_AVAILABLE: u8 = 0x01;
/// RR0 bit 2: the transmit buffer is empty.
const RR0_TX_EMPTY: u8 = 0x04;

/// 9600 baud from the 3.672 MHz PCLK in x16 clock mode:
/// baud = PCLK / (2 * 16 * (tc + 2)); tc = 10 gives 9562.
const BAUD_9600_TIME_CONSTANT: u8 = 10;

/// Polls of RR0 before the transmitter is declared stuck: about 40 ms, where
/// a character takes 1 ms.
const TX_POLL_LIMIT: u32 = 20_000;

/// Transmit queue size. At 9600 baud it drains in about two seconds.
const TX_QUEUE: usize = 2048;

pub struct SerialPort {
    control: *mut u8,
    data: *mut u8,
    /// The transmitter never became ready once; output is dropped from then
    /// on so a dead port cannot hang the console.
    tx_stuck: bool,
    /// Output goes through `queue` and the transmit interrupt, and is
    /// dropped when the queue is full. Off: `send` waits for the chip.
    queued: bool,
    queue: [u8; TX_QUEUE],
    head: usize,
    len: usize,
}

/// Recovery time between accesses to the chip. The Z85C30 needs a few PCLK
/// cycles after each access; back-to-back accesses can corrupt a register
/// write on real hardware. Eight uncached bus reads take over a microsecond.
#[inline(never)]
fn recover() {
    const MC_SYSID: *const u32 = 0xBFA0_001C as *const u32;
    for _ in 0..8 {
        let _ = unsafe { read_volatile(MC_SYSID) };
    }
}

// The port is only ever reached through the COM1 mutex.
unsafe impl Send for SerialPort {}

impl SerialPort {
    /// `control` is the kseg1 address of the channel's control port; the data
    /// port is the next word (same byte lane).
    pub const unsafe fn new(control: usize) -> Self {
        Self {
            control: control as *mut u8,
            data: (control + 4) as *mut u8,
            tx_stuck: false,
            queued: false,
            queue: [0; TX_QUEUE],
            head: 0,
            len: 0,
        }
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        unsafe {
            write_volatile(self.control, reg);
            recover();
            write_volatile(self.control, value);
            recover();
        }
    }

    fn rr0(&self) -> u8 {
        let value = unsafe { read_volatile(self.control) };
        recover();
        value
    }

    /// 9600 8N1, no interrupts, DTR and RTS asserted.
    pub fn init(&mut self) {
        self.write_reg(9, 0x40); // channel reset B
        self.write_reg(4, 0x44); // x16 clock, 1 stop bit, no parity
        self.write_reg(3, 0xc0); // rx 8 bits per character, receiver off
        self.write_reg(5, 0x60); // tx 8 bits per character, transmitter off
        self.write_reg(9, 0x00); // no interrupts
        self.write_reg(11, 0x50); // tx and rx clocks from the baud rate generator
        self.write_reg(12, BAUD_9600_TIME_CONSTANT);
        self.write_reg(13, 0x00);
        self.write_reg(14, 0x02); // baud rate generator source PCLK
        self.write_reg(14, 0x03); // baud rate generator on
        self.write_reg(3, 0xc1); // receiver on
        self.write_reg(5, 0xea); // DTR, tx 8 bits, transmitter on, RTS
    }

    /// Turn on the receive interrupt, and with `queue_output` the transmit
    /// interrupt and queue. The chip's interrupt reaches the CPU through
    /// INT3's mappable line 5.
    pub fn enable_interrupts(&mut self, queue_output: bool) {
        self.queued = queue_output;
        self.write_reg(15, 0x00); // no external/status interrupts
        // WR1: receive interrupt on every character or special condition
        // (bits 4:3 = 10), transmit interrupt (bit 1) for a queued port.
        self.write_reg(1, if queue_output { 0x12 } else { 0x10 });
        self.write_reg(9, 0x0a); // master interrupt enable, no vector
    }

    /// Service the transmit side from the interrupt: send the next queued
    /// byte, or clear the pending transmit interrupt when there is none.
    pub fn service(&mut self) {
        // Error reset, for an overrun or framing error latched with the
        // character, then the highest interrupt under service.
        unsafe {
            write_volatile(self.control, 0x30);
            recover();
        }
        if self.queued && self.rr0() & RR0_TX_EMPTY != 0 {
            if self.len != 0 {
                let byte = self.queue[self.head];
                self.head = (self.head + 1) % TX_QUEUE;
                self.len -= 1;
                unsafe { write_volatile(self.data, byte) }
                recover();
            } else {
                unsafe {
                    write_volatile(self.control, 0x28); // reset Tx interrupt pending
                    recover();
                }
            }
        }
        unsafe {
            write_volatile(self.control, 0x38); // reset highest IUS
            recover();
        }
    }

    fn enqueue(&mut self, byte: u8) {
        if self.len == 0 && self.rr0() & RR0_TX_EMPTY != 0 {
            unsafe { write_volatile(self.data, byte) }
            recover();
        } else if self.len < TX_QUEUE {
            self.queue[(self.head + self.len) % TX_QUEUE] = byte;
            self.len += 1;
        }
    }

    pub fn receive(&mut self) -> Option<u8> {
        if self.rr0() & RR0_RX_AVAILABLE != 0 {
            let byte = unsafe { read_volatile(self.data) };
            recover();
            Some(byte)
        } else {
            None
        }
    }

    pub fn send(&mut self, byte: u8) {
        if self.tx_stuck {
            return;
        }
        let mut polls = 0;
        while self.rr0() & RR0_TX_EMPTY == 0 {
            polls += 1;
            if polls > TX_POLL_LIMIT {
                self.tx_stuck = true;
                return;
            }
        }
        unsafe { write_volatile(self.data, byte) }
        recover();
    }

    pub fn write(&mut self, buf: &[u8]) {
        for &byte in buf {
            if self.queued {
                if byte == b'\n' {
                    self.enqueue(b'\r');
                }
                self.enqueue(byte);
                continue;
            }
            if byte == b'\n' {
                self.send(b'\r');
            }
            self.send(byte);
        }
    }
}
