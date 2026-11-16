//! Text console on the Indy's Newport (XL) graphics board.
//!
//! The kernel draws its console itself in an 8 by 16 font through the REX3
//! drawing engine: 160 columns by 64 rows at 1280 by 1024, 128 by 48 at
//! 1024 by 768. The size comes from the VC2's video timing table. The PROM
//! is not called. It has already brought the board up (video timing, pixel
//! format, colour map), and that state is kept: the console sets only the
//! drawing registers it uses.
//!
//! REX3 is at physical 0x1F0F_0000. A register written at its offset plus
//! 0x800 also starts the command ("go"). Three commands are used:
//!
//! - a character: block draw with the Z pattern as a one-bit source, opaque,
//!   one 8-pixel span for each of 16 pattern writes (foreground in COLORI,
//!   background in COLORBACK);
//! - a filled rectangle: block draw in COLORI;
//! - a scroll: screen-to-screen block copy by XYMOVE.
//!
//! Colours are indices 0 to 7 of the PROM's colour map, which are black,
//! red, green, yellow, blue, magenta, cyan and white, the order of the ANSI
//! colours.
//!
//! The terminal understands the VT100/xterm subset that the shell, the line
//! editor, `less` and `kibi` send: cursor movement, erase, insert and delete,
//! SGR colours, save and restore, cursor visibility and the cursor position
//! report. Replies to reports are queued for the keyboard poll, which feeds
//! them to the console input.
//!
//! Register facts: SGI's REX3 specification as summarised in the IRIX 7
//! project's `docs/research/indy-hardware.md`, and the register model of the
//! iris emulator, which the gate runs against.

use core::sync::atomic::{AtomicBool, Ordering};

use spin::Mutex;

const REX3: usize = 0xBF0F_0000;
const GO: usize = 0x0800;

const DRAWMODE1: usize = 0x0000;
const DRAWMODE0: usize = 0x0004;
const ZPATTERN: usize = 0x0014;
const COLORBACK: usize = 0x0018;
const XYMOVE: usize = 0x0114;
const XYSTARTI: usize = 0x0150;
const XYENDI: usize = 0x0154;
const WRMASK: usize = 0x0220;
const COLORI: usize = 0x0224;
const TOPSCAN: usize = 0x1320;
const XYWIN: usize = 0x1324;
const CLIPMODE: usize = 0x1328;
const STATUS: usize = 0x1338;

const STATUS_GFXBUSY: u32 = 1 << 3;
const STATUS_BACKBUSY: u32 = 1 << 4;

// The display control bus, which reaches the VC2 video timing controller.
const DCBMODE: usize = 0x0238;
const DCBDATA0: usize = 0x0240;
/// DCBMODE for the VC2 (bus address 0): asynchronous acknowledge, one
/// cycle of chip select setup and hold.
const VC2_PROTOCOL: u32 = 0x0000_1000 | 0x0080_0000 | 0x0004_0000;
const DCB_WIDTH_1: u32 = 1;
const DCB_WIDTH_2: u32 = 2;
const DCB_WIDTH_3: u32 = 3;
const DCB_REGISTER_INCREMENT: u32 = 1 << 3;
/// VC2 register select: the index register, the indexed register, the RAM.
const VC2_INDEX: u32 = 0x00;
const VC2_IREG: u32 = 0x10;
const VC2_RAM: u32 = 0x30;
/// VC2 indexed registers: video entry pointer and RAM address.
const VC2_VIDEO_ENTRY: u8 = 0x00;
const VC2_RAM_ADDRESS: u8 = 0x07;

// DRAWMODE0
const OP_DRAW: u32 = 2;
const OP_SCR2SCR: u32 = 3;
const ADR_BLOCK: u32 = 1 << 2;
const DOSETUP: u32 = 1 << 5;
const STOPONX: u32 = 1 << 8;
const STOPONY: u32 = 1 << 9;
const ENZPATTERN: u32 = 1 << 12;
const LENGTH32: u32 = 1 << 15;
const ZOPAQUE: u32 = 1 << 16;

// DRAWMODE1: the planes, depth and packing bits (0..11) stay as the PROM
// left them for its own text; comparison is off and the logic op is "source".
const DM1_KEEP: u32 = 0x0000_0fff;
const DM1_COMPARE_OFF: u32 = 7 << 12;
const DM1_LOGIC_SRC: u32 = 3 << 28;
/// Colour index planes, 8 bits deep, 8-bit packed host pixels: the value for
/// a board whose register reads back as zero.
const DM1_CI8: u32 = 1 | (1 << 3) | (1 << 7) | (1 << 8);

/// The largest screen: 1280 by 1024. The console uses as many columns and
/// rows as the video mode shows (`screen_size`).
const MAX_COLS: usize = 160;
const MAX_ROWS: usize = 64;
const CELL_W: usize = 8;
const CELL_H: usize = 16;

static FONT: &[u8] = include_bytes!("../../../../res/unifont.font");

static ACTIVE: AtomicBool = AtomicBool::new(false);

/// A program has the screen (`CSI ? 9900 h`): it maps REX3 itself and draws
/// its own frames. The console keeps its cell buffer up to date and draws
/// nothing until the program gives the screen back (`CSI ? 9900 l`) or the
/// user presses Control-Alt-Backspace. The keyboard sends key codes instead
/// of characters meanwhile (`kbd::set_raw`).
static LENT: AtomicBool = AtomicBool::new(false);

/// DRAWMODE1 for console drawing, kept so that the screen can be taken back
/// after a program changed it.
static CONSOLE_DRAWMODE1: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// DCBMODE for the colour maps (bus address 1, both): the address register
/// takes a 16-bit index, the palette buffer three bytes of red, green, blue.
const CMAP_PROTOCOL: u32 = 0x0000_8000 | 0x0004_0000 | 0x0080_0000;
const CMAP_ALL: u32 = 1 << 7;
const CMAP_ADDRESS: u32 = 0x00;
const CMAP_PALETTE: u32 = 0x20;
const DCB_SWAP: u32 = 1 << 28;

/// The console's eight colours, as the PROM loads them.
const CONSOLE_COLOURS: [[u8; 3]; 8] = [
    [0, 0, 0],
    [255, 0, 0],
    [0, 255, 0],
    [255, 255, 0],
    [0, 0, 255],
    [255, 0, 255],
    [0, 255, 255],
    [255, 255, 255],
];

fn rd(reg: usize) -> u32 {
    unsafe { ((REX3 + reg) as *const u32).read_volatile() }
}

fn wr(reg: usize, value: u32) {
    unsafe { ((REX3 + reg) as *mut u32).write_volatile(value) }
}

/// Wait for the drawing engine to finish before its mode registers change.
/// Bounded: a board that never goes idle must not hang the console.
fn wait() {
    for _ in 0..2_000_000 {
        if rd(STATUS) & STATUS_GFXBUSY == 0 {
            return;
        }
    }
}

fn xy(x: usize, y: usize) -> u32 {
    ((x as u32) << 16) | (y as u32 & 0xffff)
}

fn fill(x0: usize, y0: usize, x1: usize, y1: usize, colour: u8) {
    if LENT.load(Ordering::Relaxed) {
        return;
    }
    wait();
    wr(DRAWMODE0, OP_DRAW | ADR_BLOCK | DOSETUP | STOPONX | STOPONY);
    wr(COLORI, colour as u32);
    wr(XYSTARTI, xy(x0, y0));
    wr(GO + XYENDI, xy(x1, y1));
}

/// Copy rows `y0..=y1` of a `width` pixel wide screen by `dy` pixel rows
/// (negative: up).
fn copy_rows(width: usize, y0: usize, y1: usize, dy: i32) {
    if LENT.load(Ordering::Relaxed) {
        return;
    }
    wait();
    wr(DRAWMODE0, OP_SCR2SCR | ADR_BLOCK | DOSETUP | STOPONX | STOPONY);
    wr(XYSTARTI, xy(0, y0));
    wr(XYENDI, xy(width - 1, y1));
    wr(GO + XYMOVE, dy as u32 & 0xffff);
}

fn glyph(col: usize, row: usize, ch: u8, fg: u8, bg: u8) {
    if LENT.load(Ordering::Relaxed) {
        return;
    }
    let x = col * CELL_W;
    let y = row * CELL_H;
    wait();
    wr(
        DRAWMODE0,
        OP_DRAW | ADR_BLOCK | STOPONX | ENZPATTERN | LENGTH32 | ZOPAQUE,
    );
    wr(COLORI, fg as u32);
    wr(COLORBACK, bg as u32);
    wr(XYSTARTI, xy(x, y));
    wr(XYENDI, xy(x + CELL_W - 1, y + CELL_H - 1));
    let base = ch as usize * CELL_H;
    for line in 0..CELL_H {
        let bits = FONT.get(base + line).copied().unwrap_or(0);
        wr(GO + ZPATTERN, (bits as u32) << 24);
    }
}

#[derive(Clone, Copy)]
struct Cell {
    ch: u8,
    /// Foreground in the low nibble, background in the high one.
    colours: u8,
}

const DEFAULT_FG: u8 = 7;
const DEFAULT_BG: u8 = 0;
const BLANK: Cell = Cell {
    ch: b' ',
    colours: DEFAULT_FG | (DEFAULT_BG << 4),
};

#[derive(Clone, Copy, PartialEq)]
enum State {
    Ground,
    Escape,
    Csi,
    /// An operating system command or other string: skipped up to BEL or ST.
    String,
    StringEscape,
    /// ESC followed by an intermediate such as `(`: one more byte to skip.
    Skip,
}

struct Console {
    cells: [[Cell; MAX_COLS]; MAX_ROWS],
    cols: usize,
    rows: usize,
    col: usize,
    row: usize,
    /// The cursor is past the last column: the next character wraps first.
    wrap: bool,
    fg: u8,
    bg: u8,
    reverse: bool,
    saved: (usize, usize),
    cursor_on: bool,
    cursor_drawn: bool,
    state: State,
    params: [u16; 8],
    nparams: usize,
    private: bool,
    /// UTF-8 decoding: code point so far and continuation bytes to come.
    utf8: u32,
    utf8_left: u8,
    reply: [u8; 32],
    reply_len: usize,
}

static CONSOLE: Mutex<Console> = Mutex::new(Console {
    cells: [[BLANK; MAX_COLS]; MAX_ROWS],
    cols: MAX_COLS,
    rows: MAX_ROWS,
    col: 0,
    row: 0,
    wrap: false,
    fg: DEFAULT_FG,
    bg: DEFAULT_BG,
    reverse: false,
    saved: (0, 0),
    cursor_on: true,
    cursor_drawn: false,
    state: State::Ground,
    params: [0; 8],
    nparams: 0,
    private: false,
    utf8: 0,
    utf8_left: 0,
    reply: [0; 32],
    reply_len: 0,
});

impl Console {
    fn colours(&self) -> u8 {
        if self.reverse {
            self.bg | (self.fg << 4)
        } else {
            self.fg | (self.bg << 4)
        }
    }

    fn blank(&self) -> Cell {
        Cell {
            ch: b' ',
            colours: self.fg | (self.bg << 4),
        }
    }

    fn draw(&self, col: usize, row: usize) {
        let cell = self.cells[row][col];
        glyph(col, row, cell.ch, cell.colours & 0xf, cell.colours >> 4);
    }

    fn draw_row(&self, row: usize, from: usize) {
        for col in from..self.cols {
            self.draw(col, row);
        }
    }

    fn hide_cursor(&mut self) {
        if self.cursor_drawn {
            self.cursor_drawn = false;
            self.draw(self.col.min(self.cols - 1), self.row);
        }
    }

    fn show_cursor(&mut self) {
        if self.cursor_on && !self.cursor_drawn {
            let col = self.col.min(self.cols - 1);
            let cell = self.cells[self.row][col];
            glyph(col, self.row, cell.ch, cell.colours >> 4, cell.colours & 0xf);
            self.cursor_drawn = true;
        }
    }

    fn clear_rows(&mut self, from: usize, to: usize) {
        if from >= to {
            return;
        }
        let blank = self.blank();
        for row in from..to {
            self.cells[row] = [blank; MAX_COLS];
        }
        fill(0, from * CELL_H, self.cols * CELL_W - 1, to * CELL_H - 1, self.bg);
    }

    fn clear_cols(&mut self, row: usize, from: usize, to: usize) {
        if from >= to {
            return;
        }
        let blank = self.blank();
        for cell in &mut self.cells[row][from..to] {
            *cell = blank;
        }
        fill(
            from * CELL_W,
            row * CELL_H,
            to * CELL_W - 1,
            (row + 1) * CELL_H - 1,
            self.bg,
        );
    }

    /// Move rows `top + n..self.rows` up by `n` and blank the rows at the bottom.
    fn scroll_up(&mut self, top: usize, n: usize) {
        let n = n.min(self.rows - top);
        if n == 0 {
            return;
        }
        if top + n < self.rows {
            self.cells.copy_within(top + n..self.rows, top);
            copy_rows(self.cols * CELL_W, (top + n) * CELL_H, self.rows * CELL_H - 1, -((n * CELL_H) as i32));
        }
        self.clear_rows(self.rows - n, self.rows);
    }

    /// Move rows `top..self.rows - n` down by `n` and blank `n` rows at `top`.
    /// The engine copies top to bottom, which would smear an overlapping
    /// downward copy, so the rows are redrawn from the cell buffer.
    fn scroll_down(&mut self, top: usize, n: usize) {
        let n = n.min(self.rows - top);
        if n == 0 {
            return;
        }
        self.cells.copy_within(top..self.rows - n, top + n);
        let blank = self.blank();
        for row in top..top + n {
            self.cells[row] = [blank; MAX_COLS];
        }
        for row in top..self.rows {
            self.draw_row(row, 0);
        }
    }

    fn newline(&mut self) {
        self.wrap = false;
        if self.row + 1 < self.rows {
            self.row += 1;
        } else {
            self.scroll_up(0, 1);
        }
    }

    fn put(&mut self, ch: u8) {
        if self.wrap {
            self.col = 0;
            self.newline();
        }
        self.cells[self.row][self.col] = Cell {
            ch,
            colours: self.colours(),
        };
        self.draw(self.col, self.row);
        if self.col + 1 < self.cols {
            self.col += 1;
        } else {
            self.wrap = true;
        }
    }

    fn param(&self, index: usize, default: usize) -> usize {
        match self.params.get(index) {
            Some(&value) if index < self.nparams && value != 0 => value as usize,
            _ => default,
        }
    }

    fn reply(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if self.reply_len < self.reply.len() {
                self.reply[self.reply_len] = byte;
                self.reply_len += 1;
            }
        }
    }

    fn reply_number(&mut self, mut value: usize) {
        let mut digits = [0u8; 5];
        let mut count = 0;
        loop {
            digits[count] = b'0' + (value % 10) as u8;
            count += 1;
            value /= 10;
            if value == 0 || count == digits.len() {
                break;
            }
        }
        while count > 0 {
            count -= 1;
            self.reply(&[digits[count]]);
        }
    }

    fn sgr(&mut self) {
        if self.nparams == 0 {
            self.nparams = 1;
            self.params[0] = 0;
        }
        for index in 0..self.nparams {
            match self.params[index] {
                0 => {
                    self.fg = DEFAULT_FG;
                    self.bg = DEFAULT_BG;
                    self.reverse = false;
                }
                7 => self.reverse = true,
                27 => self.reverse = false,
                value @ 30..=37 => self.fg = (value - 30) as u8,
                39 => self.fg = DEFAULT_FG,
                value @ 40..=47 => self.bg = (value - 40) as u8,
                49 => self.bg = DEFAULT_BG,
                // Bright colours: the colour map past index 7 is not known,
                // so they take the plain ones.
                value @ 90..=97 => self.fg = (value - 90) as u8,
                value @ 100..=107 => self.bg = (value - 100) as u8,
                // 256-colour and direct colour: skip their arguments.
                38 | 48 => return,
                _ => {}
            }
        }
    }

    fn csi(&mut self, last: u8) {
        if self.private {
            // DEC private modes: cursor visibility, and this console's own
            // mode 9900, which lends the screen and the keyboard to a
            // program that draws through REX3 itself.
            match (self.param(0, 0), last) {
                (25, b'h') => self.cursor_on = true,
                (25, b'l') => self.cursor_on = false,
                (9900, b'h') => {
                    LENT.store(true, Ordering::Relaxed);
                    super::kbd::set_raw(true);
                }
                (9900, b'l') => self.take_back(),
                _ => {}
            }
            return;
        }
        let n = self.param(0, 1);
        match last {
            b'A' => self.row = self.row.saturating_sub(n),
            b'B' | b'e' => self.row = (self.row + n).min(self.rows - 1),
            b'C' | b'a' => self.col = (self.col + n).min(self.cols - 1),
            b'D' => self.col = self.col.min(self.cols - 1).saturating_sub(n),
            b'E' => {
                self.row = (self.row + n).min(self.rows - 1);
                self.col = 0;
            }
            b'F' => {
                self.row = self.row.saturating_sub(n);
                self.col = 0;
            }
            b'G' | b'`' => self.col = (n - 1).min(self.cols - 1),
            b'd' => self.row = (n - 1).min(self.rows - 1),
            b'H' | b'f' => {
                self.row = (n - 1).min(self.rows - 1);
                self.col = (self.param(1, 1) - 1).min(self.cols - 1);
            }
            b'J' => match self.param(0, 0) {
                0 => {
                    self.clear_cols(self.row, self.col.min(self.cols - 1), self.cols);
                    self.clear_rows(self.row + 1, self.rows);
                }
                1 => {
                    self.clear_rows(0, self.row);
                    self.clear_cols(self.row, 0, self.col.min(self.cols - 1) + 1);
                }
                _ => self.clear_rows(0, self.rows),
            },
            b'K' => match self.param(0, 0) {
                0 => self.clear_cols(self.row, self.col.min(self.cols - 1), self.cols),
                1 => self.clear_cols(self.row, 0, self.col.min(self.cols - 1) + 1),
                _ => self.clear_cols(self.row, 0, self.cols),
            },
            b'L' => self.scroll_down(self.row, n),
            b'M' => self.scroll_up(self.row, n),
            b'S' => self.scroll_up(0, n),
            b'T' => self.scroll_down(0, n),
            b'P' => {
                let col = self.col.min(self.cols - 1);
                let n = n.min(self.cols - col);
                let blank = self.blank();
                self.cells[self.row].copy_within(col + n..self.cols, col);
                for cell in &mut self.cells[self.row][self.cols - n..] {
                    *cell = blank;
                }
                self.draw_row(self.row, col);
            }
            b'@' => {
                let col = self.col.min(self.cols - 1);
                let n = n.min(self.cols - col);
                let blank = self.blank();
                self.cells[self.row].copy_within(col..self.cols - n, col + n);
                for cell in &mut self.cells[self.row][col..col + n] {
                    *cell = blank;
                }
                self.draw_row(self.row, col);
            }
            b'X' => {
                let col = self.col.min(self.cols - 1);
                self.clear_cols(self.row, col, (col + n).min(self.cols));
            }
            b'm' => self.sgr(),
            b's' => self.saved = (self.col, self.row),
            b'u' => (self.col, self.row) = self.saved,
            b'n' => match self.param(0, 0) {
                5 => self.reply(b"\x1b[0n"),
                6 => {
                    self.reply(b"\x1b[");
                    self.reply_number(self.row + 1);
                    self.reply(b";");
                    self.reply_number(self.col.min(self.cols - 1) + 1);
                    self.reply(b"R");
                }
                _ => {}
            },
            _ => {}
        }
        if !matches!(last, b'm' | b'n' | b's') {
            self.wrap = false;
        }
    }

    /// Take the screen back from a program: console registers, the eight
    /// console colours, and everything redrawn from the cell buffer.
    fn take_back(&mut self) {
        super::kbd::set_raw(false);
        if !LENT.swap(false, Ordering::Relaxed) {
            return;
        }
        registers();
        for (index, colour) in CONSOLE_COLOURS.iter().enumerate() {
            cmap_set(index as u16, *colour);
        }
        fill(0, 0, 1279, 1023, DEFAULT_BG);
        self.cursor_drawn = false;
        for row in 0..self.rows {
            self.draw_row(row, 0);
        }
    }

    fn reset(&mut self) {
        self.fg = DEFAULT_FG;
        self.bg = DEFAULT_BG;
        self.reverse = false;
        self.cursor_on = true;
        self.col = 0;
        self.row = 0;
        self.wrap = false;
        self.clear_rows(0, self.rows);
    }

    fn ground(&mut self, byte: u8) {
        // UTF-8: code points up to 0xFF index the font; others show as `?`.
        if self.utf8_left > 0 {
            if byte & 0xc0 == 0x80 {
                self.utf8 = (self.utf8 << 6) | (byte & 0x3f) as u32;
                self.utf8_left -= 1;
                if self.utf8_left == 0 {
                    let ch = match self.utf8 {
                        code @ 0..=0xff => code as u8,
                        // Box drawing, as programs use it for rules and
                        // separators, and common punctuation.
                        0x2500 | 0x2501 | 0x2013 | 0x2014 => b'-',
                        0x2502 | 0x2503 => b'|',
                        0x2504..=0x257f => b'+',
                        0x2018 | 0x2019 => b'\'',
                        0x201c | 0x201d => b'"',
                        0x2026 => b'.',
                        // ion marks a missing final newline with U+23CE.
                        0x23ce => b' ',
                        _ => b'?',
                    };
                    self.put(ch);
                }
                return;
            }
            self.utf8_left = 0;
        }
        match byte {
            0x1b => self.state = State::Escape,
            b'\n' => {
                // The kernel log ends lines with LF alone, and a terminal's
                // output has CR LF: LF returns the carriage in both.
                self.col = 0;
                self.newline();
            }
            b'\r' => {
                self.col = 0;
                self.wrap = false;
            }
            0x08 => {
                self.col = self.col.min(self.cols - 1).saturating_sub(1);
                self.wrap = false;
            }
            b'\t' => {
                self.col = ((self.col / 8 + 1) * 8).min(self.cols - 1);
                self.wrap = false;
            }
            0x00..=0x1f | 0x7f => {}
            0xc0..=0xdf => {
                self.utf8 = (byte & 0x1f) as u32;
                self.utf8_left = 1;
            }
            0xe0..=0xef => {
                self.utf8 = (byte & 0x0f) as u32;
                self.utf8_left = 2;
            }
            0xf0..=0xf7 => {
                self.utf8 = (byte & 0x07) as u32;
                self.utf8_left = 3;
            }
            0x80..=0xbf | 0xf8..=0xff => self.put(b'?'),
            _ => self.put(byte),
        }
    }

    fn byte(&mut self, byte: u8) {
        match self.state {
            State::Ground => self.ground(byte),
            State::Escape => {
                self.state = State::Ground;
                match byte {
                    b'[' => {
                        self.params = [0; 8];
                        self.nparams = 0;
                        self.private = false;
                        self.state = State::Csi;
                    }
                    b']' | b'P' | b'^' | b'_' => self.state = State::String,
                    b'(' | b')' | b'#' | b'%' => self.state = State::Skip,
                    b'7' => self.saved = (self.col, self.row),
                    b'8' => {
                        (self.col, self.row) = self.saved;
                        self.wrap = false;
                    }
                    b'c' => self.reset(),
                    b'D' => self.newline(),
                    b'E' => {
                        self.col = 0;
                        self.newline();
                    }
                    b'M' => {
                        if self.row > 0 {
                            self.row -= 1;
                        } else {
                            self.scroll_down(0, 1);
                        }
                    }
                    _ => {}
                }
            }
            State::Csi => match byte {
                b'0'..=b'9' => {
                    if self.nparams == 0 {
                        self.nparams = 1;
                    }
                    if let Some(param) = self.params.get_mut(self.nparams - 1) {
                        *param = param.saturating_mul(10).saturating_add((byte - b'0') as u16);
                    }
                }
                b';' | b':' => {
                    if self.nparams == 0 {
                        self.nparams = 1;
                    }
                    if self.nparams < self.params.len() {
                        self.nparams += 1;
                    }
                }
                b'?' | b'>' | b'<' | b'=' => self.private = true,
                0x20..=0x2f => {}
                0x40..=0x7e => {
                    self.state = State::Ground;
                    self.csi(byte);
                }
                _ => self.state = State::Ground,
            },
            State::String => match byte {
                0x07 => self.state = State::Ground,
                0x1b => self.state = State::StringEscape,
                _ => {}
            },
            State::StringEscape => {
                self.state = if byte == b'\\' { State::Ground } else { State::String };
            }
            State::Skip => self.state = State::Ground,
        }
    }
}

fn dcb_wait() {
    for _ in 0..200_000 {
        if rd(STATUS) & STATUS_BACKBUSY == 0 {
            return;
        }
    }
}

fn vc2_set(register: u8, value: u16) {
    wr(
        DCBMODE,
        VC2_INDEX | DCB_WIDTH_3 | DCB_REGISTER_INCREMENT | VC2_PROTOCOL,
    );
    wr(DCBDATA0, ((register as u32) << 24) | ((value as u32) << 8));
}

fn vc2_get(register: u8) -> u16 {
    wr(
        DCBMODE,
        VC2_INDEX | DCB_WIDTH_1 | DCB_REGISTER_INCREMENT | VC2_PROTOCOL,
    );
    // One byte, in the low byte lane of the data register.
    unsafe { ((REX3 + DCBDATA0 + 3) as *mut u8).write_volatile(register) };
    wr(
        DCBMODE,
        VC2_IREG | DCB_WIDTH_2 | DCB_REGISTER_INCREMENT | VC2_PROTOCOL,
    );
    vc2_ram_word()
}

/// The next 16-bit word of whatever DCBMODE selects, from the low half of
/// the data register.
fn vc2_ram_word() -> u16 {
    dcb_wait();
    unsafe { ((REX3 + DCBDATA0 + 2) as *const u16).read_volatile() }
}

/// The visible size in pixels, from the video timing table the PROM loaded
/// into the VC2. The frame table at the video entry pointer is pairs of
/// (line table address, repeat count), ended by a zero count; a line table
/// is a run of entries whose bit 0 clear marks displayed pixels, with the
/// pixel count in bits 8..14 (times two) and bit 15 on the last entry. This
/// is the method of Linux's Newport console (`newport_get_screensize`),
/// reimplemented from its description of the table format.
///
/// Falls back to 1024 by 768, which every Indy monitor mode can show, when
/// the result is not a plausible mode.
fn screen_size() -> (usize, usize) {
    let mut frame = [0u16; 128];
    let entry = vc2_get(VC2_VIDEO_ENTRY);
    vc2_set(VC2_RAM_ADDRESS, entry);
    wr(DCBMODE, VC2_RAM | DCB_WIDTH_2 | VC2_PROTOCOL);
    for word in frame.iter_mut() {
        *word = vc2_ram_word();
    }
    let mut width = 0;
    let mut height = 0;
    let mut index = 0;
    while index + 1 < frame.len() && frame[index + 1] != 0 {
        let mut pixels = 0usize;
        vc2_set(VC2_RAM_ADDRESS, frame[index]);
        wr(DCBMODE, VC2_RAM | DCB_WIDTH_2 | VC2_PROTOCOL);
        // Bounded: a table that never ends must not hang the boot.
        for _ in 0..256 {
            let mut entry = vc2_ram_word();
            if entry & 1 == 0 {
                pixels += ((entry >> 7) & 0xfe) as usize;
            }
            if entry & 0x80 == 0 {
                entry = vc2_ram_word();
            }
            if entry & 0x8000 != 0 {
                break;
            }
        }
        if pixels != 0 {
            width = width.max(pixels);
            height += frame[index + 1] as usize;
        }
        index += 2;
    }
    *RAW.lock() = (entry, [frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]], width, height);
    // The pixel count runs a couple of pixels over the nominal width
    // (1282 for 1280).
    if (640..=1300).contains(&width) && (480..=1024).contains(&height) {
        (width.min(1280), height)
    } else {
        (1024, 768)
    }
}

/// What `screen_size` read, for the boot log: the video entry pointer, the
/// first frame table words, and the size they add up to.
pub static RAW: Mutex<(u16, [u16; 6], usize, usize)> = Mutex::new((0, [0; 6], 0, 0));

static SIZE: Mutex<(usize, usize)> = Mutex::new((0, 0));

/// The screen size in pixels that `init` found.
pub fn size() -> (usize, usize) {
    *SIZE.lock()
}

/// Load one colour map entry.
fn cmap_set(index: u16, [red, green, blue]: [u8; 3]) {
    dcb_wait();
    wr(
        DCBMODE,
        CMAP_ALL | CMAP_PROTOCOL | DCB_SWAP | DCB_REGISTER_INCREMENT | CMAP_ADDRESS | DCB_WIDTH_2,
    );
    unsafe { ((REX3 + DCBDATA0 + 2) as *mut u16).write_volatile(index) };
    wr(DCBMODE, CMAP_ALL | CMAP_PROTOCOL | CMAP_PALETTE | DCB_WIDTH_3);
    wr(
        DCBDATA0,
        ((red as u32) << 24) | ((green as u32) << 16) | ((blue as u32) << 8),
    );
}

/// The drawing registers the console relies on.
fn registers() {
    wait();
    // Screen coordinates start at (0, 0) at the top left.
    wr(XYWIN, (4096 << 16) | 4096);
    wr(TOPSCAN, 0x3ff);
    // No screen masks; the window ID comparison stays as the PROM set it.
    wr(CLIPMODE, rd(CLIPMODE) & !0x1f);
    wr(WRMASK, 0xffff_ffff);
    wr(DRAWMODE1, CONSOLE_DRAWMODE1.load(Ordering::Relaxed));
}

/// Give the screen back to the console, from the keyboard driver's
/// Control-Alt-Backspace. Returns false when the console is busy; the
/// caller tries again on the next key.
pub fn force_text() -> bool {
    let Some(mut console) = CONSOLE.try_lock() else {
        return false;
    };
    console.take_back();
    console.show_cursor();
    true
}

/// Take over the screen. Call when the PROM's console is the graphics board.
pub fn init() {
    wait();
    let (width, height) = screen_size();
    *SIZE.lock() = (width, height);
    wait();
    let prom = rd(DRAWMODE1) & DM1_KEEP;
    let planes = if prom & 7 == 0 { DM1_CI8 } else { prom };
    CONSOLE_DRAWMODE1.store(planes | DM1_COMPARE_OFF | DM1_LOGIC_SRC, Ordering::Relaxed);
    registers();
    let mut console = CONSOLE.lock();
    console.cols = (width / CELL_W).min(MAX_COLS);
    console.rows = (height / CELL_H).min(MAX_ROWS);
    // Clear the whole frame, also the part past the last text row.
    fill(0, 0, 1279, 1023, DEFAULT_BG);
    let rows = console.rows;
    console.clear_rows(0, rows);
    console.show_cursor();
    drop(console);
    ACTIVE.store(true, Ordering::Relaxed);
}

pub fn active() -> bool {
    ACTIVE.load(Ordering::Relaxed)
}

/// Write console output. Skipped when the console is already being written
/// (a fault inside the console must not deadlock its own report).
pub fn write(buf: &[u8]) {
    if !active() {
        return;
    }
    let Some(mut console) = CONSOLE.try_lock() else {
        return;
    };
    console.hide_cursor();
    for &byte in buf {
        console.byte(byte);
    }
    console.show_cursor();
}

/// Take the queued replies to terminal reports (cursor position, status).
pub fn take_reply(out: &mut [u8; 32]) -> usize {
    if !active() {
        return 0;
    }
    let Some(mut console) = CONSOLE.try_lock() else {
        return 0;
    };
    let len = console.reply_len;
    out[..len].copy_from_slice(&console.reply[..len]);
    console.reply_len = 0;
    len
}
