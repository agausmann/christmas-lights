#![no_std]
#![no_main]
#![deny(unsafe_op_in_unsafe_fn)]

use core::panic::PanicInfo;
use cortex_m::asm::wfi;
use embedded_hal::{
    digital::v2::{OutputPin, ToggleableOutputPin},
    timer::CountDown,
};
use fugit::RateExtU64;
use micromath::F32Ext;
use nanorand::{Rng, WyRand};
use once_cell::sync::Lazy;
use rp2040_hal::{
    clocks, gpio::Pins, pac::Peripherals, pio::PIOExt, sio::Sio, timer::Timer, watchdog::Watchdog,
    Clock,
};
use smart_leds::SmartLedsWrite;
use ws2812_pio::Ws2812;

const TREE_LEDS: usize = 200;
const WINDOW_ROWS: usize = 25;
const WINDOW_COLS: usize = 4;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let _ = info;

    cortex_m::interrupt::disable();
    loop {
        wfi();
    }
}

static GAMMA: Lazy<[u8; 256]> = Lazy::new(|| {
    let mut gamma = [0u8; 256];
    for i in 0..256 {
        gamma[i as usize] = ((i as f32) * (i as f32) / 255.0).ceil() as u8;
    }
    gamma
});

// NOTE GRB
const PALETTE: [[u8; 3]; 3] = [
    // Red
    [0, 1, 0],
    // White
    [1, 1, 1],
    // Green
    [1, 0, 0],
];

struct Shimmer {
    period: u32,
    phase: u32,
    color: usize,
}

struct Snowflake {
    column: usize,
    phase: u32,
    period: u32,
}

impl Snowflake {
    fn generate<R, const OUTPUT: usize>(rng: &mut R) -> Self
    where
        R: Rng<OUTPUT>,
    {
        Self {
            column: rng.generate_range(0..WINDOW_COLS),
            phase: 0,
            period: rng.generate_range(200..720),
        }
    }

    fn tick<R, const OUTPUT: usize>(&mut self, rng: &mut R)
    where
        R: Rng<OUTPUT>,
    {
        self.phase = (self.phase + 1) % self.period;
        if self.phase == 0 {
            *self = Self::generate(rng);
        }
    }

    fn row(&self) -> usize {
        ((self.phase as usize) * WINDOW_ROWS) / (self.period as usize)
    }
}

struct Snowflakes {
    leds: [[bool; WINDOW_ROWS]; WINDOW_COLS],
    snowflakes: [Snowflake; 20],
}

impl Snowflakes {
    fn generate<R, const OUTPUT: usize>(rng: &mut R) -> Self
    where
        R: Rng<OUTPUT>,
    {
        Self {
            leds: [[false; WINDOW_ROWS]; WINDOW_COLS],
            snowflakes: core::array::from_fn(|_| Snowflake::generate(rng)),
        }
    }

    fn tick<R, const OUTPUT: usize>(&mut self, rng: &mut R)
    where
        R: Rng<OUTPUT>,
    {
        // Clear led buffer
        self.leds = [[false; WINDOW_ROWS]; WINDOW_COLS];

        // Update snowflakes and set leds
        for flake in &mut self.snowflakes {
            flake.tick(rng);
            self.leds[flake.column][flake.row()] = true;
        }
    }

    fn color(&self, row: usize, col: usize, max_brightness: u32) -> [u8; 3] {
        if self.leds[col][row] {
            [max_brightness as u8; 3]
        } else {
            [0; 3]
        }
    }
}

impl Shimmer {
    fn generate<R, const OUTPUT: usize>(rng: &mut R) -> Self
    where
        R: Rng<OUTPUT>,
    {
        Self {
            period: rng.generate_range(200..480),
            phase: 0,
            color: rng.generate_range(0..PALETTE.len()),
        }
    }

    fn tick<R, const OUTPUT: usize>(&mut self, rng: &mut R)
    where
        R: Rng<OUTPUT>,
    {
        self.phase = (self.phase + 1) % self.period;
        if self.phase == 0 {
            self.color = rng.generate_range(0..PALETTE.len());
        }
    }

    fn color(&self, max_brightness: u32) -> [u8; 3] {
        let midpoint = self.period / 2;
        let x = if self.phase < midpoint {
            self.phase
        } else {
            self.period - self.phase
        };
        let brightness = x * max_brightness / midpoint;
        let output = GAMMA[brightness as usize];
        let color = &PALETTE[self.color];

        [color[0] * output, color[1] * output, color[2] * output]
    }
}

#[rp2040_hal::entry]
fn main() -> ! {
    let mut dp = Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(dp.WATCHDOG);

    let clocks = clocks::init_clocks_and_plls(
        12_000_000,
        dp.XOSC,
        dp.CLOCKS,
        dp.PLL_SYS,
        dp.PLL_USB,
        &mut dp.RESETS,
        &mut watchdog,
    )
    .map_err(|_| "failed to init clocks")
    .unwrap();

    let sio = Sio::new(dp.SIO);
    let pins = Pins::new(dp.IO_BANK0, dp.PADS_BANK0, sio.gpio_bank0, &mut dp.RESETS);

    let timer = Timer::new(dp.TIMER, &mut dp.RESETS);
    let (mut pio, sm0, sm1, _, _) = dp.PIO0.split(&mut dp.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio16.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut ws2 = Ws2812::new(
        pins.gpio17.into_mode(),
        &mut pio,
        sm1,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut rng = WyRand::new_seed(0x60511865);
    let mut tree_leds: [Shimmer; TREE_LEDS] = core::array::from_fn(|_| Shimmer::generate(&mut rng));
    let mut flakes = Snowflakes::generate(&mut rng);

    let mut frame_timer = timer.count_down();
    frame_timer.start(60.Hz::<1, 1000000>().into_duration());

    let mut indicator = pins.gpio25.into_push_pull_output();
    indicator.set_high().ok();

    loop {
        nb::block!(frame_timer.wait()).unwrap();

        // Tree shimmer
        ws.write(tree_leds.iter().map(|led| led.color(127)))
            .unwrap();
        for led in &mut tree_leds {
            led.tick(&mut rng);
        }

        // Snowflakes
        // Note - column-major order; zigzag (every other column is reversed)
        ws2.write((0..WINDOW_COLS).flat_map(|col| {
            // NOTE - needed to prevent `move` from trying to move flakes
            let flakes = &flakes;
            (0..WINDOW_ROWS).map(move |row| {
                let row = if col % 2 == 0 {
                    WINDOW_ROWS - 1 - row
                } else {
                    row
                };
                flakes.color(row, col, 63)
            })
        }))
        .unwrap();
        flakes.tick(&mut rng);

        indicator.toggle().unwrap();
    }
}
