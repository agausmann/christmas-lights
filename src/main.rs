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
use rp2040_hal::{
    clocks, gpio::Pins, pac::Peripherals, pio::PIOExt, sio::Sio, timer::Timer, watchdog::Watchdog,
    Clock,
};
use smart_leds::SmartLedsWrite;
use ws2812_pio::Ws2812;

const NUM_LEDS: usize = 200;

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
    let (mut pio, sm0, _, _, _) = dp.PIO0.split(&mut dp.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio16.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut gamma = [0u8; 256];
    for i in 0..256 {
        gamma[i as usize] = ((i as f32) * (i as f32) / 255.0).ceil() as u8;
    }

    let mut rng = WyRand::new_seed(0x60511865);

    let max_brightness: u32 = 127;
    // NOTE GRB
    let palette = [
        // Red
        [0, 1, 0],
        // White
        [1, 1, 1],
        // Green
        [1, 0, 0],
    ];
    let mut led_periods = [0u32; NUM_LEDS];
    for period in &mut led_periods {
        *period = rng.generate_range(200..480);
    }
    let mut led_colors = [0u8; NUM_LEDS];
    for color in &mut led_colors {
        *color = rng.generate_range(0..palette.len() as u8);
    }

    let mut led_phases = [0u32; NUM_LEDS];

    let mut frame_timer = timer.count_down();
    frame_timer.start(60.Hz::<1, 1000000>().into_duration());

    let mut indicator = pins.gpio25.into_push_pull_output();
    indicator.set_high().ok();

    loop {
        nb::block!(frame_timer.wait()).unwrap();
        ws.write(led_phases.iter().zip(&led_colors).zip(&led_periods).map(
            |((&phase, &color), &period)| {
                let midpoint = period / 2;
                let x = if phase < midpoint {
                    phase
                } else {
                    period - phase
                };
                let brightness = x * max_brightness / midpoint;
                let output = gamma[brightness as usize];
                let color = &palette[color as usize];

                //NOTE GRB
                [color[0] * output, color[1] * output, color[2] * output]
            },
        ))
        .unwrap();
        for ((phase, color), period) in led_phases.iter_mut().zip(&mut led_colors).zip(&led_periods)
        {
            *phase = (*phase + 1) % *period;
            if *phase == 0 {
                *color = rng.generate_range(0..palette.len() as u8);
            }
        }
        indicator.toggle().unwrap();
    }
}
