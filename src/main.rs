#![no_std]
#![no_main]
#![deny(unsafe_op_in_unsafe_fn)]

use core::panic::PanicInfo;
use cortex_m::asm::wfi;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    clocks, gpio::Pins, pac::Peripherals, pio::PIOExt, sio::Sio, timer::Timer, watchdog::Watchdog,
    Clock,
};
use smart_leds::SmartLedsWrite;
use ws2812_pio::Ws2812;

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
    let cp = cortex_m::Peripherals::take().unwrap();

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
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.freq().to_Hz());

    let mut indicator = pins.gpio25.into_push_pull_output();
    indicator.set_high().ok();

    loop {
        for i in 0..200 {
            ws.write((0..200).map(|j| {
                if j == i || j == i + 1 {
                    [255, 255, 255]
                } else {
                    [0, 0, 0]
                }
            }))
            .ok();
            delay.delay_ms(100);
        }
    }
}
