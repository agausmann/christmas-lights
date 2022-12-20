#![no_std]
#![no_main]
#![deny(unsafe_op_in_unsafe_fn)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_rp::{gpio, spi};
use embassy_time::{Duration, Ticker};
use futures_util::StreamExt;
use ws2812_spi::Ws2812;

const NUM_LEDS: usize = 200;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 3_800_000;
    spi_config.polarity = spi::Polarity::IdleLow;
    /*
    let mut ws = Ws2812::new(spi::Spi::new_blocking(
        p.SPI1, p.PIN_14, // Throwaway pin for clk (FIXME address this in embassy)
        p.PIN_15, p.PIN_16, // Throwaway pin for miso (FIXME address this in ws2812-spi)
        spi_config,
    ));
    */
    let mut ticker = Ticker::every(Duration::from_millis(33));

    let mut indicator = gpio::Output::new(p.PIN_25, gpio::Level::High);

    loop {
        for i in 0..NUM_LEDS {
            ticker.next().await;
            indicator.toggle();
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
