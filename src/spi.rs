//! Serial Peripheral Interface
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for clock signal (SCLK)
//! - Output GPIO pin for data transmission (Master Output Slave Input - MOSI)
//! - Input GPIO pin for data reception (Master Input Slave Output - MISO)
//!
//! The timer must be configured to twice the desired communication frequency.
//!
//! SS/CS (slave select) must be handled independently.
//!
//! MSB-first and LSB-first bit orders are supported.
//!

use core::cmp::max;

pub use embedded_hal_async::spi::{MODE_0, MODE_1, MODE_2, MODE_3};
use embedded_hal_async::{delay::DelayNs, spi::SpiBus};

use embedded_hal::{
    digital::{InputPin, OutputPin},
    spi::ErrorType,
};
use embedded_hal_async::spi::{Mode, Polarity};
use fugit::Duration;
use fugit::RateExtU32;

/// Error type
#[derive(Debug)]
pub enum Error<E> {
    /// Communication error
    Bus(E),
    /// Attempted read without input data
    NoData,
}

impl<E: core::fmt::Debug> embedded_hal::spi::Error for Error<E> {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            Error::Bus(_) => embedded_hal::spi::ErrorKind::Other,
            Error::NoData => embedded_hal::spi::ErrorKind::Other,
        }
    }
}

/// Transmission bit order
#[derive(Debug)]
pub enum BitOrder {
    /// Most significant bit first
    MSBFirst,
    /// Least significant bit first
    LSBFirst,
}

impl Default for BitOrder {
    /// Default bit order: MSB first
    fn default() -> Self {
        BitOrder::MSBFirst
    }
}

/// Configuration of the SPI Interface
pub struct SpiConfig {
    mode: Mode,
    bit_order: BitOrder,
    /// Value used when still receiving bits from MISO, but not value is available
    /// in buffer to be written to MOSI. This value is used instead. Usually `0x00``
    empty_write_value: u8,
    /// controls the clock speed. `f = 2 / half_period_duration_ns`
    half_period_duration_ns: u32,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: Default::default(),
            empty_write_value: 0x00,
            half_period_duration_ns: 10, // 100 kHz.
        }
    }
}

/// A Full-Duplex SPI implementation, takes 3 pins, and a timer running at 2x
/// the desired SPI frequency.
pub struct SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Delay: DelayNs,
{
    miso: Miso,
    mosi: Mosi,
    sck: Sck,
    delay: Delay,
    config: SpiConfig,
}

impl<Miso, Mosi, Sck, Delay, E> SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Delay: DelayNs,
{
    /// Create instance
    pub fn new(
        mode: Mode,
        miso: Miso,
        mosi: Mosi,
        sck: Sck,
        delay: Delay,
        spi_config: SpiConfig,
    ) -> Self {
        let mut spi = SPI {
            miso,
            mosi,
            sck,
            delay,
            config: spi_config,
        };

        match mode.polarity {
            Polarity::IdleLow => spi.sck.set_low(),
            Polarity::IdleHigh => spi.sck.set_high(),
        }
        .unwrap_or(());

        spi
    }

    /// Set transmission bit order
    pub fn with_bit_order(&mut self, order: BitOrder) {
        self.config.bit_order = order;
    }

    async fn read_bit(&mut self, read_val: &mut u8) -> Result<(), crate::spi::Error<E>> {
        let is_miso_high = self.miso.is_high().map_err(Error::Bus)?;
        let shifted_value = *read_val << 1;
        if is_miso_high {
            *read_val = shifted_value | 1;
        } else {
            *read_val = shifted_value;
        }
        Ok(())
    }

    #[inline]
    fn set_clk_high(&mut self) -> Result<(), crate::spi::Error<E>> {
        self.sck.set_high().map_err(Error::Bus)
    }

    #[inline]
    fn set_clk_low(&mut self) -> Result<(), crate::spi::Error<E>> {
        self.sck.set_low().map_err(Error::Bus)
    }

    #[inline]
    async fn wait_for_timer(&mut self) {
        self.delay
            .delay_ns(self.config.half_period_duration_ns)
            .await;
    }

    #[inline]
    async fn rw_byte(
        &mut self,
        clock_out: u8,
        read_in: &mut u8,
    ) -> Result<(), crate::spi::Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = match self.config.bit_order {
                BitOrder::MSBFirst => (clock_out >> (7 - bit_offset)) & 0b1,
                BitOrder::LSBFirst => (clock_out >> bit_offset) & 0b1,
            };

            if out_bit == 1 {
                self.mosi.set_high().map_err(Error::Bus)?;
            } else {
                self.mosi.set_low().map_err(Error::Bus)?;
            }

            match self.config.mode {
                MODE_0 => {
                    self.wait_for_timer().await;
                    self.set_clk_high()?;
                    self.read_bit(read_in).await?;
                    self.wait_for_timer().await;
                    self.set_clk_low()?;
                }
                MODE_1 => {
                    self.set_clk_high()?;
                    self.wait_for_timer().await;
                    self.read_bit(read_in).await?;
                    self.set_clk_low()?;
                    self.wait_for_timer().await;
                }
                MODE_2 => {
                    self.wait_for_timer().await;
                    self.set_clk_low()?;
                    self.read_bit(read_in).await?;
                    self.wait_for_timer().await;
                    self.set_clk_high()?;
                }
                MODE_3 => {
                    self.set_clk_low()?;
                    self.wait_for_timer().await;
                    self.read_bit(read_in).await?;
                    self.set_clk_high()?;
                    self.wait_for_timer().await;
                }
            };
        }
        Ok(())
    }
}

impl<Miso, Mosi, Sck, Timer, E> ErrorType for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: DelayNs,
    E: core::fmt::Debug,
{
    type Error = crate::spi::Error<E>;
}

impl<Miso, Mosi, Sck, Timer, E> SpiBus<u8> for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: DelayNs,
    E: core::fmt::Debug,
{
    #[inline]
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            self.rw_byte(self.config.empty_write_value, word).await?;
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_write = 0u8;
        for byte in words {
            self.rw_byte(*byte, &mut ignored_write).await?;
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_write = 0u8;
        for i in 0..max(read.len(), write.len()) {
            let read_in_byte = read.get_mut(i).unwrap_or(&mut ignored_write);
            let clock_out_byte = write
                .get(i)
                .copied()
                .unwrap_or(self.config.empty_write_value);
            self.rw_byte(clock_out_byte, read_in_byte).await?;
        }

        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut current_read_byte = 0u8;
        for clock_out_byte in words {
            self.rw_byte(*clock_out_byte, &mut current_read_byte)
                .await?;
            *clock_out_byte = current_read_byte;
        }

        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // we always flush. Nothing to do here.
        Ok(())
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State as PinState, Transaction as PinTransaction};
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_async::spi::MODE_0;
    use embedded_hal_mock::eh1::MockError;

    #[tokio::test]
    async fn test_spi_write_single_byte() {
        let miso = PinMock::new(&[]);
        let mosi = PinMock::new(&[
            PinTransaction::set(PinState::Low), // MOSI writes start here
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ]);
        let sck = PinMock::new(&[
            PinTransaction::set(PinState::Low), // SCK toggles start here
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ]);
        let delay = MockDelay::new();

        let mut spi = SPI::new(MODE_0, miso, mosi, sck, delay, SpiConfig::default());
        let data = [0xAA]; // 10101010 in binary
        spi.write(&data).await.expect("SPI write failed");

        // Verify that all transactions were completed
        spi.mosi.done();
        spi.sck.done();
    }

    #[tokio::test]
    async fn test_spi_read_single_byte() {
        let miso = PinMock::new(&[
            PinTransaction::get(PinState::High), // MISO returns 1
            PinTransaction::get(PinState::Low),  // MISO returns 0
            PinTransaction::get(PinState::High), // MISO returns 1
            PinTransaction::get(PinState::Low),  // MISO returns 0
            PinTransaction::get(PinState::High), // MISO returns 1
            PinTransaction::get(PinState::Low),  // MISO returns 0
            PinTransaction::get(PinState::High), // MISO returns 1
            PinTransaction::get(PinState::Low),  // MISO returns 0
        ]);
        let mosi = PinMock::new(&[]);
        let sck = PinMock::new(&[
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ]);
        let delay = MockDelay::new();

        let mut spi = SPI::new(MODE_0, miso, mosi, sck, delay, SpiConfig::default());
        let mut data = [0x00];
        spi.read(&mut data).await.expect("SPI read failed");

        assert_eq!(data[0], 0b10101010); // 0xAA in binary
    }

    #[tokio::test]
    async fn test_spi_transfer() {
        let miso = PinMock::new(&[
            PinTransaction::get(PinState::Low),
            PinTransaction::get(PinState::High),
            PinTransaction::get(PinState::Low),
            PinTransaction::get(PinState::High),
            PinTransaction::get(PinState::Low),
            PinTransaction::get(PinState::High),
            PinTransaction::get(PinState::Low),
            PinTransaction::get(PinState::High),
        ]);
        let mosi = PinMock::new(&[
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
        ]);
        let sck = PinMock::new(&[
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ]);
        let delay = MockDelay::new();

        let mut spi = SPI::new(MODE_0, miso, mosi, sck, delay, SpiConfig::default());
        let mut read_data = [0x00];
        let write_data = [0xAA];

        spi.transfer(&mut read_data, &write_data)
            .await
            .expect("SPI transfer failed");

        assert_eq!(read_data[0], 0b01010101); // Received bits in opposite phase
    }
}
