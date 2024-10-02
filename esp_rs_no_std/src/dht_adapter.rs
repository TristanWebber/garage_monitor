use esp_hal::{
    clock::Clocks,
    delay::Delay,
    gpio::{InputPin, Level, OutputOpenDrain, OutputPin, Pull},
    peripheral::Peripheral,
};

use embedded_hal::digital::v2::{InputPin as DhtInputPin, OutputPin as DhtOutputPin};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

// sensor pin needs to implement traits embedded_hal::digital::v2::InputPin and embedded_hal::digital::v2::OutputPin
pub struct SensorAdapter<'a, P> {
    inner: OutputOpenDrain<'a, P>,
}

impl<'a, P> SensorAdapter<'a, P>
where
    P: InputPin + OutputPin,
{
    pub fn new(pin: impl Peripheral<P = P> + 'a) -> Self {
        SensorAdapter {
            inner: OutputOpenDrain::new(pin, Level::High, Pull::None),
        }
    }
}

impl<'a, P> DhtInputPin for SensorAdapter<'a, P>
where
    P: InputPin + OutputPin,
{
    type Error = core::convert::Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.inner.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.inner.is_low())
    }
}

impl<'a, P> DhtOutputPin for SensorAdapter<'a, P>
where
    P: InputPin + OutputPin,
{
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.inner.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.inner.set_low())
    }
}

// Delay needs to implement traits DelayMs and DelayUs
pub struct DelayAdapter {
    inner: Delay,
}

impl DelayAdapter {
    pub fn new(clocks: &Clocks) -> Self {
        DelayAdapter {
            inner: Delay::new(clocks),
        }
    }
}

impl DelayMs<u8> for DelayAdapter
{
    fn delay_ms(&mut self, ms: u8) {
        self.inner.delay_millis(ms.into())
    }
}

impl DelayUs<u8> for DelayAdapter
{
    fn delay_us(&mut self, us: u8) {
        self.inner.delay_micros(us.into())
    }
}

