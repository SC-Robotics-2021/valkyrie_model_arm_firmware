pub struct AdcTuning {
    /// minimum input value, in milivolts
    minima: u16,
    /// maximum value, in milivolts
    maxima: u16
}

const SIGNAL_CEIL: u16 = 3300;

impl AdcTuning {
    const fn new(minima: u16, maxima: u16) -> Self{
        Self {minima, maxima}
    }

    pub fn rescale(&self, millivolts: u16) -> f32{
        // ensure we arn't converting out-of-bounds.
        let millivolts = if millivolts > self.maxima { self.maxima } else {millivolts};
        let millivolts = if millivolts < self.minima { self.minima } else {millivolts};

        let out_min = -1.0f32;
        let out_max = 1.0f32;
        return ((millivolts - self.minima) as f32) * ((out_max - out_min) as f32) / ((self.maxima - self.minima) as f32) + out_min;
    }
}

pub const LOWER_ARM_TUNING: AdcTuning = AdcTuning::new(1000,SIGNAL_CEIL);
pub const UPPER_ARM_TUNING: AdcTuning = AdcTuning::new(650, 1300);
pub const AZIMUTH_AXIS_TUNING: AdcTuning = AdcTuning::new(0, 740);