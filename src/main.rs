use core::time;
use std::thread::{self};

use colored::Colorize;
use rand::{thread_rng, Rng};

static PIXELS: i32 = 200;

#[derive(PartialEq)]
pub enum DerivativeMeasurement {
    Velocity,
    ErrorRateOfChange,
}

pub struct PIDController {
    pub proportianal_gain: f32,
    pub integral_gain: f32,
    pub derivative_gain: f32,

    pub integration_stored: f32,
    pub integration_saturation: f32,

    pub error_last: f32,
    pub value_last: f32,

    /// use DerivativeMeasurement::Velocity to avoid derivative kick
    /// use DerivativeMeasurement::ErrorRateOfChange to keep derivative kick
    pub derivative_measurement: DerivativeMeasurement,
    pub derivative_initialized: bool,

    pub output_min: f32,
    pub output_max: f32,
}

impl PIDController {
    fn new() -> Self {
        PIDController {
            proportianal_gain: 0.0,
            integral_gain: 0.0,
            derivative_gain: 0.0,
            integration_stored: 0.0,
            integration_saturation: 0.0,

            error_last: 0.0,
            value_last: 0.0,

            derivative_measurement: DerivativeMeasurement::Velocity,
            derivative_initialized: false,

            output_min: 0.0,
            output_max: 0.0,
        }
    }

    fn update(&mut self, dt: f32, current_value: f32, target_value: f32) -> f32 {
        let error = target_value - current_value;

        //> calculate p term
            let p = self.proportianal_gain * error;

        //<> calculate D term
            let error_rate_of_change = (error - self.error_last) / dt;
            self.error_last = error;

            let value_rate_of_change = (current_value - self.value_last) / dt;
            self.value_last = current_value;

            let mut d = 0.0;
            // skip calculating dterm if first itertion
            if self.derivative_initialized {
                // choose d term to use
                let derivative_measure =
                    if self.derivative_measurement == DerivativeMeasurement::Velocity {
                        -value_rate_of_change
                    } else {
                        error_rate_of_change
                    };

                d = self.derivative_gain * derivative_measure;
            } else {
                self.derivative_initialized = true;
            }

        //<> calculate I term
            self.integration_stored = self.integration_stored + (error * dt);

            // limit integration_stored
            if self.integration_stored > self.integration_saturation {
                self.integration_stored = self.integration_saturation;
            } else if self.integration_stored < -self.integration_saturation {
                self.integration_stored = -self.integration_saturation;
            }

            let i = self.integral_gain * self.integration_stored;
        //<

        let mut result = p + i + d;

        if result > self.output_max {
            result = self.output_max;
        } else if result < self.output_min {
            result = self.output_min;
        }

        result
    }

    /// Should be called if the system has been moved by external means, such as teleportation,
    /// or if the PID controller has been turned off for a long period of time.
    pub fn reset(&mut self) {
        self.derivative_initialized = false;
    }
}

struct ControlledBox {
    pub pos: f32,
    speed: f32,
}

fn brightness_from_distance_to(cur: f32, target: f32, max_pixel_disance: f32) -> u8 {
    let distance = (cur as f32 - target).abs();

    if distance > max_pixel_disance {
        0
    } else {
        ((max_pixel_disance - (distance / max_pixel_disance)) * 255.0) as u8
    }
}

impl ControlledBox {
    fn new() -> Self {
        ControlledBox {
            pos: 0.0,
            speed: 0.0,
        }
    }

    fn display_position(&self, target: f32) {
        for x in 0..=PIXELS {
            // let distance = (x as f32 - self.pos).abs();
            let max_pixel_disance = 1.0;
            let mut pixel = [0, 0, 0];

            //>  red target
                let brightness = brightness_from_distance_to(x as f32, target, max_pixel_disance);
                if brightness > 0 {
                    pixel = [brightness, 0, 0];
                }

            //<> white box
                if pixel == [0, 0, 0] {
                    let brightness = brightness_from_distance_to(x as f32, self.pos, max_pixel_disance);
                    if brightness > 0 {
                        pixel = [brightness, brightness, brightness];
                    }
                }

            //<

            // print pixel
            print!("{}", "█".truecolor(pixel[0], pixel[1], pixel[2]));
        }
    }

    fn add_force(&mut self, force: f32) {
        let gravity = -0.0;
        self.speed += force + gravity;
    }

    fn update(&mut self) {
        self.pos += self.speed;

        if self.pos < 0.0 {
            self.pos = 0.0;
        } else if self.pos > PIXELS as f32 {
            self.pos = PIXELS as f32;
        }
    }
}

fn main() {
    //> setup box
        let mut cbox = ControlledBox::new();
        cbox.pos = 10.0;

    //<> setup PIDController
        let mut pidc = PIDController::new();
        pidc.proportianal_gain = 0.001;
        pidc.derivative_gain = 1.0;
        // pidc.integral_gain = 0.0000005;
        pidc.integration_saturation = 20000.0;
        pidc.output_max = f32::MAX;
        pidc.output_min = f32::MIN;

    //<> run sim
        let mut rng = thread_rng();
        let mut target: f32 = rng.gen_range(0.0..=PIXELS as f32);
        let mut count = 0;
        let timestep = 20;

        loop {
            if count > PIXELS {
                count = 0;
                target = rng.gen_range(0.0..=PIXELS as f32);
            }

            cbox.display_position(target);
            let input = pidc.update(timestep as f32, cbox.pos, target);
            cbox.add_force(input);
            cbox.update();

            //> sleep
                let ten_millis = time::Duration::from_millis(timestep);
                thread::sleep(ten_millis);
            //<
            println!();
            count += 1;
        }
    //<
}
