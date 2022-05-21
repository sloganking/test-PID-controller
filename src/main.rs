use core::time;
use std::thread::{self, current};

use colored::Colorize;

pub struct PIDController {
    pub proportianalGain: f32,
    pub integralGain: f32,
    pub derivativeGain: f32,
}

impl PIDController {
    fn new() -> Self {
        PIDController {
            proportianalGain: 0.0,
            integralGain: 0.0,
            derivativeGain: 0.0,
        }
    }
    fn update(&self, dt: f32, current_value: f32, target_value: f32) -> f32 {
        let error = target_value - current_value;

        let p = self.proportianalGain * error;

        p
    }
}

struct ControlledBox {
    pub pos: f32,
    speed: f32,
}

impl ControlledBox {
    fn new() -> Self {
        ControlledBox {
            pos: 0.0,
            speed: 0.0,
        }
    }

    fn display_position(&self) {
        //> gradiantless
            // for x in 0..100{
            //     if (x as f32 - self.pos).abs() <= 0.5{
            //         print!("{}", "█".truecolor(255, 255, 255));
            //     } else {
            //         print!("{}", "█".truecolor(0, 0, 0));
            //     }
            // }

        //<> color gradiant
            for x in 0..100 {
                let distance = (x as f32 - self.pos).abs();
                let max_pixel_disance = 1.0;

                let brightness = if distance > max_pixel_disance {
                    0
                } else {
                    ((max_pixel_disance - (distance / max_pixel_disance)) * 255.0) as u8
                };

                print!("{}", "█".truecolor(brightness, brightness, brightness));
            }
        //<

        // println!();
    }

    fn add_force(&mut self, force: f32) {
        self.speed += force;
    }

    fn update(&mut self) {
        self.pos += self.speed;

        if self.pos < 0.0 {
            self.pos = 0.0;
        } else if self.pos > 100.0 {
            self.pos = 100.0;
        }
    }
}

fn main() {
    //>  sample display
        // for x in 0..10000{
        //     let x = x as f32 / 100.0;

        //     ControlledBox.pos = x;
        //     ControlledBox.display_position();

        //     //<> sleep
        //         let ten_millis = time::Duration::from_millis(5);
        //         thread::sleep(ten_millis);
        //     //<
        //     print!("\r")
        // }
    //<

    // setup box
    let mut cbox = ControlledBox::new();
    cbox.pos = 10.0;

    // setup PIDController
    let mut pidc = PIDController::new();
    pidc.proportianalGain = 0.001;

    // run sim
    loop {
        let timestep = 10;
        cbox.display_position();
        let input = pidc.update(timestep as f32, cbox.pos, 50.0);
        cbox.add_force(input);
        cbox.update();

        //> sleep
            let ten_millis = time::Duration::from_millis(timestep);
            thread::sleep(ten_millis);
        //<
        print!("\r");
    }
}
