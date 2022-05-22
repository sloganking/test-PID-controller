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
            integration_saturation: f32::MAX,

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
            self.integration_stored += error * dt;

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

    fn update_angle(&mut self, dt: f32, current_angle: f32, target_angle: f32) -> f32 {
        let error = angle_difference(target_angle, current_angle);

        //> calculate p term
            let p = self.proportianal_gain * error;

        //<> calculate D term
            // let error_rate_of_change = (error - self.error_last) / dt;
            // self.error_last = error;

            // let value_rate_of_change = (current_value - self.value_last) / dt;
            // self.value_last = current_value;

            let error_rate_of_change = angle_difference(error, self.error_last);
            self.error_last = error;

            let value_rate_of_change = angle_difference(current_angle, self.value_last);
            self.value_last = current_angle;

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
            self.integration_stored += error * dt;

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
}

/// Gives distance bewteen two angles (in degrees) with an output range of [-180, 180]
fn angle_difference(a: f32, b: f32) -> f32 {
    (a - b + 540.0) % 360.0 - 180.0
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

    fn display_angular_position(&self, target: f32) {
        for x in 0..=PIXELS {
            // let distance = (x as f32 - self.pos).abs();

            //> convert from angular to linear
                let transformed_pos = self.pos / 360.0 * PIXELS as f32;
                let target = target / 360.0 * PIXELS as f32;
            //<

            let max_pixel_disance = 1.0;
            let mut pixel = [0, 0, 0];

            //>  red target
                let brightness = brightness_from_distance_to(x as f32, target, max_pixel_disance);
                if brightness > 0 {
                    pixel = [brightness, 0, 0];
                }

            //<> white box
                if pixel == [0, 0, 0] {
                    let brightness =
                        brightness_from_distance_to(x as f32, transformed_pos, max_pixel_disance);
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
            self.speed = 0.0;
        } else if self.pos > PIXELS as f32 {
            self.pos = PIXELS as f32;
            self.speed = 0.0;
        }
    }

    fn update_angle(&mut self) {
        self.pos += self.speed;

        self.pos %= 360.0;

        if self.pos < 0.0 {
            self.pos = 360.0 - self.pos;
        }

        // if self.pos < 0.0 {
        //     self.pos = 0.0;
        // } else if self.pos > PIXELS as f32 {
        //     self.pos = PIXELS as f32;
        // }
    }
}

struct ControlledBox2d {
    pub x_pos: f32,
    x_speed: f32,
    pub y_pos: f32,
    y_speed: f32,
    x_max: u32,
    y_max: u32,
}

impl ControlledBox2d {
    fn new(x_max: u32, y_max: u32) -> Self {
        ControlledBox2d {
            x_pos: x_max as f32 / 2.0,
            x_speed: 0.0,
            y_pos: 0.0,
            y_speed: 0.0,
            x_max,
            y_max,
        }
    }

    fn update(&mut self) {

        // update position
        self.x_pos += self.x_speed;
        self.y_pos += self.y_speed;

        // keep x_pos in bounds
        if self.x_pos < 0.0 {
            self.x_pos = 0.0;
            self.x_speed = 0.0;
        } else if self.x_pos > self.x_max as f32 {
            self.x_pos = self.x_max as f32;
            self.x_speed = 0.0;
        }

        // keep y_pos in bounds
        if self.y_pos < 0.0 {
            self.y_pos = 0.0;
            self.y_speed = 0.0;
        } else if self.y_pos > self.y_max as f32 {
            self.y_pos = self.y_max as f32;
            self.y_speed = 0.0;
        }
    }

    fn add_force(&mut self, x_force: f32, y_force: f32){
        self.x_speed += x_force;
        self.y_speed += y_force;
    }

    fn display_scene(&self, target: (f32, f32)){

        let max_pixel_distance = 1.0;

        println!();
        for y in (0..=self.y_max).rev(){
            for x in 0..=self.x_max{

                // set default color to black
                let mut pixel = [20, 20, 20];

                // render red target
                    let distance = distance_2d(x as f32,y as f32, target.0, target.1);

                    if distance <= max_pixel_distance{
                        let brighness = ((max_pixel_distance - (distance / max_pixel_distance)) * 255.0) as u8; 
                        pixel[0] = brighness;
                    }

                //> render white box
                    let distance = distance_2d(x as f32,y as f32, self.x_pos, self.y_pos);

                    if distance <= max_pixel_distance{
                        let brighness = ((max_pixel_distance - (distance / max_pixel_distance)) * 255.0) as u8; 
                        pixel = [brighness, brighness, brighness];
                    }
                //<

                // print pixel
                print!("{}", "█".truecolor(pixel[0], pixel[1], pixel[2]));

            }
            println!();
        }
    }
}

fn distance_2d(p1x: f32,p1y: f32, p2x: f32,p2y: f32) -> f32{

    let x_distance = (p1x - p2x).abs();
    let y_distance = (p1y - p2y).abs();

    (x_distance.powf(2.0) + y_distance.powf(2.0)).sqrt()

}

enum Sim {
    _Linear,
    _Angular,
    _2d
}

fn main() {
    // decind which program we're going to run
    let sim = Sim::_2d;

    match sim {
        Sim::_Linear => {

            //> setup box
                let mut cbox = ControlledBox::new();
                cbox.pos = 10.0;
            //<

            //> setup PIDController
                let mut pidc = PIDController::new();
                pidc.proportianal_gain = 0.001;
                pidc.derivative_gain = 1.0;
                // pidc.integral_gain = 0.0000005;
                pidc.integration_saturation = 20000.0;
                pidc.output_max = f32::MAX;
                pidc.output_min = f32::MIN;
            //<

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
        }
        Sim::_Angular => {

            //> setup box
                let mut cbox = ControlledBox::new();
                cbox.pos = 10.0;
            //<

            //> setup PIDController
                let mut pidc = PIDController::new();
                pidc.proportianal_gain = 0.01;
                pidc.derivative_gain = 0.5;
                // pidc.integral_gain = 0.0000005;
                pidc.integration_saturation = 20000.0;
                pidc.output_max = f32::MAX;
                pidc.output_min = f32::MIN;
            //<

            //> run sim
                let mut rng = thread_rng();
                let mut target: f32 = rng.gen_range(0.0..=360.0);
                // let mut target: f32 = 20.0;
                let mut count = 0;
                let timestep = 20;

                loop {
                    if count > PIXELS {
                        count = 0;
                        target = rng.gen_range(0.0..=360.0);
                        // target = 340.0;
                    }

                    // println!("cbox.pos: {}", cbox.pos);
                    cbox.display_angular_position(target);
                    let input = pidc.update_angle(timestep as f32, cbox.pos, target);
                    cbox.add_force(input);
                    cbox.update_angle();

                    //> sleep
                        let ten_millis = time::Duration::from_millis(timestep);
                        thread::sleep(ten_millis);
                    //<
                    println!();
                    count += 1;
                }
            //<
        }
        Sim::_2d => {

            let mut cbox2d = ControlledBox2d::new(40,20);

            cbox2d.x_pos = 20.0;

            //> setup PIDController for x axis
                let mut pid_x = PIDController::new();
                pid_x.proportianal_gain = 0.01;
                pid_x.derivative_gain = 10.0;
                // pid_x.integral_gain = 0.0000005;
                // pid_x.integration_saturation = 20000.0;
                pid_x.output_max = f32::MAX;
                pid_x.output_min = f32::MIN;

            //<> setup PIDController for y axis
                let mut pid_y = PIDController::new();
                pid_y.proportianal_gain = 0.01;
                pid_y.derivative_gain = 10.0;
                pid_y.integral_gain = 0.0000025;
                pid_y.integration_saturation = 15000.0;
                pid_y.output_max = f32::MAX;
                pid_y.output_min = f32::MIN;
            //<

            //=====================================

            let mut rng = thread_rng();
            let mut target = (rng.gen_range(0.0..=cbox2d.x_max as f32), rng.gen_range(0.0..=cbox2d.y_max as f32));
            let mut target = (cbox2d.x_max as f32 / 2.0, cbox2d.y_max as f32 / 2.0);
            let mut count = 0;
            let timestep = 100;

            loop {
                if count > 50 {
                    count = 0;
                    target = (rng.gen_range(0.0..=cbox2d.x_max as f32), rng.gen_range(0.0..=cbox2d.y_max as f32));
                }

                //> Display scene
                    cbox2d.display_scene(target);
                //<> calculate and apply PID inputs
                    let x_force = pid_x.update(timestep as f32, cbox2d.x_pos, target.0);
                    let y_force = pid_y.update(timestep as f32, cbox2d.y_pos, target.1);
                    cbox2d.add_force(x_force, y_force - 0.03);

                //<> update scene physics
                    cbox2d.update();

                //> sleep
                    let ten_millis = time::Duration::from_millis(timestep);
                    thread::sleep(ten_millis);
                //<
                // println!();
                count += 1;
            }

        }
    }
}
