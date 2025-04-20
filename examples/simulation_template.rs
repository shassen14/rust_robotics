use plotters_bitmap::BitMapBackend;
use rust_robotics::utils::defs::BufferWrapper;
use std::borrow::{Borrow, BorrowMut};

// simulation.rs

pub struct Simulation {
    pub state: Vec<f64>,
    pub dt: f64,
}

impl Simulation {
    pub fn new(state: Vec<f64>, dt: f64) -> Self {
        Simulation { state, dt }
    }

    pub fn update(&mut self) {
        // update the state of the simulation
        self.state.push(self.state[self.state.len() - 1] + self.dt);
    }
}

// renderer.rs
use minifb::{Window, WindowOptions};
use plotters::{backend::BGRXPixel, prelude::*};

pub struct Renderer {
    width: usize,
    height: usize,
    buffer: BufferWrapper,
    window: Window,
}

impl Renderer {
    pub fn new(width: usize, height: usize) -> Self {
        let buffer = BufferWrapper(vec![0u32; width * height]);
        let window = Window::new("Simulation", width, height, WindowOptions::default()).unwrap();
        Renderer {
            width,
            height,
            buffer,
            window,
        }
    }

    pub fn render(&mut self, simulation: &Simulation) -> Result<(), Box<dyn std::error::Error>> {
        {
            let backend = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
                self.buffer.borrow_mut(),
                (self.width as u32, self.height as u32),
            )?;
            let root = backend.into_drawing_area();
            root.fill(&WHITE)?;

            let max_time = simulation.state.len() as f64 * simulation.dt;
            let max_val = simulation
                .state
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);

            let mut chart = ChartBuilder::on(&root)
                .margin(10)
                .caption("Simulation", ("sans-serif", 30))
                .x_label_area_size(30)
                .y_label_area_size(30)
                .build_cartesian_2d(0.0..max_time, 0.0..max_val)?;

            chart.configure_mesh().draw()?;

            chart.draw_series(LineSeries::new(
                simulation
                    .state
                    .iter()
                    .enumerate()
                    .map(|(i, &val)| (i as f64 * simulation.dt, val)),
                &RED,
            ))?;
        }

        self.window
            .update_with_buffer(self.buffer.borrow(), self.width, self.height)?;

        Ok(())
    }
    pub fn is_open(&self) -> bool {
        self.window.is_open() && !self.window.is_key_down(minifb::Key::Escape)
    }
}

// main.rs
fn main() {
    let mut simulation = Simulation::new(vec![0.0], 0.01);
    let mut renderer = Renderer::new(800, 600);

    while renderer.is_open() {
        simulation.update();
        if let Err(e) = renderer.render(&simulation) {
            eprintln!("Render error: {}", e);
            break;
        }
    }
}
