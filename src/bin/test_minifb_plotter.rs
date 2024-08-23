use minifb::{Key, Window, WindowOptions};
use plotters::prelude::*;
use plotters_bitmap::bitmap_pixel::BGRXPixel;
use plotters_bitmap::BitMapBackend;
use std::collections::VecDeque;
// use std::time::SystemTime;

// TODO: what is this
use std::borrow::{Borrow, BorrowMut};
use std::error::Error;

// rust_robotics
use nalgebra as na;
use rust_robotics::models::base;
use rust_robotics::models::generic::ca_1dof;
use rust_robotics::num_methods::runge_kutta;

const W: usize = 800;
const H: usize = 400;
const FPS: u32 = 30;

#[derive(Debug)]
struct BufferWrapper(Vec<u32>);
impl Borrow<[u8]> for BufferWrapper {
    fn borrow(&self) -> &[u8] {
        // Safe for alignment: align_of(u8) <= align_of(u32)
        // Safe for cast: u32 can be thought of as being transparent over [u8; 4]
        unsafe { std::slice::from_raw_parts(self.0.as_ptr() as *const u8, self.0.len() * 4) }
    }
}
impl BorrowMut<[u8]> for BufferWrapper {
    fn borrow_mut(&mut self) -> &mut [u8] {
        // Safe for alignment: align_of(u8) <= align_of(u32)
        // Safe for cast: u32 can be thought of as being transparent over [u8; 4]
        unsafe { std::slice::from_raw_parts_mut(self.0.as_mut_ptr() as *mut u8, self.0.len() * 4) }
    }
}
impl Borrow<[u32]> for BufferWrapper {
    fn borrow(&self) -> &[u32] {
        self.0.as_slice()
    }
}
impl BorrowMut<[u32]> for BufferWrapper {
    fn borrow_mut(&mut self) -> &mut [u32] {
        self.0.as_mut_slice()
    }
}

fn get_window_title(title: String) -> String {
    format!("{}", title)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut buf = BufferWrapper(vec![0u32; W * H]);

    let mut window = Window::new(
        &get_window_title("Here is a title <Esc>=Exit".to_string()),
        W,
        H,
        WindowOptions::default(),
    )?;

    let cs = {
        // create an image using a buffer of integers that represent the colo
        let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
            buf.borrow_mut(),
            (W as u32, H as u32),
        )?
        .into_drawing_area();
        root.fill(&BLACK)?;

        // create 2d plane here
        let mut chart = ChartBuilder::on(&root)
            .margin(30)
            .set_left_and_bottom_label_area_size(30.)
            .build_cartesian_2d(-0.0..70.0, 0.0..7500.0)?;

        // axes label and line color
        chart
            .configure_mesh()
            .label_style(("sans-serif", 15).into_font().color(&GREEN))
            .axis_style(&GREEN)
            .draw()?;

        let cs = chart.into_chart_state();
        // root.present()?;
        cs
    };

    let mut data: VecDeque<(f64, na::SVector<f64, 3>)> = VecDeque::new();
    // let start_ts = SystemTime::now();
    // let mut last_flushed = 0.0;

    // time and initialize vehicle
    let start: f64 = 0.;
    let end: f64 = 70.;
    let step: f64 = 1. / FPS as f64;
    let state0: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(0., 1., 3.);

    let mut t0: f64 = start;
    let mut tf: f64 = t0 + step;
    let mut result = state0;
    let veh: ca_1dof::Model = ca_1dof::Model {};
    data.reserve(((end - start) / step) as usize);

    while window.is_open() && !window.is_key_down(Key::Escape) {
        if tf <= end {
            result = base::System::propagate(
                &veh,
                &result,
                &na::SVector::<f64, 0>::zeros(),
                t0,
                step,
                runge_kutta::rk4,
            );

            data.push_back((t0, result));
            t0 = tf;
            tf += step;
        }

        if true {
            let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
                buf.borrow_mut(),
                (W as u32, H as u32),
            )?
            .into_drawing_area();
            {
                let mut chart = cs.clone().restore(&root);
                chart.plotting_area().fill(&BLACK)?;

                chart
                    .configure_mesh()
                    .bold_line_style(&GREEN.mix(0.2))
                    .light_line_style(&TRANSPARENT)
                    .draw()?;

                chart.draw_series(data.iter().zip(data.iter().skip(1)).map(
                    |(&(t0, x0), &(t1, x1))| {
                        PathElement::new(vec![(t0, x0[0]), (t1, x1[0])], &GREEN)
                    },
                ))?;
            }
            // root.present()?;
        }

        window.update_with_buffer(buf.borrow(), W, H)?;
        // if tf <= end && data.len() > 2 {
        //     data.pop_front();
        // }
    }

    // println!("{:?}", data);

    Ok(())
}
