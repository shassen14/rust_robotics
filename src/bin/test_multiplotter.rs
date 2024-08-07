// Plotters
use plotters::prelude::*;
use plotters_bitmap::bitmap_pixel::RGBPixel;
use plotters_bitmap::BitMapBackend;

const W: usize = 640;
const H: usize = 360;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::<RGBPixel>::new("bob.png", (W as u32, H as u32)).into_drawing_area();
    root.fill(&BLACK)?;

    let (left, right) = root.split_horizontally((50).percent_width());

    let mut left_chart = ChartBuilder::on(&left)
        .margin(5)
        .set_all_label_area_size(35)
        .build_cartesian_2d(-1.0..1.0, -1.0..1.0)?;

    left_chart
        .configure_mesh()
        .label_style(("sans-serif", 15).into_font().color(&WHITE))
        .axis_style(&WHITE)
        .draw()?;

    let data = [(-1.0, -1.0), (1., 1.0)];

    left_chart.draw_series(LineSeries::new(data, &YELLOW))?;
    left_chart.draw_series(std::iter::once(Rectangle::new(
        [(0.5, 0.5), (0.75, 0.75)],
        &WHITE,
    )))?;

    let mut right_chart = ChartBuilder::on(&right)
        .margin(5)
        .set_all_label_area_size(35)
        .build_cartesian_2d(-1.0..1.0, -1.0..1.0)?;

    right_chart
        .configure_mesh()
        .label_style(("sans-serif", 15).into_font().color(&WHITE))
        .axis_style(&WHITE)
        .draw()?;

    let data2 = [(-1.0, 1.0), (1., -1.0)];

    right_chart.draw_series(LineSeries::new(data2, &YELLOW))?;
    // left_chart.draw_series(std::iter::once(Rectangle::new(
    //     [(0.5, 0.5), (0.75, 0.75)],
    //     &WHITE,
    // )))?;

    Ok(())
}
