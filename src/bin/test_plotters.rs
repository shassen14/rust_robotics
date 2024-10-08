use full_palette::PURPLE;
// Plotters
use plotters::prelude::*;
use plotters_bitmap::bitmap_pixel::RGBPixel;
use plotters_bitmap::BitMapBackend;

const W: usize = 640;
const H: usize = 360;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::<RGBPixel>::new("bob.png", (W as u32, H as u32)).into_drawing_area();
    root.fill(&BLACK)?;

    let mut chart = ChartBuilder::on(&root)
        .margin(10)
        .set_all_label_area_size(30)
        .build_cartesian_2d(-1.0..1.0, -1.0..1.0)?;

    chart
        .configure_mesh()
        .label_style(("sans-serif", 15).into_font().color(&WHITE))
        .axis_style(&WHITE)
        .draw()?;

    let data = [(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (-0.5, -0.5)];

    chart.draw_series(LineSeries::new(data, &YELLOW))?;
    chart.draw_series(std::iter::once(Rectangle::new(
        [(0.5, 0.5), (0.75, 0.75)],
        &WHITE,
    )))?;

    // chart.draw_series(std::iter::once(Polygon::new(
    //     [
    //         (-0.9, 0.9),
    //         (-0.8, 0.8),
    //         (-0.7, 0.85),
    //         (-0.8, 0.9),
    //         (-0.9, 0.9),
    //     ],
    //     &PURPLE,
    // )))?;

    let path = PathElement::new(
        [
            (-0.9, 0.9),
            (-0.8, 0.8),
            (-0.7, 0.85),
            (-0.8, 0.9),
            (-0.9, 0.9),
        ],
        &PURPLE,
    );

    chart.draw_series(std::iter::once(path))?;

    Ok(())
}
