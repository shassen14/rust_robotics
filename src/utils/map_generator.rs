use crate::utils::geometry;
use num_traits::Float;
use num_traits::Zero;

// TODO: make this a one dimensional vector which uce index arithmetics instead of
// the traditional. This is only for performance sake
pub struct GridMap2D<T, U> {
    map: Vec<Vec<T>>,
    x_range: [U; 2],
    y_range: [U; 2],
    resolution: U,
}

impl<T, U> GridMap2D<T, U>
where
    T: Zero + Clone + std::fmt::Debug,
    U: Float + Copy,
{
    pub fn new(
        obstacles: &Vec<geometry::Shape2D<T>>,
        x_range: &[U; 2],
        y_range: &[U; 2],
        resolution: U,
    ) {
        // index length
        let x_length: usize = ((x_range[1] - x_range[0]) / resolution)
            .to_usize()
            .expect("Couldn't convert Type U to usize");
        let y_length: usize = ((y_range[1] - y_range[0]) / resolution)
            .to_usize()
            .expect("Couldn't convert Type U to usize");
        let total_length = x_length * y_length;

        // TODO: check if x_length or y_length
        let mut obs_map: Vec<Vec<T>> = vec![vec![Zero::zero(); x_length]; y_length];

        println!("{:?}", obs_map)
    }

    fn process_shapes<I: Iterator<Item = geometry::Shape2D<U>> + Copy>(&self, obstacles: &I) {
        for shape in *obstacles {
            match shape {
                geometry::Shape2D::<U>::Circle(circle) => {}
                geometry::Shape2D::<U>::Polygon(polygon) => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geometry::CircleS;

    #[test]
    fn test_gridmap() {
        let obstacles = vec![geometry::Shape2D::<f64>::Circle(CircleS::new(
            (0.0, 0.0),
            4.0,
        ))];

        let x_range: [f64; 2] = [-5.0, 5.0];
        let y_range: [f64; 2] = [-5.0, 5.0];
        let resolution: f64 = 0.5;
        GridMap2D::<f64, f64>::new(&obstacles, &x_range, &y_range, resolution);

        assert!(false);
    }
}
