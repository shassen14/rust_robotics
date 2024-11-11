use crate::utils::geometry;
use crate::utils::geometry::CircleS;
use num_traits::Float;
use num_traits::Zero;

// TODO: make this a one dimensional vector which uce index arithmetics instead of
// the traditional. This is only for performance sake
pub struct GridMap2D<T, U> {
    pub map: Vec<Vec<T>>,
    pub x_range: [U; 2],
    pub y_range: [U; 2],
    pub resolution: U,
}

impl<T, U> GridMap2D<T, U>
where
    T: Zero + Copy + std::fmt::Debug,
    U: Float + Copy,
{
    pub fn new(
        obstacles: &Vec<(geometry::Shape2D<U>, T)>,
        x_range: &[U; 2],
        y_range: &[U; 2],
        resolution: U,
    ) -> Self {
        // index length
        let x_length: usize = ((x_range[1] - x_range[0]) / resolution)
            .round()
            .to_usize()
            .expect("Couldn't convert Type U to isize");
        let y_length: usize = ((y_range[1] - y_range[0]) / resolution)
            .round()
            .to_usize()
            .expect("Couldn't convert Type U to isize");

        // TODO: check if x_length or y_length
        let mut obs_map: Vec<Vec<T>> = vec![vec![Zero::zero(); x_length]; y_length];
        let new_obstacles = Self::shapes_unit_to_index(obstacles, x_range, y_range, resolution);

        Self::process_shapes(&mut obs_map, &new_obstacles);

        println!("{:?}", obs_map);
        GridMap2D::<T, U> {
            map: obs_map,
            x_range: *x_range,
            y_range: *y_range,
            resolution: resolution,
        }
    }

    fn shapes_unit_to_index(
        obstacles: &Vec<(geometry::Shape2D<U>, T)>,
        x_range: &[U; 2],
        y_range: &[U; 2],
        resolution: U,
    ) -> Vec<(geometry::Shape2D<isize>, T)> {
        let mut obs_index: Vec<(geometry::Shape2D<isize>, T)> = Vec::with_capacity(obstacles.len());

        for (shape, cost) in obstacles {
            match shape {
                geometry::Shape2D::<U>::Circle(circle) => {
                    obs_index.push((
                        geometry::Shape2D::Circle(CircleS::new(
                            (
                                ((circle.center.0 - x_range[0]) / resolution)
                                    .round()
                                    .to_isize()
                                    .expect("cannot convert f64 to isize"),
                                ((circle.center.1 - y_range[0]) / resolution)
                                    .round()
                                    .to_isize()
                                    .expect("cannot convert f64 to isize"),
                            ),
                            (circle.radius / resolution)
                                .round()
                                .to_isize()
                                .expect("cannot convert f64 to isize"),
                        )),
                        *cost,
                    ));
                }
                geometry::Shape2D::<U>::Polygon(polygon) => {}
            }
        }

        obs_index
    }

    fn process_shapes(map: &mut Vec<Vec<T>>, obstacles: &Vec<(geometry::Shape2D<isize>, T)>) -> () {
        // TODO: very slow, time complexity is MNO where M is number of obstacles,
        // N is rows, O is columns
        for (shape, cost) in obstacles {
            match shape {
                geometry::Shape2D::<isize>::Circle(circle) => {
                    // row or y
                    for (y_index, row) in map.clone().iter().enumerate() {
                        // columng or x
                        for (x_index, _col) in row.iter().enumerate() {
                            let x_dif = (circle.center.0 - x_index as isize) as f64;
                            let y_dif = (circle.center.1 - y_index as isize) as f64;
                            let distance =
                                f64::sqrt(x_dif * x_dif + y_dif * y_dif).round() as isize;

                            if distance < circle.radius {
                                map[y_index][x_index] = *cost;
                            }
                        }
                    }
                }
                geometry::Shape2D::<isize>::Polygon(polygon) => {}
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
        let obstacles = vec![(
            geometry::Shape2D::<f64>::Circle(CircleS::new((0.0, 0.0), 1.5)),
            1.0,
        )];

        let x_range: [f64; 2] = [-5.0, 5.0];
        let y_range: [f64; 2] = [-5.0, 5.0];
        let resolution: f64 = 1.0;
        let map = GridMap2D::<f64, f64>::new(&obstacles, &x_range, &y_range, resolution);

        assert!(true);
    }
}
