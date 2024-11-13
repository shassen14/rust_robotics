use crate::utils::geometry;
use crate::utils::geometry::CircleS;
use num_traits::Float;
use num_traits::Zero;
use serde::{Deserialize, Serialize};

// TODO: Move this??? and other structs, maybe even convert functions
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MapGeneratorParams<T, U> {
    pub shape_list: ShapeList<T, U>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ShapeList<T, U> {
    pub circles: Vec<(T, T, T, U)>,
    pub polygons: Vec<(Vec<(T, T)>, U)>,
}

pub fn convert_circle_param_to_struct<T, U>(
    circles: &Vec<(T, T, T, U)>,
) -> Vec<(geometry::Shape2D<T>, U)>
where
    T: Zero + Copy,
    U: Copy,
{
    circles
        .iter()
        .map(|param| {
            (
                geometry::Shape2D::Circle(CircleS::new((param.0, param.1), param.2)),
                param.3,
            )
        })
        .collect()
}

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
    T: Zero + Copy,
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
            .expect("Couldn't convert Type U to usize")
            + 1usize;
        let y_length: usize = ((y_range[1] - y_range[0]) / resolution)
            .round()
            .to_usize()
            .expect("Couldn't convert Type U to usize")
            + 1usize;

        // TODO: check if x_length or y_length
        let mut obs_map: Vec<Vec<T>> = vec![vec![Zero::zero(); x_length]; y_length];
        let new_obstacles = Self::shapes_unit_to_index(obstacles, x_range, y_range, resolution);

        Self::process_shapes(&mut obs_map, &new_obstacles);

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
    ) -> Vec<(geometry::Shape2D<usize>, T)> {
        let mut obs_index: Vec<(geometry::Shape2D<usize>, T)> = Vec::with_capacity(obstacles.len());

        for (shape, cost) in obstacles {
            match shape {
                geometry::Shape2D::<U>::Circle(circle) => {
                    obs_index.push((
                        geometry::Shape2D::Circle(CircleS::new(
                            (
                                ((circle.center.0 - x_range[0]) / resolution)
                                    .round()
                                    .to_usize()
                                    .expect("cannot convert f64 to usize"),
                                ((circle.center.1 - y_range[0]) / resolution)
                                    .round()
                                    .to_usize()
                                    .expect("cannot convert f64 to usize"),
                            ),
                            (circle.radius / resolution)
                                .round()
                                .to_usize()
                                .expect("cannot convert f64 to usize"),
                        )),
                        *cost,
                    ));
                }
                geometry::Shape2D::<U>::Polygon(polygon) => {
                    todo!()
                }
            }
        }

        obs_index
    }

    fn process_shapes(map: &mut Vec<Vec<T>>, obstacles: &Vec<(geometry::Shape2D<usize>, T)>) -> () {
        // TODO: very slow, time complexity is MNO where M is number of obstacles,
        // N is rows, O is columns
        for (shape, cost) in obstacles {
            match shape {
                geometry::Shape2D::<usize>::Circle(circle) => {
                    // row or y
                    for (y_index, row) in map.clone().iter().enumerate() {
                        // columng or x
                        for (x_index, _value) in row.iter().enumerate() {
                            let x_dif = if circle.center.0 > x_index {
                                (circle.center.0 - x_index) as f64
                            } else {
                                (x_index - circle.center.0) as f64
                            };
                            let y_dif = if circle.center.1 > y_index {
                                (circle.center.1 - y_index) as f64
                            } else {
                                (y_index - circle.center.1) as f64
                            };
                            let distance =
                                f64::sqrt(x_dif * x_dif + y_dif * y_dif).round() as usize;

                            // TODO: fill or just shell modes?
                            if distance <= circle.radius {
                                map[y_index][x_index] = *cost;
                            }
                        }
                    }
                }
                geometry::Shape2D::<usize>::Polygon(polygon) => {
                    todo!()
                }
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
            geometry::Shape2D::<f64>::Circle(CircleS::new((0.0, 0.0), 1.0)),
            1.0,
        )];

        let x_range: [f64; 2] = [-1.0, 1.0];
        let y_range: [f64; 2] = [-1.0, 1.0];
        let resolution: f64 = 0.1;
        let map = GridMap2D::<f64, f64>::new(&obstacles, &x_range, &y_range, resolution);

        assert!(true);
    }
}
