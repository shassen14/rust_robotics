use crate::utils::geometry;
use crate::utils::geometry::CircleS;
use num_traits::Float;
use num_traits::Zero;
use serde::{Deserialize, Serialize};

use super::geometry::PolygonS;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MapGeneratorParams<T, U> {
    pub shape_list: ShapeList<T, U>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ShapeList<T, U> {
    // x, y, radius, cost
    pub circles: Vec<(T, T, T, U)>,
    // x, y, cost
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

// TODO: figure out this cloning business. I DO NOT LIKE CLONING
pub fn convert_polygon_param_to_struct<T, U>(
    polygons: &Vec<(Vec<(T, T)>, U)>,
) -> Vec<(geometry::Shape2D<T>, U)>
where
    T: Zero + Copy,
    U: Copy,
{
    polygons
        .iter()
        .map(|param| {
            (
                geometry::Shape2D::Polygon(PolygonS::new(param.0.clone().into_iter())),
                param.1,
            )
        })
        .collect()
}

// TODO: make this a one dimensional vector which use index arithmetics instead of
// the traditional. This is only for performance sake
/// 2D Grid Map
///
/// * Template T is the cost value type
/// * Template U is the resolution type (most likely float)
pub struct GridMap2D<T, U> {
    pub map: Vec<Vec<T>>,
    pub x_range: [U; 2],
    pub y_range: [U; 2],
    pub resolution: U,
}

// Create a 2D Grid Map given a list of obstacles, map size and resolution
impl<T, U> GridMap2D<T, U>
where
    T: Zero + Copy,
    U: Float + Copy,
{
    /// Constructor
    ///
    /// * `obstacles` - Vector of 2D Shapes and their associated cost values
    /// * `x_range` - Assuming NED Frame [West -, East +]
    /// * `y_range` - Assuming NED Frame [South -, North +]
    /// * `resolution` - How detailed the map is
    /// * returns Self object (GridMap2D)
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
        // Declare a 2D grid x by y indices
        let mut obs_map: Vec<Vec<T>> = vec![vec![Zero::zero(); x_length]; y_length];

        // converts obstacle list with their cost value to 2D Grid Map
        let new_obstacles = Self::shapes_unit_to_index(obstacles, x_range, y_range, resolution);

        // Puts the obstacle cost values into the map
        Self::process_shapes(&new_obstacles, &mut obs_map);

        GridMap2D::<T, U> {
            map: obs_map,
            x_range: *x_range,
            y_range: *y_range,
            resolution: resolution,
        }
    }

    /// Converts the shapes units (meters, feet, miles, etc.) to index coordinates
    ///
    /// # Arguments
    ///
    /// * `obstacles` - Vector of 2D Shapes and their associated cost values
    /// * `x_range` - Assuming NED Frame [West -, East +]
    /// * `y_range` - Assuming NED Frame [South -, North +]
    /// * `resolution` - How detailed the map is
    /// * Returns a vector of shapes in the index frame with their associated costs
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
                geometry::Shape2D::<U>::Polygon(_polygon) => {
                    todo!()
                }
            }
        }

        obs_index
    }

    /// Put the actual cost values from the obstacles into the GridMap2D (map)
    ///
    /// # Arguments
    ///
    /// * `obstacles` [in] - Vector of 2D shapes with their associated cost value
    /// * `map` [out]- 2D discretized map with space already reserved
    fn process_shapes(obstacles: &Vec<(geometry::Shape2D<usize>, T)>, map: &mut Vec<Vec<T>>) -> () {
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
                geometry::Shape2D::<usize>::Polygon(_polygon) => {
                    todo!()
                }
            }
        }
    }
}

// TODO: This tests nothing. Actually unit test this
#[cfg(test)]
mod tests {
    use super::*;
    use geometry::CircleS;

    #[test]
    fn test_gridmap() {
        // TODO: have an actual test
        let obstacles = vec![(
            geometry::Shape2D::<f64>::Circle(CircleS::new((0.0, 0.0), 1.0)),
            1.0,
        )];

        let x_range: [f64; 2] = [-1.0, 1.0];
        let y_range: [f64; 2] = [-1.0, 1.0];
        let resolution: f64 = 0.1;
        let _map = GridMap2D::<f64, f64>::new(&obstacles, &x_range, &y_range, resolution);

        assert!(true);
    }
}
