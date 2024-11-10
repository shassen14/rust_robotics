use std::mem::discriminant;

use num_traits::Zero;

/// Variant Type Implementation
#[derive(Clone, Debug)]
pub enum Shape2D<T> {
    Circle(CircleS<T>),
    Polygon(PolygonS<T>),
}

// This implementation is to check the Shape type, not necessarily if they are the exact same shape
// This is to ensure how to handle the shape's properties
// TODO: decide if this is what it means to be the "same shape"
impl<T> PartialEq for Shape2D<T> {
    fn eq(&self, other: &Self) -> bool {
        discriminant(self) == discriminant(other)
    }
}

impl<T> Eq for Shape2D<T> {}

// impl<T> Shape2D<T> {
//     pub fn is_same_shape_type(&self, other: &Self) -> bool {
//         discriminant(self) == discriminant(other)
//     }
// }

#[derive(Clone, Debug)]
pub struct CircleS<T> {
    pub center: (T, T),
    pub radius: T,
}

impl<T: Zero> CircleS<T> {
    pub fn new(center: (T, T), radius: T) -> Self {
        CircleS {
            center: center,
            radius: radius,
        }
    }
    pub fn default() -> Self {
        CircleS {
            center: (Zero::zero(), Zero::zero()),
            radius: (Zero::zero()),
        }
    }
}

#[derive(Clone, Debug)]
pub struct PolygonS<T> {
    pub points: Vec<(T, T)>,
}

impl<T: Zero> PolygonS<T> {
    pub fn new<I>(points: I) -> Self
    where
        I: Iterator<Item = (T, T)>,
    {
        PolygonS {
            points: points.collect(),
        }
    }
    pub fn default() -> Self {
        PolygonS {
            points: Vec::default(),
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_shape2d() {
        let circle = Shape2D::<i32>::Circle(CircleS {
            center: (0, 0),
            radius: 2,
        });

        let polygon = Shape2D::<i32>::Polygon(PolygonS {
            points: vec![(-1, 1), (5, 5), (3, 3)],
        });

        let mut shape_vector: Vec<Shape2D<i32>> = Vec::default();
        shape_vector.push(circle.clone());
        shape_vector.push(polygon.clone());

        // The vector length should be 2,
        // The pops compare if they are the same shape
        assert_eq!(shape_vector.len(), 2);
        assert_eq!(shape_vector.pop(), Some(polygon.clone()));
        assert_eq!(shape_vector.pop(), Some(circle.clone()));
        assert_eq!(shape_vector.pop(), None);

        // assert!(shape_vector
        //     .pop()
        //     .expect("No Shape")
        //     .is_same_shape_type(&polygon));
        // assert!(shape_vector
        //     .pop()
        //     .expect("No Shape")
        //     .is_same_shape_type(&circle));
    }

    #[test]
    fn test_circle() {
        let circle1 = Shape2D::<i32>::Circle(CircleS::new((0, 0), 0));
        let circle2 = Shape2D::<i32>::Circle(CircleS::default());

        assert_eq!(circle1, circle2)
    }

    #[test]
    fn test_polygon() {
        let polygon1 = Shape2D::<i32>::Polygon(PolygonS::new(Vec::default().into_iter()));
        let polygon2 = Shape2D::<i32>::Polygon(PolygonS::default());

        assert_eq!(polygon1, polygon2)
    }
}
