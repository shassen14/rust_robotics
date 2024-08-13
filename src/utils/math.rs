// not really sure what to call this or where to put these
// functions. Just needing them

use std::fmt::Debug;

pub fn bound_value<T>(value: T, lower_bound: T, upper_bound: T) -> T
where
    T: Debug + std::cmp::PartialOrd,
{
    // Error occurs if
    assert!(
        lower_bound <= upper_bound,
        "lower bound ({:?}) must be lower than or equal upper bound ({:?})",
        lower_bound,
        upper_bound
    );

    if value < lower_bound {
        return lower_bound;
    }
    if value > upper_bound {
        return upper_bound;
    }
    value
}
