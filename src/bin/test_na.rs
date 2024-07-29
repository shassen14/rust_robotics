extern crate nalgebra as na;

fn main() {
    // interesting enough, the fill the data by columns instead of row by row
    // Therefore, for the intended matrix, we need to transpose
    let _a: na::SMatrix<f64, 9, 9> = na::SMatrix::<f64, 9, 9>::from_vec(vec![
        0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
    ])
    .transpose();

    // interesting enough, the fill the data by columns instead of row by row
    // Therefore, for the intended matrix, we need to transpose
    let _b: na::SMatrix<f64, 9, 9> =
        na::SMatrix::<f64, 9, 9>::from_array_storage(na::ArrayStorage([
            [0., 1., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 1., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 1., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 1.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
        ]))
        .transpose();
    const F: na::SMatrix<f64, 9, 9> =
        na::SMatrix::<f64, 9, 9>::from_array_storage(na::ArrayStorage([
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [1., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 1., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 1., 0.],
        ]));

    // const F: na::SMatrix<f64, 9, 9> = b;

    // for row in &B {
    //     println!("{}", row)
    // }
    println!("{}", F);
    // println!("{}", b[(0, 1)]);
}
