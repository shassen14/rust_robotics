use minifb::{Window, WindowOptions};

const W: usize = 800;
const H: usize = 400;

fn main() -> Result<(), std::io::Error> {
    let _window = Window::new(
        &"Here is a title".to_string(),
        W,
        H,
        WindowOptions::default(),
    );

    Ok(())
}
