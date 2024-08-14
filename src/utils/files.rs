use serde::de::DeserializeOwned;
use std::fs;
use std::process::exit;

// Found from https://codingpackets.com/blog/rust-load-a-toml-file/
pub fn read_config<T>(config_path: &str) -> T
where
    T: DeserializeOwned,
{
    println!("Attempting to read file `{}`", config_path);

    // Converts the file text to string
    let contents = match fs::read_to_string(config_path) {
        // If successful return the files text as `contents`.
        // `c` is a local variable.
        Ok(c) => c,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            eprintln!("Could not read file `{}`", config_path);
            // Exit the program with exit code `1`.
            exit(1);
        }
    };

    // Use a `match` block to return the
    // file `contents` as a `Data struct: Ok(d)`
    // or handle any `errors: Err(_)`.
    let config_params: T = match toml::from_str(&contents) {
        // If successful, return data as `Data` struct.
        // `d` is a local variable.
        Ok(d) => d,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            eprintln!("Unable to load data from `{}`", config_path);
            // Exit the program with exit code `1`.
            exit(1);
        }
    };

    println!("Finished reading file `{}`", config_path);

    config_params
}
