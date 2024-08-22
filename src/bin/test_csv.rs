use std::{error::Error, io, process, result};

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
struct Record {
    city: String,
    region: String,
    country: String,
    population: Option<u64>,
}

fn example_writer() -> Result<(), Box<dyn Error>> {
    let mut wtr = csv::Writer::from_path("logs/examples/test_csv.csv")?;

    // When writing records with Serde using structs, the header row is written
    // automatically.
    wtr.serialize(Record {
        city: "Southborough".to_string(),
        region: "MA".to_string(),
        country: "United States".to_string(),
        population: Some(9686),
    })?;
    wtr.serialize(Record {
        city: "Northbridge".to_string(),
        region: "MA".to_string(),
        country: "United States".to_string(),
        population: Some(14061),
    })?;
    wtr.flush()?;
    Ok(())
}

fn example_reader() -> Result<(), Box<dyn Error>> {
    let mut rdr = csv::Reader::from_path("logs/examples/test_csv.csv")?;

    for result in rdr.deserialize() {
        let record: Record = result?;
        println!("{:#?}", record);
    }
    Ok(())
}

fn main() {
    if let Err(err) = example_writer() {
        println!("error running writer example: {}", err);
        process::exit(1);
    }

    if let Err(err) = example_reader() {
        println!("error running reading example: {}", err);
        process::exit(1);
    }
}
