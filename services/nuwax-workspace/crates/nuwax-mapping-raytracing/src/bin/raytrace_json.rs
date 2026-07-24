// Copyright 2026 Nuwax Team
// SPDX-License-Identifier: Apache-2.0

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;

use nuwax_mapping_raytracing::binding::{execute, RayTraceRequest};

const INPUT_ERROR: u8 = 2;
const EXECUTION_ERROR: u8 = 3;
const OUTPUT_ERROR: u8 = 4;
const MAX_INPUT_BYTES: u64 = 32 * 1024 * 1024;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err((code, message)) => {
            eprintln!("raytrace_json: {message}");
            ExitCode::from(code)
        }
    }
}

fn run() -> Result<(), (u8, String)> {
    let options = Options::parse(std::env::args_os().skip(1))?;
    let request = read_request(options.input.as_ref())?;
    let response = execute(request).map_err(|error| (EXECUTION_ERROR, error.to_string()))?;

    let stdout = io::stdout();
    let mut output = BufWriter::new(stdout.lock());
    let result = if options.pretty {
        serde_json::to_writer_pretty(&mut output, &response)
    } else {
        serde_json::to_writer(&mut output, &response)
    };
    result.map_err(|error| (OUTPUT_ERROR, format!("failed to encode output: {error}")))?;
    writeln!(output).map_err(|error| (OUTPUT_ERROR, format!("failed to write output: {error}")))?;
    output
        .flush()
        .map_err(|error| (OUTPUT_ERROR, format!("failed to flush output: {error}")))
}

fn read_request(path: Option<&PathBuf>) -> Result<RayTraceRequest, (u8, String)> {
    let mut input = Vec::new();
    match path {
        Some(path) => {
            let file = File::open(path).map_err(|error| {
                (
                    INPUT_ERROR,
                    format!("failed to read {}: {error}", path.display()),
                )
            })?;
            let metadata = file.metadata().map_err(|error| {
                (
                    INPUT_ERROR,
                    format!("failed to inspect {}: {error}", path.display()),
                )
            })?;
            if metadata.len() > MAX_INPUT_BYTES {
                return Err((
                    INPUT_ERROR,
                    format!(
                        "input exceeds the {MAX_INPUT_BYTES}-byte limit: {}",
                        path.display()
                    ),
                ));
            }
            read_limited(
                BufReader::new(file),
                &mut input,
                &path.display().to_string(),
            )?;
        }
        None => read_limited(io::stdin().lock(), &mut input, "stdin")?,
    };
    serde_json::from_slice(&input)
        .map_err(|error| (INPUT_ERROR, format!("invalid request JSON: {error}")))
}

fn read_limited(reader: impl Read, input: &mut Vec<u8>, source: &str) -> Result<(), (u8, String)> {
    reader
        .take(MAX_INPUT_BYTES + 1)
        .read_to_end(input)
        .map_err(|error| (INPUT_ERROR, format!("failed to read {source}: {error}")))?;
    if input.len() as u64 > MAX_INPUT_BYTES {
        return Err((
            INPUT_ERROR,
            format!("input exceeds the {MAX_INPUT_BYTES}-byte limit: {source}"),
        ));
    }
    Ok(())
}

struct Options {
    input: Option<PathBuf>,
    pretty: bool,
}

impl Options {
    fn parse(args: impl Iterator<Item = std::ffi::OsString>) -> Result<Self, (u8, String)> {
        let mut input = None;
        let mut pretty = false;
        let mut args = args;
        while let Some(argument) = args.next() {
            match argument.to_str() {
                Some("--input") => {
                    let path = args
                        .next()
                        .ok_or_else(|| (INPUT_ERROR, "--input requires a file path".to_owned()))?;
                    if input.replace(PathBuf::from(path)).is_some() {
                        return Err((INPUT_ERROR, "--input may only be specified once".to_owned()));
                    }
                }
                Some("--pretty") => pretty = true,
                Some(other) => {
                    return Err((INPUT_ERROR, format!("unknown argument: {other}")));
                }
                None => {
                    return Err((INPUT_ERROR, "arguments must be valid UTF-8".to_owned()));
                }
            }
        }
        Ok(Self { input, pretty })
    }
}
