fn main() {
    if let Err(error) = nuwax_lio_backends::cli(nuwax_lio_backends::BackendKind::FastLio2) {
        eprintln!("fastlio2 backend: {error}");
        std::process::exit(2);
    }
}
