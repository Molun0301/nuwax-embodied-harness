fn main() {
    if let Err(error) = nuwax_lio_backends::cli(nuwax_lio_backends::BackendKind::PointLio) {
        eprintln!("pointlio backend: {error}");
        std::process::exit(2);
    }
}
