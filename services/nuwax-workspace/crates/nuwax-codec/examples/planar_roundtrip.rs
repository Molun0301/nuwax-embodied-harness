//! 二维LCM负载跨语言核对入口，不连接运动设备。
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("用法：planar_roundtrip 输入负载 输出负载".into());
    }
    let input = std::fs::read(&args[0])?;
    let scan = nuwax_codec::lcm::planar::decode(&input)?;
    std::fs::write(&args[1], nuwax_codec::lcm::planar::encode(&scan)?)?;
    Ok(())
}
