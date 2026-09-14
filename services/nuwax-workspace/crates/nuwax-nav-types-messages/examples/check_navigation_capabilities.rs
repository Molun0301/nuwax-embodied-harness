//! 逐行校验标定JSON，供跨语言契约对照和本地配置诊断。
use nuwax_nav_types_messages::NavigationCapabilities;
use std::io::{self, BufRead};
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("读取输入");
        match NavigationCapabilities::from_json(&line) {
            Ok(value) => println!("{}", serde_json::to_string(&value).unwrap()),
            Err(_) => println!("invalid"),
        }
    }
}
