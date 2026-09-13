//! 父进程只依赖协议和监督接口；仅worker特性链接原生二维算法。
pub mod input;
pub mod archive;
pub mod protocol;
pub mod supervisor;
#[cfg(feature = "worker")]
pub mod worker;
