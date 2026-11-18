//! 行为时间与样本来源时间。进程启动时由环境决定时钟来源。
use memmap2::Mmap;
use std::{
    fs::File,
    future::Future,
    io::ErrorKind,
    ops::{Add, Sub},
    sync::{
        atomic::{fence, AtomicU64, Ordering},
        LazyLock,
    },
    thread,
    time::{Duration, Instant as SystemInstant, SystemTime, UNIX_EPOCH},
};

const CLOCK_LEN: usize = 40;
const FILE_POLL: Duration = Duration::from_millis(10);

static DEVICE_PATH: LazyLock<Option<std::ffi::OsString>> =
    LazyLock::new(|| std::env::var_os("NUWAX_DEVICE_CLOCK").filter(|path| !path.is_empty()));
static DEVICE: LazyLock<Mmap> = LazyLock::new(|| {
    let path = DEVICE_PATH.as_ref().unwrap();
    let file = loop {
        match File::open(path) {
            Ok(file) => break file,
            Err(error) if error.kind() == ErrorKind::NotFound => thread::sleep(FILE_POLL),
            Err(error) => panic!("打开设备时钟失败：{error}"),
        }
    };
    // 同一会话的时钟文件保持长度与映射，平台按序号协议更新。
    unsafe { memmap2::MmapOptions::new().len(CLOCK_LEN).map(&file) }.expect("映射设备时钟失败")
});

struct Sample {
    source_ns: u64,
    rate: f64,
    paused: bool,
}

fn read_sample(map: &Mmap) -> Sample {
    let field = |offset: usize| {
        // 映射起点按页对齐，各字段为对齐的八字节小端值。
        unsafe { AtomicU64::from_ptr(map.as_ptr().add(offset).cast_mut().cast::<u64>()) }
    };
    let sequence = field(8);
    loop {
        let before = u64::from_le(sequence.load(Ordering::Acquire));
        if before & 1 == 0 {
            let source_ns = u64::from_le(field(16).load(Ordering::Relaxed));
            let rate = f64::from_bits(u64::from_le(field(24).load(Ordering::Relaxed)));
            let paused = u64::from_le(field(32).load(Ordering::Relaxed)) == 1;
            fence(Ordering::Acquire);
            if u64::from_le(sequence.load(Ordering::Relaxed)) == before {
                return Sample {
                    source_ns,
                    rate,
                    paused,
                };
            }
        }
        std::hint::spin_loop();
    }
}

/// 进程是否启用了设备时钟。
pub fn uses_device_clock() -> bool {
    DEVICE_PATH.is_some()
}

/// 当前样本来源时刻，未配置设备时为系统 Unix 纳秒。
pub fn source_now_ns() -> u64 {
    if uses_device_clock() {
        read_sample(&DEVICE).source_ns
    } else {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum Time {
    System(SystemInstant),
    Device(u64),
}

/// 行为时刻。比较和算术在同一进程选定的时域内进行。
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Instant(Time);

impl Instant {
    /// 读取当前行为时刻。
    pub fn now() -> Self {
        if uses_device_clock() {
            Self(Time::Device(read_sample(&DEVICE).source_ns))
        } else {
            Self(Time::System(SystemInstant::now()))
        }
    }

    /// 自此时刻起经过的行为时间。
    pub fn elapsed(self) -> Duration {
        Self::now().duration_since(self)
    }

    /// 返回两个时刻的时间差，未来起点的差值为零。
    pub fn duration_since(self, earlier: Self) -> Duration {
        match (self.0, earlier.0) {
            (Time::System(now), Time::System(earlier)) => now.duration_since(earlier),
            (Time::Device(now), Time::Device(earlier)) => {
                Duration::from_nanos(now.saturating_sub(earlier))
            }
            _ => unreachable!("一个进程只使用一种行为时钟"),
        }
    }
}

impl Add<Duration> for Instant {
    type Output = Self;

    fn add(self, duration: Duration) -> Self {
        Self(match self.0 {
            Time::System(value) => Time::System(value + duration),
            Time::Device(value) => Time::Device(value + duration.as_nanos() as u64),
        })
    }
}

impl Sub<Duration> for Instant {
    type Output = Self;

    fn sub(self, duration: Duration) -> Self {
        Self(match self.0 {
            Time::System(value) => Time::System(value - duration),
            Time::Device(value) => Time::Device(value - duration.as_nanos() as u64),
        })
    }
}

impl Sub<Instant> for Instant {
    type Output = Duration;

    fn sub(self, earlier: Instant) -> Duration {
        self.duration_since(earlier)
    }
}

/// 下一段真实等待时长。行为期限到达时返回 `None`。
pub fn next_wait(deadline: Instant) -> Option<Duration> {
    match deadline.0 {
        Time::System(deadline) => deadline
            .checked_duration_since(SystemInstant::now())
            .filter(|remaining| !remaining.is_zero()),
        Time::Device(deadline) => {
            let sample = read_sample(&DEVICE);
            if sample.source_ns >= deadline {
                return None;
            }
            Some(if sample.paused {
                FILE_POLL
            } else if sample.rate == 0.0 {
                Duration::from_millis(1)
            } else {
                let seconds = (deadline - sample.source_ns) as f64 / (sample.rate * 1e9);
                Duration::from_secs_f64(seconds.min(FILE_POLL.as_secs_f64()))
            })
        }
    }
}

/// 异步等待到行为时刻。
pub async fn sleep_until(deadline: Instant) {
    match deadline.0 {
        Time::System(deadline) => tokio::time::sleep_until(deadline.into()).await,
        Time::Device(_) => {
            while let Some(wait) = next_wait(deadline) {
                tokio::time::sleep(wait).await;
            }
        }
    }
}

/// 异步等待一段行为时间。
pub async fn sleep_async(duration: Duration) {
    sleep_until(Instant::now() + duration).await;
}

/// 行为期限已经到达。
#[derive(Debug)]
pub struct Elapsed;

/// 在行为期限内等待异步操作结束。
pub async fn timeout<F: Future>(deadline: Instant, future: F) -> Result<F::Output, Elapsed> {
    match deadline.0 {
        Time::System(deadline) => tokio::time::timeout_at(deadline.into(), future)
            .await
            .map_err(|_| Elapsed),
        Time::Device(_) => tokio::select! {
            biased;
            value = future => Ok(value),
            _ = sleep_until(deadline) => Err(Elapsed),
        },
    }
}

/// 跳过错过的节拍，起点可选择立即触发或等待一个周期。
pub fn interval(period: Duration, delay_first: bool) -> Interval {
    let delay = if delay_first { period } else { Duration::ZERO };
    let inner = if uses_device_clock() {
        IntervalInner::Device {
            next: Instant::now() + delay,
            period,
        }
    } else {
        let mut ticker = tokio::time::interval_at(tokio::time::Instant::now() + delay, period);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        IntervalInner::System(ticker)
    };
    Interval(inner)
}

enum IntervalInner {
    System(tokio::time::Interval),
    Device { next: Instant, period: Duration },
}

/// 固定行为节拍。
pub struct Interval(IntervalInner);

impl Interval {
    /// 等待下一拍。等待被取消时保留原计划时刻。
    pub async fn tick(&mut self) {
        match &mut self.0 {
            IntervalInner::System(ticker) => {
                ticker.tick().await;
            }
            IntervalInner::Device { next, period } => {
                sleep_until(*next).await;
                let now = Instant::now();
                let offset = now.duration_since(*next).as_nanos() % period.as_nanos();
                *next = now + (*period - Duration::from_nanos(offset as u64));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{os::unix::fs::FileExt, process::Command};

    #[tokio::test]
    async fn system_clock() {
        assert!(!uses_device_clock());
        let unix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        assert!(source_now_ns() >= unix);
        let start = Instant::now();
        let duration = Duration::from_millis(3);
        assert_eq!((start + duration) - start, duration);
        sleep_async(duration).await;
        assert!(start.elapsed() >= duration);
        assert_eq!(
            timeout(Instant::now() + duration, async { 42 })
                .await
                .unwrap(),
            42
        );
        assert!(
            timeout(Instant::now() + duration, std::future::pending::<()>())
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn device_clock() {
        const CHILD: &str = "NEH_TEST_DEVICE_CLOCK";
        let Some(path) = std::env::var_os(CHILD) else {
            let dir = std::env::temp_dir().join(format!("neh-device-clock-{}", std::process::id()));
            std::fs::create_dir(&dir).unwrap();
            let path = dir.join("clock");
            let status = Command::new(std::env::current_exe().unwrap())
                .args(["--exact", "clock::tests::device_clock"])
                .env(CHILD, &path)
                .env("NUWAX_DEVICE_CLOCK", &path)
                .status()
                .unwrap();
            std::fs::remove_dir_all(dir).unwrap();
            assert!(status.success());
            return;
        };
        let initial = 1_700_000_000_000_000_000_u64;
        let reader = thread::spawn(source_now_ns);
        thread::sleep(Duration::from_millis(20));
        assert!(!reader.is_finished());

        let file = File::options()
            .read(true)
            .write(true)
            .create_new(true)
            .open(path)
            .unwrap();
        file.set_len(CLOCK_LEN as u64).unwrap();
        file.write_all_at(b"USIMCLK1", 0).unwrap();
        let publish = |sequence: u64, source: u64, rate: f64, paused: bool| {
            file.write_all_at(&(sequence - 1).to_le_bytes(), 8).unwrap();
            file.write_all_at(&source.to_le_bytes(), 16).unwrap();
            file.write_all_at(&rate.to_le_bytes(), 24).unwrap();
            file.write_all_at(&u64::from(paused).to_le_bytes(), 32)
                .unwrap();
            file.write_all_at(&sequence.to_le_bytes(), 8).unwrap();
        };
        publish(2, initial, 1.0, false);
        assert_eq!(reader.join().unwrap(), initial);
        let now = Instant::now();
        let duration = Duration::from_millis(4);
        let deadline = now + duration;
        for (sequence, rate, micros) in [
            (4, 0.5, 8000),
            (6, 1.0, 4000),
            (8, 2.5, 1600),
            (10, 0.0, 1000),
        ] {
            publish(sequence, initial, rate, false);
            assert_eq!(next_wait(deadline), Some(Duration::from_micros(micros)));
        }
        publish(12, initial, 0.5, false);
        assert_eq!(next_wait(now + Duration::from_secs(1)), Some(FILE_POLL));
        file.write_all_at(&13_u64.to_le_bytes(), 8).unwrap();
        let reader = thread::spawn(source_now_ns);
        thread::sleep(Duration::from_millis(15));
        assert!(!reader.is_finished());
        publish(14, initial, 2.5, true);
        assert_eq!(reader.join().unwrap(), initial);
        assert_eq!(next_wait(deadline), Some(FILE_POLL));

        let sleeper = tokio::spawn(sleep_until(deadline));
        let mut delayed = interval(duration, true);
        let ticker = tokio::spawn(async move { delayed.tick().await });
        let expiring = tokio::spawn(timeout(deadline, std::future::pending::<()>()));
        tokio::time::sleep(Duration::from_millis(35)).await;
        assert!(!sleeper.is_finished() && !ticker.is_finished() && !expiring.is_finished());
        publish(16, initial + 4_000_000, 2.5, false);
        assert_eq!(now.elapsed(), duration);
        assert_eq!(source_now_ns(), initial + 4_000_000);
        assert!(next_wait(deadline).is_none());
        sleeper.await.unwrap();
        ticker.await.unwrap();
        assert!(expiring.await.unwrap().is_err());
    }
}
