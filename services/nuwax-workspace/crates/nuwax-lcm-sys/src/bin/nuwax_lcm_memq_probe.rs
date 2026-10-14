use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_void};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;

use nuwax_lcm_sys::{
    lcm_create, lcm_destroy, lcm_handle_timeout, lcm_publish, lcm_recv_buf_t, lcm_subscribe,
    lcm_subscription_set_queue_capacity, lcm_subscription_t, lcm_t, lcm_unsubscribe,
};

const PROVIDER: &str = "memq://";
const CHANNEL: &str = "NUWAX_LCM_SYS_I3_PROBE";
const PAYLOAD: &[u8] = b"nuwax-lcm-sys memq lifecycle probe";

struct LcmContext(*mut lcm_t);

impl LcmContext {
    fn create(provider: &CString) -> Result<Self, String> {
        let raw = unsafe { lcm_create(provider.as_ptr()) };
        if raw.is_null() {
            Err(format!("lcm_create({PROVIDER:?}) returned null"))
        } else {
            Ok(Self(raw))
        }
    }
}

impl Drop for LcmContext {
    fn drop(&mut self) {
        unsafe { lcm_destroy(self.0) };
    }
}

struct Subscription {
    lcm: *mut lcm_t,
    raw: *mut lcm_subscription_t,
}

impl Subscription {
    fn unsubscribe(&mut self) -> Result<(), String> {
        let raw = std::mem::replace(&mut self.raw, ptr::null_mut());
        let status = unsafe { lcm_unsubscribe(self.lcm, raw) };
        if status != 0 {
            return Err(format!("lcm_unsubscribe returned {status}"));
        }
        Ok(())
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        if !self.raw.is_null() {
            unsafe {
                lcm_unsubscribe(self.lcm, self.raw);
            }
        }
    }
}

#[derive(Default)]
struct CallbackState {
    callbacks: usize,
    invalid_message: bool,
}

unsafe extern "C" fn probe_callback(
    rbuf: *const lcm_recv_buf_t,
    channel: *const c_char,
    user: *mut c_void,
) {
    if user.is_null() {
        return;
    }

    let state = unsafe { &mut *user.cast::<CallbackState>() };
    let result = catch_unwind(AssertUnwindSafe(|| validate_message(rbuf, channel)));
    match result {
        Ok(true) => state.callbacks = state.callbacks.saturating_add(1),
        Ok(false) | Err(_) => state.invalid_message = true,
    }
}

fn validate_message(rbuf: *const lcm_recv_buf_t, channel: *const c_char) -> bool {
    if rbuf.is_null() || channel.is_null() {
        return false;
    }

    let received_channel = unsafe { CStr::from_ptr(channel) };
    if received_channel.to_bytes() != CHANNEL.as_bytes() {
        return false;
    }

    let rbuf = unsafe { &*rbuf };
    let data_len = rbuf.data_size as usize;
    if rbuf.data.is_null() {
        return false;
    }
    let payload = unsafe { std::slice::from_raw_parts(rbuf.data.cast::<u8>(), data_len) };
    payload == PAYLOAD
}

fn publish(lcm: *mut lcm_t, channel: &CString) -> Result<(), String> {
    let payload_len = u32::try_from(PAYLOAD.len()).map_err(|error| error.to_string())?;
    let status = unsafe {
        lcm_publish(
            lcm,
            channel.as_ptr(),
            PAYLOAD.as_ptr().cast::<c_void>(),
            payload_len,
        )
    };
    if status == 0 {
        Ok(())
    } else {
        Err(format!("lcm_publish returned {status}"))
    }
}

fn validate_post_unsubscribe_handle(status: i32) -> Result<(), String> {
    if status < 0 {
        Err(format!(
            "lcm_handle_timeout after unsubscribe returned {status}"
        ))
    } else {
        Ok(())
    }
}

fn run() -> Result<(), String> {
    let provider = CString::new(PROVIDER).map_err(|error| error.to_string())?;
    let channel = CString::new(CHANNEL).map_err(|error| error.to_string())?;
    let context = LcmContext::create(&provider)?;
    let mut callback_state = Box::<CallbackState>::default();

    let subscription_raw = unsafe {
        lcm_subscribe(
            context.0,
            channel.as_ptr(),
            Some(probe_callback),
            (&mut *callback_state as *mut CallbackState).cast::<c_void>(),
        )
    };
    if subscription_raw.is_null() {
        return Err("lcm_subscribe returned null".to_owned());
    }
    let mut subscription = Subscription {
        lcm: context.0,
        raw: subscription_raw,
    };

    let queue_status = unsafe { lcm_subscription_set_queue_capacity(subscription.raw, 4) };
    if queue_status != 0 {
        return Err(format!(
            "lcm_subscription_set_queue_capacity returned {queue_status}"
        ));
    }

    publish(context.0, &channel)?;
    let handle_status = unsafe { lcm_handle_timeout(context.0, 1_000) };
    if handle_status <= 0 {
        return Err(format!(
            "lcm_handle_timeout did not dispatch the published message: {handle_status}"
        ));
    }
    if callback_state.invalid_message || callback_state.callbacks != 1 {
        return Err(format!(
            "callback validation failed: callbacks={}, invalid_message={}",
            callback_state.callbacks, callback_state.invalid_message
        ));
    }

    subscription.unsubscribe()?;
    publish(context.0, &channel)?;
    let post_unsubscribe_status = unsafe { lcm_handle_timeout(context.0, 50) };
    validate_post_unsubscribe_handle(post_unsubscribe_status)?;
    if callback_state.invalid_message || callback_state.callbacks != 1 {
        return Err(format!(
            "callback ran after unsubscribe: callbacks={}, invalid_message={}",
            callback_state.callbacks, callback_state.invalid_message
        ));
    }

    println!("memq lifecycle probe passed: callbacks=1");
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("memq lifecycle probe failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::validate_post_unsubscribe_handle;

    #[test]
    fn post_unsubscribe_handle_rejects_errors_but_accepts_timeout() {
        assert!(validate_post_unsubscribe_handle(-1).is_err());
        assert!(validate_post_unsubscribe_handle(0).is_ok());
    }
}
