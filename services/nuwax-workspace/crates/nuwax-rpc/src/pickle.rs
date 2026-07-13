use crate::{RpcError, RpcRequest, RpcResponse, RpcResult};
use serde::de::DeserializeOwned;
use serde_json::{Map, Number, Value};

const PROTO: u8 = 0x80;
const STOP: u8 = b'.';
const MARK: u8 = b'(';
const NONE: u8 = b'N';
const BININT: u8 = b'J';
const BININT1: u8 = b'K';
const BININT2: u8 = b'M';
const LONG1: u8 = 0x8a;
const LONG4: u8 = 0x8b;
const BINFLOAT: u8 = b'G';
const EMPTY_LIST: u8 = b']';
const APPEND: u8 = b'a';
const APPENDS: u8 = b'e';
const LIST: u8 = b'l';
const EMPTY_TUPLE: u8 = b')';
const TUPLE: u8 = b't';
const TUPLE1: u8 = 0x85;
const TUPLE2: u8 = 0x86;
const TUPLE3: u8 = 0x87;
const EMPTY_DICT: u8 = b'}';
const DICT: u8 = b'd';
const SETITEM: u8 = b's';
const SETITEMS: u8 = b'u';
const SHORT_BINUNICODE: u8 = 0x8c;
const BINUNICODE: u8 = b'X';
const BINUNICODE8: u8 = 0x8d;
const NEWTRUE: u8 = 0x88;
const NEWFALSE: u8 = 0x89;
const FRAME: u8 = 0x95;
const MEMOIZE: u8 = 0x94;
const BINPUT: u8 = b'q';
const LONG_BINPUT: u8 = b'r';
const BINGET: u8 = b'h';
const LONG_BINGET: u8 = b'j';
const POP: u8 = b'0';
const POP_MARK: u8 = b'1';
const DUP: u8 = b'2';
const MAX_MEMO_ENTRIES: usize = 65_536;

pub(crate) fn encode_request(request: &RpcRequest) -> RpcResult<Vec<u8>> {
    let positional = Value::Array(request.args.0.clone());
    let keyword = Value::Object(
        request
            .args
            .1
            .iter()
            .map(|(key, value)| (key.clone(), value.clone()))
            .collect(),
    );
    let id = request.id.map_or(Value::Null, Value::from);

    encode_document(|output| {
        begin_dict(output);
        encode_string("name", output)?;
        encode_string(&request.name, output)?;
        encode_string("args", output)?;
        encode_value(&positional, output)?;
        encode_value(&keyword, output)?;
        output.push(TUPLE2);
        encode_string("id", output)?;
        encode_value(&id, output)?;
        end_dict(output);
        Ok(())
    })
}

pub(crate) fn encode_response(response: &RpcResponse) -> RpcResult<Vec<u8>> {
    encode_document(|output| {
        begin_dict(output);
        encode_string("id", output)?;
        encode_value(&response.id.map_or(Value::Null, Value::from), output)?;
        if let Some(result) = &response.res {
            encode_string("res", output)?;
            encode_value(result, output)?;
        }
        if let Some(exception) = &response.exception {
            encode_string("exception", output)?;
            encode_value(exception, output)?;
        }
        end_dict(output);
        Ok(())
    })
}

pub(crate) fn decode_request(payload: &[u8]) -> RpcResult<RpcRequest> {
    decode_document(payload)
}

pub(crate) fn decode_response(payload: &[u8]) -> RpcResult<RpcResponse> {
    decode_document(payload)
}

fn encode_document(encode_root: impl FnOnce(&mut Vec<u8>) -> RpcResult<()>) -> RpcResult<Vec<u8>> {
    let mut output = vec![PROTO, 4];
    encode_root(&mut output)?;
    output.push(STOP);
    Ok(output)
}

fn begin_dict(output: &mut Vec<u8>) {
    output.extend_from_slice(&[EMPTY_DICT, MARK]);
}

fn end_dict(output: &mut Vec<u8>) {
    output.push(SETITEMS);
}

fn encode_value(value: &Value, output: &mut Vec<u8>) -> RpcResult<()> {
    match value {
        Value::Null => output.push(NONE),
        Value::Bool(true) => output.push(NEWTRUE),
        Value::Bool(false) => output.push(NEWFALSE),
        Value::Number(number) => encode_number(number, output)?,
        Value::String(text) => encode_string(text, output)?,
        Value::Array(values) => {
            output.push(EMPTY_LIST);
            if !values.is_empty() {
                output.push(MARK);
                for value in values {
                    encode_value(value, output)?;
                }
                output.push(APPENDS);
            }
        }
        Value::Object(values) => {
            output.push(EMPTY_DICT);
            if !values.is_empty() {
                output.push(MARK);
                for (key, value) in values {
                    encode_string(key, output)?;
                    encode_value(value, output)?;
                }
                output.push(SETITEMS);
            }
        }
    }
    Ok(())
}

fn encode_number(number: &Number, output: &mut Vec<u8>) -> RpcResult<()> {
    if let Some(value) = number.as_i64() {
        encode_i64(value, output);
        return Ok(());
    }
    if let Some(value) = number.as_u64() {
        encode_u64(value, output);
        return Ok(());
    }
    let value = number
        .as_f64()
        .ok_or_else(|| RpcError::Serialize(format!("unsupported JSON number: {number}")))?;
    output.push(BINFLOAT);
    output.extend_from_slice(&value.to_be_bytes());
    Ok(())
}

fn encode_i64(value: i64, output: &mut Vec<u8>) {
    if value >= 0 {
        encode_u64(value as u64, output);
    } else if let Ok(value) = i32::try_from(value) {
        output.push(BININT);
        output.extend_from_slice(&value.to_le_bytes());
    } else {
        let mut bytes = value.to_le_bytes().to_vec();
        while bytes.len() > 1 && bytes.last() == Some(&0xff) && bytes[bytes.len() - 2] & 0x80 != 0 {
            bytes.pop();
        }
        encode_long(&bytes, output);
    }
}

fn encode_u64(value: u64, output: &mut Vec<u8>) {
    if let Ok(value) = u8::try_from(value) {
        output.extend_from_slice(&[BININT1, value]);
    } else if let Ok(value) = u16::try_from(value) {
        output.push(BININT2);
        output.extend_from_slice(&value.to_le_bytes());
    } else if let Ok(value) = i32::try_from(value) {
        output.push(BININT);
        output.extend_from_slice(&value.to_le_bytes());
    } else {
        let mut bytes = value.to_le_bytes().to_vec();
        while bytes.last() == Some(&0) {
            bytes.pop();
        }
        if bytes.last().is_some_and(|byte| byte & 0x80 != 0) {
            bytes.push(0);
        }
        encode_long(&bytes, output);
    }
}

fn encode_long(bytes: &[u8], output: &mut Vec<u8>) {
    if let Ok(length) = u8::try_from(bytes.len()) {
        output.extend_from_slice(&[LONG1, length]);
    } else {
        output.push(LONG4);
        output.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
    }
    output.extend_from_slice(bytes);
}

fn encode_string(text: &str, output: &mut Vec<u8>) -> RpcResult<()> {
    let bytes = text.as_bytes();
    if let Ok(length) = u8::try_from(bytes.len()) {
        output.extend_from_slice(&[SHORT_BINUNICODE, length]);
    } else {
        let length = u32::try_from(bytes.len())
            .map_err(|_| RpcError::Serialize("string exceeds pickle BINUNICODE limit".into()))?;
        output.push(BINUNICODE);
        output.extend_from_slice(&length.to_le_bytes());
    }
    output.extend_from_slice(bytes);
    Ok(())
}

fn decode_document<T: DeserializeOwned>(payload: &[u8]) -> RpcResult<T> {
    let value = PickleParser::new(payload).parse()?;
    serde_json::from_value(value).map_err(|error| RpcError::Deserialize(error.to_string()))
}

#[derive(Debug, Clone)]
enum StackValue {
    Json { value: Value, identity: Option<u64> },
    Tuple { values: Vec<Value>, identity: u64 },
    Mark,
}

impl StackValue {
    fn scalar(value: Value) -> Self {
        Self::Json {
            value,
            identity: None,
        }
    }

    fn container(value: Value, identity: u64) -> Self {
        Self::Json {
            value,
            identity: Some(identity),
        }
    }

    fn identity(&self) -> Option<u64> {
        match self {
            Self::Json { identity, .. } => *identity,
            Self::Tuple { identity, .. } => Some(*identity),
            Self::Mark => None,
        }
    }

    fn into_json(self) -> RpcResult<Value> {
        match self {
            Self::Json { value, .. } => Ok(value),
            Self::Tuple { values, .. } => Ok(Value::Array(values)),
            Self::Mark => Err(RpcError::Deserialize("unexpected pickle MARK".into())),
        }
    }
}

struct PickleParser<'a> {
    payload: &'a [u8],
    position: usize,
    stack: Vec<StackValue>,
    memo: Vec<Option<StackValue>>,
    next_identity: u64,
}

impl<'a> PickleParser<'a> {
    fn new(payload: &'a [u8]) -> Self {
        Self {
            payload,
            position: 0,
            stack: Vec::new(),
            memo: Vec::new(),
            next_identity: 1,
        }
    }

    fn parse(mut self) -> RpcResult<Value> {
        loop {
            let opcode = self.read_u8()?;
            match opcode {
                PROTO => {
                    let protocol = self.read_u8()?;
                    if protocol > 5 {
                        return Err(self.error(format!("unsupported pickle protocol {protocol}")));
                    }
                }
                FRAME => {
                    let frame_length = self.read_u64_le()?;
                    let remaining = self.payload.len().saturating_sub(self.position) as u64;
                    if frame_length > remaining {
                        return Err(self.error("truncated pickle FRAME"));
                    }
                }
                STOP => {
                    let value = self
                        .stack
                        .pop()
                        .ok_or_else(|| self.error("empty stack at STOP"))?
                        .into_json()?;
                    if !self.stack.is_empty() {
                        return Err(self.error("extra values remain at STOP"));
                    }
                    if self.position != self.payload.len() {
                        return Err(self.error("trailing bytes after STOP"));
                    }
                    return Ok(value);
                }
                MARK => self.stack.push(StackValue::Mark),
                NONE => self.stack.push(StackValue::scalar(Value::Null)),
                NEWTRUE => self.stack.push(StackValue::scalar(Value::Bool(true))),
                NEWFALSE => self.stack.push(StackValue::scalar(Value::Bool(false))),
                BININT1 => {
                    let value = u64::from(self.read_u8()?);
                    self.push_number(Number::from(value));
                }
                BININT2 => {
                    let value = u64::from(self.read_u16_le()?);
                    self.push_number(Number::from(value));
                }
                BININT => {
                    let value = i64::from(self.read_i32_le()?);
                    self.push_number(Number::from(value));
                }
                LONG1 => {
                    let length = usize::from(self.read_u8()?);
                    let bytes = self.take(length)?;
                    let number = decode_long(bytes)?;
                    self.push_number(number);
                }
                LONG4 => {
                    let length = self.read_u32_le()? as usize;
                    let bytes = self.take(length)?;
                    let number = decode_long(bytes)?;
                    self.push_number(number);
                }
                BINFLOAT => {
                    let value = f64::from_be_bytes(self.take_array()?);
                    let number = Number::from_f64(value)
                        .ok_or_else(|| self.error("NaN/Infinity is outside the JSON RPC domain"))?;
                    self.push_number(number);
                }
                SHORT_BINUNICODE => {
                    let length = usize::from(self.read_u8()?);
                    let text = self.read_utf8(length)?;
                    self.stack.push(StackValue::scalar(Value::String(text)));
                }
                BINUNICODE => {
                    let length = self.read_u32_le()? as usize;
                    let text = self.read_utf8(length)?;
                    self.stack.push(StackValue::scalar(Value::String(text)));
                }
                BINUNICODE8 => {
                    let length = usize::try_from(self.read_u64_le()?)
                        .map_err(|_| self.error("BINUNICODE8 length overflows usize"))?;
                    let text = self.read_utf8(length)?;
                    self.stack.push(StackValue::scalar(Value::String(text)));
                }
                EMPTY_LIST => self.push_container(Value::Array(Vec::new())),
                APPEND => self.append_one()?,
                APPENDS => self.append_many()?,
                LIST => self.build_list()?,
                EMPTY_TUPLE => self.push_tuple(Vec::new()),
                TUPLE => self.build_tuple()?,
                TUPLE1 => self.build_fixed_tuple(1)?,
                TUPLE2 => self.build_fixed_tuple(2)?,
                TUPLE3 => self.build_fixed_tuple(3)?,
                EMPTY_DICT => self.push_container(Value::Object(Map::new())),
                SETITEM => self.set_one_item()?,
                SETITEMS => self.set_many_items()?,
                DICT => self.build_dict()?,
                MEMOIZE => self.memoize_top()?,
                BINPUT => {
                    let index = usize::from(self.read_u8()?);
                    self.memoize_at(index)?;
                }
                LONG_BINPUT => {
                    let index = self.read_u32_le()? as usize;
                    self.memoize_at(index)?;
                }
                BINGET => {
                    let index = usize::from(self.read_u8()?);
                    self.push_memo(index)?;
                }
                LONG_BINGET => {
                    let index = self.read_u32_le()? as usize;
                    self.push_memo(index)?;
                }
                POP => {
                    self.stack
                        .pop()
                        .ok_or_else(|| self.error("POP on empty stack"))?;
                }
                POP_MARK => {
                    let mark = self.mark_position()?;
                    self.stack.truncate(mark);
                }
                DUP => {
                    let value = self
                        .stack
                        .last()
                        .cloned()
                        .ok_or_else(|| self.error("DUP on empty stack"))?;
                    self.stack.push(value);
                }
                _ => {
                    return Err(self.error(format!(
                        "unsupported or unsafe pickle opcode 0x{opcode:02x}"
                    )));
                }
            }
        }
    }

    fn push_number(&mut self, number: Number) {
        self.stack.push(StackValue::scalar(Value::Number(number)));
    }

    fn push_container(&mut self, value: Value) {
        let identity = self.allocate_identity();
        self.stack.push(StackValue::container(value, identity));
    }

    fn push_tuple(&mut self, values: Vec<Value>) {
        let identity = self.allocate_identity();
        self.stack.push(StackValue::Tuple { values, identity });
    }

    fn allocate_identity(&mut self) -> u64 {
        let identity = self.next_identity;
        self.next_identity = self.next_identity.wrapping_add(1);
        identity
    }

    fn append_one(&mut self) -> RpcResult<()> {
        let value = self.pop_json()?;
        let target = self
            .stack
            .last_mut()
            .ok_or_else(|| RpcError::Deserialize("APPEND on empty stack".into()))?;
        let identity = match target {
            StackValue::Json {
                value: Value::Array(values),
                identity,
            } => {
                values.push(value);
                *identity
            }
            _ => return Err(self.error("APPEND target is not a list")),
        };
        let snapshot = target.clone();
        if let Some(identity) = identity {
            self.sync_memo(identity, snapshot);
        }
        Ok(())
    }

    fn append_many(&mut self) -> RpcResult<()> {
        let mark = self.mark_position()?;
        let values = self.take_values_after_mark(mark)?;
        let target = self
            .stack
            .last_mut()
            .ok_or_else(|| RpcError::Deserialize("APPENDS on empty stack".into()))?;
        let identity = match target {
            StackValue::Json {
                value: Value::Array(target_values),
                identity,
            } => {
                target_values.extend(values);
                *identity
            }
            _ => return Err(self.error("APPENDS target is not a list")),
        };
        let snapshot = target.clone();
        if let Some(identity) = identity {
            self.sync_memo(identity, snapshot);
        }
        Ok(())
    }

    fn build_list(&mut self) -> RpcResult<()> {
        let mark = self.mark_position()?;
        let values = self.take_values_after_mark(mark)?;
        self.push_container(Value::Array(values));
        Ok(())
    }

    fn build_tuple(&mut self) -> RpcResult<()> {
        let mark = self.mark_position()?;
        let values = self.take_values_after_mark(mark)?;
        self.push_tuple(values);
        Ok(())
    }

    fn build_fixed_tuple(&mut self, count: usize) -> RpcResult<()> {
        if self.stack.len() < count {
            return Err(self.error("tuple opcode underflow"));
        }
        let values = self.stack.split_off(self.stack.len() - count);
        let values = values
            .into_iter()
            .map(StackValue::into_json)
            .collect::<RpcResult<Vec<_>>>()?;
        self.push_tuple(values);
        Ok(())
    }

    fn set_one_item(&mut self) -> RpcResult<()> {
        let value = self.pop_json()?;
        let key = self.pop_string()?;
        let target = self
            .stack
            .last_mut()
            .ok_or_else(|| RpcError::Deserialize("SETITEM on empty stack".into()))?;
        let identity = match target {
            StackValue::Json {
                value: Value::Object(target_values),
                identity,
            } => {
                target_values.insert(key, value);
                *identity
            }
            _ => return Err(self.error("SETITEM target is not a dict")),
        };
        let snapshot = target.clone();
        if let Some(identity) = identity {
            self.sync_memo(identity, snapshot);
        }
        Ok(())
    }

    fn set_many_items(&mut self) -> RpcResult<()> {
        let mark = self.mark_position()?;
        let entries = self.take_stack_after_mark(mark)?;
        self.insert_entries(entries)
    }

    fn build_dict(&mut self) -> RpcResult<()> {
        let mark = self.mark_position()?;
        let entries = self.take_stack_after_mark(mark)?;
        let mut object = Map::new();
        insert_entries_into(&mut object, entries)?;
        self.push_container(Value::Object(object));
        Ok(())
    }

    fn insert_entries(&mut self, entries: Vec<StackValue>) -> RpcResult<()> {
        let target = self
            .stack
            .last_mut()
            .ok_or_else(|| RpcError::Deserialize("SETITEMS on empty stack".into()))?;
        let identity = match target {
            StackValue::Json {
                value: Value::Object(target_values),
                identity,
            } => {
                insert_entries_into(target_values, entries)?;
                *identity
            }
            _ => return Err(self.error("SETITEMS target is not a dict")),
        };
        let snapshot = target.clone();
        if let Some(identity) = identity {
            self.sync_memo(identity, snapshot);
        }
        Ok(())
    }

    fn sync_memo(&mut self, identity: u64, snapshot: StackValue) {
        for memoized in self.memo.iter_mut().flatten() {
            if memoized.identity() == Some(identity) {
                *memoized = snapshot.clone();
            }
        }
    }

    fn memoize_top(&mut self) -> RpcResult<()> {
        if self.memo.len() >= MAX_MEMO_ENTRIES {
            return Err(self.error("pickle memo entry limit exceeded"));
        }
        let value = self
            .stack
            .last()
            .cloned()
            .ok_or_else(|| self.error("MEMOIZE on empty stack"))?;
        self.memo.push(Some(value));
        Ok(())
    }

    fn memoize_at(&mut self, index: usize) -> RpcResult<()> {
        if index >= MAX_MEMO_ENTRIES {
            return Err(self.error("pickle memo index limit exceeded"));
        }
        let value = self
            .stack
            .last()
            .cloned()
            .ok_or_else(|| self.error("BINPUT on empty stack"))?;
        if self.memo.len() <= index {
            self.memo.resize(index + 1, None);
        }
        self.memo[index] = Some(value);
        Ok(())
    }

    fn push_memo(&mut self, index: usize) -> RpcResult<()> {
        let value = self
            .memo
            .get(index)
            .and_then(Clone::clone)
            .ok_or_else(|| self.error(format!("memo index {index} is not defined")))?;
        self.stack.push(value);
        Ok(())
    }

    fn mark_position(&self) -> RpcResult<usize> {
        self.stack
            .iter()
            .rposition(|value| matches!(value, StackValue::Mark))
            .ok_or_else(|| self.error("container opcode without MARK"))
    }

    fn take_values_after_mark(&mut self, mark: usize) -> RpcResult<Vec<Value>> {
        self.take_stack_after_mark(mark)?
            .into_iter()
            .map(StackValue::into_json)
            .collect()
    }

    fn take_stack_after_mark(&mut self, mark: usize) -> RpcResult<Vec<StackValue>> {
        let values = self.stack.split_off(mark + 1);
        match self.stack.pop() {
            Some(StackValue::Mark) => Ok(values),
            _ => Err(self.error("internal MARK mismatch")),
        }
    }

    fn pop_json(&mut self) -> RpcResult<Value> {
        self.stack
            .pop()
            .ok_or_else(|| self.error("pickle stack underflow"))?
            .into_json()
    }

    fn pop_string(&mut self) -> RpcResult<String> {
        match self.pop_json()? {
            Value::String(value) => Ok(value),
            _ => Err(self.error("dict key is not a string")),
        }
    }

    fn read_u8(&mut self) -> RpcResult<u8> {
        Ok(self.take(1)?[0])
    }

    fn read_u16_le(&mut self) -> RpcResult<u16> {
        Ok(u16::from_le_bytes(self.take_array()?))
    }

    fn read_u32_le(&mut self) -> RpcResult<u32> {
        Ok(u32::from_le_bytes(self.take_array()?))
    }

    fn read_i32_le(&mut self) -> RpcResult<i32> {
        Ok(i32::from_le_bytes(self.take_array()?))
    }

    fn read_u64_le(&mut self) -> RpcResult<u64> {
        Ok(u64::from_le_bytes(self.take_array()?))
    }

    fn take_array<const N: usize>(&mut self) -> RpcResult<[u8; N]> {
        self.take(N)?
            .try_into()
            .map_err(|_| self.error("invalid fixed-width pickle field"))
    }

    fn read_utf8(&mut self, length: usize) -> RpcResult<String> {
        let bytes = self.take(length)?;
        std::str::from_utf8(bytes)
            .map(str::to_owned)
            .map_err(|error| RpcError::Deserialize(format!("pickle string is not UTF-8: {error}")))
    }

    fn take(&mut self, length: usize) -> RpcResult<&'a [u8]> {
        let end = self
            .position
            .checked_add(length)
            .ok_or_else(|| self.error("pickle length overflow"))?;
        if end > self.payload.len() {
            return Err(self.error("truncated pickle payload"));
        }
        let bytes = &self.payload[self.position..end];
        self.position = end;
        Ok(bytes)
    }

    fn error(&self, message: impl Into<String>) -> RpcError {
        RpcError::Deserialize(format!("{} at byte {}", message.into(), self.position))
    }
}

fn insert_entries_into(target: &mut Map<String, Value>, entries: Vec<StackValue>) -> RpcResult<()> {
    if entries.len() % 2 != 0 {
        return Err(RpcError::Deserialize(
            "pickle dict has an odd number of stack entries".into(),
        ));
    }
    let mut entries = entries.into_iter();
    while let Some(key) = entries.next() {
        let key = match key.into_json()? {
            Value::String(key) => key,
            _ => {
                return Err(RpcError::Deserialize(
                    "pickle dict key is not a string".into(),
                ))
            }
        };
        let value = entries
            .next()
            .expect("even entry count checked above")
            .into_json()?;
        target.insert(key, value);
    }
    Ok(())
}

fn decode_long(bytes: &[u8]) -> RpcResult<Number> {
    if bytes.is_empty() {
        return Ok(Number::from(0));
    }
    let negative = bytes.last().is_some_and(|byte| byte & 0x80 != 0);
    if negative {
        if bytes.len() > 9 || (bytes.len() == 9 && bytes[8] != 0xff) {
            return Err(RpcError::Deserialize(
                "pickle negative LONG exceeds i64 RPC domain".into(),
            ));
        }
        let mut raw = [0xff_u8; 8];
        let copy_length = bytes.len().min(8);
        raw[..copy_length].copy_from_slice(&bytes[..copy_length]);
        Ok(Number::from(i64::from_le_bytes(raw)))
    } else {
        if bytes.len() > 9 || (bytes.len() == 9 && bytes[8] != 0) {
            return Err(RpcError::Deserialize(
                "pickle positive LONG exceeds u64 RPC domain".into(),
            ));
        }
        let mut raw = [0_u8; 8];
        let copy_length = bytes.len().min(8);
        raw[..copy_length].copy_from_slice(&bytes[..copy_length]);
        Ok(Number::from(u64::from_le_bytes(raw)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::process::Command;

    fn decode_hex(hex: &str) -> Vec<u8> {
        hex.as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                let pair = std::str::from_utf8(pair).unwrap();
                u8::from_str_radix(pair, 16).unwrap()
            })
            .collect()
    }

    fn fixture_request() -> RpcRequest {
        RpcRequest {
            name: "Echo/say".into(),
            args: (
                vec![
                    Value::from(42),
                    Value::from(3.5),
                    Value::Bool(true),
                    Value::Null,
                    serde_json::json!({"k": "v"}),
                ],
                HashMap::from([("flag".into(), Value::Bool(false))]),
            ),
            id: Some(1.0),
        }
    }

    #[test]
    fn decodes_python_protocol4_request_fixture() {
        let payload = decode_hex(
            "80049555000000000000007d94288c046e616d65948c084563686f2f736179948c0461726773945d94284b2a47400c000000000000884e7d948c016b948c01769473657d948c04666c616794897386948c02696494473ff0000000000000752e",
        );
        let request = decode_request(&payload).unwrap();
        let expected = fixture_request();
        assert_eq!(request.name, expected.name);
        assert_eq!(request.args, expected.args);
        assert_eq!(request.id, expected.id);
    }

    #[test]
    fn decodes_python_memoized_shared_container_by_value() {
        let payload = decode_hex(
            "8004953c000000000000007d94288c046e616d65948c06736861726564948c0461726773945d94285d94284b014b02656805657d9486948c02696494473ff0000000000000752e",
        );
        let request = decode_request(&payload).unwrap();
        assert_eq!(
            request.args.0,
            vec![serde_json::json!([1, 2]), serde_json::json!([1, 2])]
        );
    }

    #[test]
    fn decodes_python_protocol4_success_and_exception_fixtures() {
        let success = decode_hex(
            "8004952f000000000000007d94288c02696494473ff00000000000008c03726573947d948c026f6b9488738c09657863657074696f6e944e752e",
        );
        let success = decode_response(&success).unwrap();
        assert_eq!(success.id, Some(1.0));
        assert_eq!(success.res, Some(serde_json::json!({"ok": true})));
        assert!(success.exception.is_none());

        let exception = decode_hex(
            "8004954e000000000000007d94288c026964944740000000000000008c03726573944e8c09657863657074696f6e947d94288c0474797065948c0a56616c75654572726f72948c076d657373616765948c036261649475752e",
        );
        let exception = decode_response(&exception).unwrap();
        assert_eq!(exception.id, Some(2.0));
        assert!(exception.res.is_none());
        assert_eq!(exception.exception.unwrap()["message"], "bad");

        let original_exception_shape = decode_hex(
            "80049577000000000000007d94288c026964944740000000000000008c09657863657074696f6e947d94288c09747970655f6e616d65948c0a56616c75654572726f72948c0b747970655f6d6f64756c65948c086275696c74696e73948c0461726773948c03626164944b2a86948c0974726163656261636b948c0274629475752e",
        );
        let original_exception_shape = decode_response(&original_exception_shape).unwrap();
        let exception = original_exception_shape.exception.unwrap();
        assert_eq!(exception["type_name"], "ValueError");
        assert_eq!(exception["type_module"], "builtins");
        assert_eq!(exception["args"], serde_json::json!(["bad", 42]));
        assert_eq!(exception["traceback"], "tb");
    }

    #[test]
    fn rust_payloads_are_loadable_by_python_pickle() {
        let payload = encode_request(&fixture_request()).unwrap();
        let hex = payload
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let script = concat!(
            "import pickle,sys; ",
            "x=pickle.loads(bytes.fromhex(sys.argv[1])); ",
            "assert x['name']=='Echo/say'; ",
            "assert x['args'][0]==[42,3.5,True,None,{'k':'v'}]; ",
            "assert x['args'][1]=={'flag':False}; ",
            "assert x['id']==1.0"
        );
        let status = Command::new("python3")
            .args(["-c", script, &hex])
            .status()
            .expect("python3 is required by this Python/Rust workspace");
        assert!(status.success());

        let response = RpcResponse::error(
            Some(2.0),
            serde_json::json!({
                "type_name": "ValueError",
                "type_module": "builtins",
                "args": ["bad", 42],
                "traceback": "tb",
            }),
        );
        let hex = encode_response(&response)
            .unwrap()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let script = concat!(
            "import pickle,sys; ",
            "x=pickle.loads(bytes.fromhex(sys.argv[1])); ",
            "assert x['id']==2.0; ",
            "assert 'res' not in x; ",
            "assert x['exception']['type_name']=='ValueError'; ",
            "assert x['exception']['args']==['bad',42]"
        );
        let status = Command::new("python3")
            .args(["-c", script, &hex])
            .status()
            .expect("python3 is required by this Python/Rust workspace");
        assert!(status.success());
    }

    #[test]
    fn rejects_object_construction_opcodes() {
        let malicious = b"\x80\x04cbuiltins\neval\n.";
        let error = decode_request(malicious).unwrap_err();
        assert!(matches!(error, RpcError::Deserialize(_)));
    }

    #[test]
    fn rejects_oversized_memo_index_without_allocating_it() {
        let malicious = b"\x80\x04Nr\xff\xff\xff\xff.";
        let error = decode_request(malicious).unwrap_err();
        assert!(matches!(error, RpcError::Deserialize(_)));
    }

    #[test]
    fn integer_boundaries_round_trip() {
        let request = RpcRequest {
            name: "ints".into(),
            args: (
                vec![
                    Value::from(i64::MIN),
                    Value::from(i64::MAX),
                    Value::from(u64::MAX),
                ],
                HashMap::new(),
            ),
            id: Some(3.0),
        };
        let decoded = decode_request(&encode_request(&request).unwrap()).unwrap();
        assert_eq!(decoded.args, request.args);
    }
}
