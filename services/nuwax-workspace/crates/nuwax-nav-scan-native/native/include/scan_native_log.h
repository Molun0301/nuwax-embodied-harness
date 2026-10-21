#pragma once
#include <cstdio>
#include <cstdarg>
#include <string>
// 原优化器候选失败保留诊断，不向上层逐行 JSON（对象序列化）协议写日志。
inline thread_local std::string scan_native_diagnostic;
inline void scanNativeDiagnostic(const char *format,...) {
  char buffer[512];
  va_list arguments;
  va_start(arguments,format);
  std::vsnprintf(buffer,sizeof(buffer),format,arguments);
  va_end(arguments);
  scan_native_diagnostic=buffer;
}
#define SCAN_WARN(...) scanNativeDiagnostic(__VA_ARGS__)
#define SCAN_ERROR(...) scanNativeDiagnostic(__VA_ARGS__)
