// Minimal swaglog stub for the Jetson bring-up.
// Real swaglog.cc drags in zmq + json11 + more just for cloud logging, which
// isn't required to run controlsd locally. Provide the C++-mangled symbols the
// rest of the code links against, as no-ops. Signatures MUST match swaglog.h
// exactly so the mangled names line up (these are NOT extern "C").
#include <cstdarg>

void cloudlog_e(int levelnum, const char* filename, int lineno,
                const char* func, const char* fmt, ...) {
  (void)levelnum; (void)filename; (void)lineno; (void)func; (void)fmt;
}

void cloudlog_te(int levelnum, const char* filename, int lineno,
                 const char* func, const char* fmt, ...) {
  (void)levelnum; (void)filename; (void)lineno; (void)func; (void)fmt;
}

void cloudlog_te(int levelnum, const char* filename, int lineno,
                 const char* func, const char* fmt, va_list args) {
  (void)levelnum; (void)filename; (void)lineno; (void)func; (void)fmt; (void)args;
}
