/*------------------------------------------------------------------------------
 * log.h  : header file of logger
 *
 * author : sujinglan
 * version  : $Revision:$ $Date:$
 * history  : 2022/11/15 1.0  new
 *-----------------------------------------------------------------------------*/
#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtklib.h"

#define OUTPUT_LOG  1

EXPORT void log_trace_open(const char *file);
EXPORT void log_trace_close(void);
EXPORT void log_set_level(int level);
EXPORT void log_trace(int level, const char *format, ...);
EXPORT void log_traceobs(int level, const obsd_t *obs, int n);
EXPORT void log_tracemat(int level, const double *A, int n, int m, int p, int q);

#ifdef __cplusplus
}
#endif
#endif