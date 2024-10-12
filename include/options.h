/*------------------------------------------------------------------------------
 * options.h: header file of CORS options
 *
 * author   : sujinglan
 * version  : $Revision:$ $Date:$
 * history  : 2022/11/15 1.0  new
 *-----------------------------------------------------------------------------*/
#ifndef OPTIONS_H
#define OPTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cors.h"

EXPORT int cors_loadopts(cors_opt_t *opt, const char *file);
EXPORT int cors_saveopts(const char *file, const char *mode, const char *comment,
                         const opt_t *opts);
EXPORT opt_t cors_opts[];
EXPORT opt_t rtk_opts[];
EXPORT opt_t pnt_opts[];

EXPORT void reset_opts(void);
EXPORT void get_opts(prcopt_t *popt, solopt_t *sopt, filopt_t *fopt);
EXPORT void set_opts(const prcopt_t *prcopt, const solopt_t *solopt, const filopt_t *filopt);
EXPORT int load_opts(const char *file, opt_t *opts);

#ifdef __cplusplus
}
#endif
#endif