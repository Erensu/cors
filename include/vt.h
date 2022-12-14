/*------------------------------------------------------------------------------
 * vt.h : header file for virtual console
 *
 *
 * version : $Revision:$ $Date:$
 * history : 2015/01/11 1.0  separated from rtkrcv.c
 *-----------------------------------------------------------------------------*/
#ifndef VT_H
#define VT_H

#include "rtklib.h"

#define MAXBUFF     4096                /* size of line buffer */
#define MAXHIST     256                 /* size of history buffer */

/* type definitions ----------------------------------------------------------*/
typedef struct vt_tag {                 /* virtual console type */
    int state;                          /* state(0:close,1:open) */
    int type;                           /* type (0:dev,1:telnet) */
    int in,out;                         /* input/output file descriptor */
    int n,nesc;                         /* number of line buffer/escape */
    int cur;                            /* cursor position */
    int cur_h;                          /* current history */
    int brk;                            /* break status */
    int blind;                          /* blind inpu mode */
#ifndef WIN32
    struct termios tio;                 /* original terminal attribute */
#endif
    char buff[MAXBUFF];                 /* line buffer */
    char esc[8];                        /* escape buffer */
    char *hist[MAXHIST];                /* history buffer */
    FILE *logfp;                        /* log file pointer */
} vt_t;

/* function prototypes -------------------------------------------------------*/
EXPORT vt_t *vt_open(const char *dev);
EXPORT void vt_close(vt_t *vt);
EXPORT int vt_getc(vt_t *vt, char *c);
EXPORT int vt_gets(vt_t *vt, char *buff, int n);
EXPORT int vt_putc(vt_t *vt, char c);
EXPORT int vt_puts(vt_t *vt, const char *buff);
EXPORT int vt_printf(vt_t *vt, const char *format, ...);
EXPORT int vt_chkbrk(vt_t *vt);
EXPORT int vt_openlog(vt_t *vt, const char *file);
EXPORT void vt_closelog(vt_t *vt);

#endif /* VT_H */
