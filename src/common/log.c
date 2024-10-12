/*------------------------------------------------------------------------------
 * log.c   : logger functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "log.h"

#if OUTPUT_LOG
static FILE *fp_trace=NULL;         /* file pointer of trace */
static char file_trace[1024]={0};   /* trace file */
static int level_trace=0;           /* level of trace */
static lock_t lock_trace;           /* lock for trace */

/* close trace output----------------------------------------------------------*/
extern void log_trace_close(void)
{
    if (fp_trace&&fp_trace!=stderr) fclose(fp_trace);
    fp_trace=NULL;
    file_trace[0]='\0';
}
/* open trace output-----------------------------------------------------------
 * args:  char* file    trace file path
 * return: none
 * ---------------------------------------------------------------------------*/
extern void log_trace_open(const char *file)
{
    gtime_t time=timeget();
    char path[1024]={0};

    reppath(file,path,time,"","");
    if (!*path||!(fp_trace=fopen(path,"w"))) fp_trace=stderr;
    strcpy(file_trace,path);
    initlock(&lock_trace);
}
/* set trace level-------------------------------------------------------------
 * args:  int level  trace level:
 *                   1: error, 2: warning, 3: info
 * return: none
 * ---------------------------------------------------------------------------*/
extern void log_set_level(int level)
{
    level_trace=level;
}
extern void log_trace(int level, const char *format, ...)
{
    char buff[4096]={0};
    va_list ap;
    gtime_t t_curr;

    if (level>level_trace) {
        return;
    }
    lock(&lock_trace);

    va_start(ap,format); vsprintf(buff,format,ap); va_end(ap);
    t_curr=timeget();

    char tbuf[64];
    time2str(t_curr,tbuf,3);

    if      (level==3) fprintf(fp_trace,"%s %s: ",tbuf,"[INFO]");
    else if (level==2) fprintf(fp_trace,"%s %s: ",tbuf,"[WARN]");
    else if (level==1) fprintf(fp_trace,"%s %s: ",tbuf,"[ERRO]");

    if (fp_trace) {
        fprintf(fp_trace,"%s",buff);
        fflush(fp_trace);
    }
    unlock(&lock_trace);
}
extern void log_traceobs(int level, const obsd_t *obs, int n)
{
    char buff[4096]={0};
    char str[64],id[16],*p=buff;
    int i;

    if (level>level_trace) {
        return;
    }
    log_trace(3,"observation: n=%d\n",n);

    lock(&lock_trace);
    for (i=0;i<n;i++) {
        time2str(obs[i].time,str,3);
        satno2id(obs[i].sat,id);
        char rcvbuf[32];

        sprintf(rcvbuf,"rcv%d",obs[i].rcv);

        sprintf(p,"  (%2d) %s %-3s %8s %13.5f %13.5f %13.5f %13.5f %2d %2d %2d %2d %6.3f %6.3f\n",
                i+1,str,id,rcvbuf,obs[i].L[0],obs[i].L[1],obs[i].P[0],
                obs[i].P[1],obs[i].LLI[0],obs[i].LLI[1],obs[i].code[0],
                obs[i].code[1],obs[i].SNR[0]*0.25,obs[i].SNR[1]*0.25);

        if (level>level_trace) continue;

        if (fp_trace) {
            fprintf(fp_trace,"%s",p);
            fflush(fp_trace);
        }
    }
    unlock(&lock_trace);
}
extern void log_tracemat(int level, const double *A, int n, int m, int p, int q)
{
    if (!fp_trace) return;

    if (level>level_trace) {
        return;
    }
    lock(&lock_trace);
    matfprint(A,n,m,p,q,fp_trace); fflush(fp_trace);
    unlock(&lock_trace);
}
#else
extern void log_trace_open(const char *file) {}
extern void log_trace_close(void) {}
extern void log_set_level(int level) {}
extern void log_trace(int level, const char *format, ...) {}
extern void log_traceobs(int level, const obsd_t *obs, int n) {}
extern void log_tracemat(int level, const double *A, int n, int m, int p, int q) {}
#endif