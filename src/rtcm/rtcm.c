/*------------------------------------------------------------------------------
 * rtcm.c : rtcm functions
 *
 * references :
 *     [1]  RTCM Recommended Standards for Differential GNSS (Global Navigation
 *          Satellite Systems) Service version 2.3, August 20, 2001
 *     [7]  RTCM Standard 10403.1 - Amendment 5, Differential GNSS (Global
 *          Navigation Satellite Systems) Services - version 3, July 1, 2011
 *     [10] RTCM Paper 059-2011-SC104-635 (draft Galileo and QZSS ssr messages)
 *     [15] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
 *          Systems) Services - version 3, with amendment 1/2, November 7, 2013
 *     [16] Proposal of new RTCM SSR Messages (ssr_1_gal_qzss_sbas_dbs_v05)
 *          2014/04/17
 *     [17] RTCM Standard 10403.3, Differential GNSS (Global Navigation Satellite
 *          Systems) Services - version 3, with amendment 1, April 28, 2020
 *     [18] IGS State Space Representation (SSR) Format version 1.00, October 5,
 *          2020
 *
 * version : $Revision:$ $Date:$
 * history : 2021/03/15 1.0  new
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include "log.h"

/* function prototypes -------------------------------------------------------*/
extern int decode_rtcm2(rtcm_t *rtcm);
extern int decode_rtcm3(rtcm_t *rtcm);
extern int encode_rtcm3(rtcm_t *rtcm, int type, int subtype, int sync);

/* constants -----------------------------------------------------------------*/
#define RTCM2PREAMB 0x66        /* rtcm ver.2 frame preamble */
#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */

/* initialize rtcm control -----------------------------------------------------
 * initialize rtcm control struct and reallocate memory for observation and
 * ephemeris buffer in rtcm control struct
 * args   : rtcm_t *raw      IO  rtcm control struct
 * return : status (1:ok,0:memory allocation error)
 *-----------------------------------------------------------------------------*/
extern int init_rtcm(rtcm_t *rtcm)
{
    gtime_t time0={0};
    obsd_t data0={{0}};
    eph_t  eph0 ={0,-1,-1};
    geph_t geph0={0,-1};
    ssr_t ssr0={{{0}}};
    int i,j;

    rtcm->staid=rtcm->stah=rtcm->seqno=rtcm->outmsg=0;
    rtcm->time=rtcm->time_s=time0;
    rtcm->sta.name[0]=rtcm->sta.marker[0]='\0';
    rtcm->sta.antdes[0]=rtcm->sta.antsno[0]='\0';
    rtcm->sta.rectype[0]=rtcm->sta.recver[0]=rtcm->sta.recsno[0]='\0';
    rtcm->sta.antsetup=rtcm->sta.itrf=rtcm->sta.deltype=0;
    for (i=0;i<3;i++) {
        rtcm->sta.pos[i]=rtcm->sta.del[i]=0.0;
    }
    rtcm->sta.hgt=0.0;
    rtcm->dgps=NULL;
    for (i=0;i<MAXSAT;i++) {
        rtcm->ssr[i]=ssr0;
    }
    rtcm->msg[0]=rtcm->msgtype[0]=rtcm->opt[0]='\0';
    for (i=0;i<6;i++) rtcm->msmtype[i][0]='\0';
    rtcm->obsflag=rtcm->ephsat=0;
    rtcm->outmsg=1;
    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ+NEXOBS;j++) {
            rtcm->cp[i][j]=0.0;
            rtcm->lock[i][j]=rtcm->loss[i][j]=0;
            rtcm->lltime[i][j]=time0;
        }
    rtcm->nbyte=rtcm->nbit=rtcm->len=0;
    rtcm->word=0;
    for (i=0;i<100;i++) rtcm->nmsg2[i]=0;
    for (i=0;i<400;i++) rtcm->nmsg3[i]=0;

    rtcm->obs.data=NULL;
    rtcm->nav.eph =NULL;
    rtcm->nav.geph=NULL;

    /* reallocate memory for observation and ephemeris buffer */
    if (!(rtcm->obs.data=(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS))||
        !(rtcm->nav.eph =(eph_t  *)malloc(sizeof(eph_t )*MAXSAT*2))||
        !(rtcm->nav.geph=(geph_t *)malloc(sizeof(geph_t)*MAXPRNGLO))) {
        free_rtcm(rtcm);
        return 0;
    }
    rtcm->obs.n=0;
    rtcm->nav.n=MAXSAT*2;
    rtcm->nav.ng=MAXPRNGLO;
    for (i=0;i<MAXOBS   ;i++) rtcm->obs.data[i]=data0;
    for (i=0;i<MAXSAT*2 ;i++) rtcm->nav.eph [i]=eph0;
    for (i=0;i<MAXPRNGLO;i++) rtcm->nav.geph[i]=geph0;
    return 1;
}
/* free rtcm control ----------------------------------------------------------
 * free observation and ephemeris buffer in rtcm control struct
 * args   : rtcm_t *raw      IO  rtcm control struct
 * return : none
 *-----------------------------------------------------------------------------*/
extern void free_rtcm(rtcm_t *rtcm)
{
    /* free memory for observation and ephemeris buffer */
    free(rtcm->obs.data); rtcm->obs.data=NULL; rtcm->obs.n=0;
    free(rtcm->nav.eph ); rtcm->nav.eph =NULL; rtcm->nav.n=0;
    free(rtcm->nav.geph); rtcm->nav.geph=NULL; rtcm->nav.ng=0;
}
/* input RTCM 2 message from stream --------------------------------------------
 * fetch next RTCM 2 message and input a message from byte stream
 * args   : rtcm_t *rtcm     IO  rtcm control struct
 *          uint8_t data     I   stream data (1 byte)
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 5: input station pos/ant parameters,
 *                  6: input time parameter, 7: input dgps corrections,
 *                  9: input special message)
 * notes  : before firstly calling the function, time in rtcm control struct has
 *          to be set to the approximate time within 1/2 hour in order to resolve
 *          ambiguity of time in rtcm messages.
 *          supported msgs RTCM ver.2: 1,3,9,14,16,17,18,19,22
 *          refer [1] for RTCM ver.2
 *-----------------------------------------------------------------------------*/
extern int input_rtcm2(rtcm_t *rtcm, uint8_t data)
{
    uint8_t preamb;
    int i;

    if ((data&0xC0)!=0x40) return 0; /* ignore if upper 2bit != 01 */

    for (i=0;i<6;i++,data>>=1) { /* decode 6-of-8 form */
        rtcm->word=(rtcm->word<<1)+(data&1);

        /* synchronize frame */
        if (rtcm->nbyte==0) {
            preamb=(uint8_t)(rtcm->word>>22);
            if (rtcm->word&0x40000000) preamb^=0xFF; /* decode preamble */
            if (preamb!=RTCM2PREAMB) continue;

            /* check parity */
            if (!decode_word(rtcm->word,rtcm->buff)) continue;
            rtcm->nbyte=3; rtcm->nbit=0;
            continue;
        }
        if (++rtcm->nbit<30) continue; else rtcm->nbit=0;

        /* check parity */
        if (!decode_word(rtcm->word,rtcm->buff+rtcm->nbyte)) {
            log_trace(2,"rtcm2 partity error: i=%d word=%08x\n",i,rtcm->word);
            rtcm->nbyte=0; rtcm->word&=0x3;
            continue;
        }
        rtcm->nbyte+=3;
        if (rtcm->nbyte==6) rtcm->len=(rtcm->buff[5]>>3)*3+6;
        if (rtcm->nbyte<rtcm->len) continue;
        rtcm->nbyte=0; rtcm->word&=0x3;

        /* decode rtcm2 message */
        return decode_rtcm2(rtcm);
    }
    return 0;
}
/* input RTCM 3 message from stream --------------------------------------------
 * fetch next RTCM 3 message and input a message from byte stream
 * args   : rtcm_t *rtcm     IO  rtcm control struct
 *          uint8_t data     I   stream data (1 byte)
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 5: input station pos/ant parameters,
 *                  10: input ssr messages)
 * notes  : before firstly calling the function, time in rtcm control struct has
 *          to be set to the approximate time within 1/2 week in order to resolve
 *          ambiguity of time in rtcm messages.
 *
 *          to specify input options, set rtcm->opt to the following option
 *          strings separated by spaces.
 *
 *          -EPHALL  : input all ephemerides (default: only new)
 *          -STA=nnn : input only message with STAID=nnn (default: all)
 *          -GLss    : select signal ss for GPS MSM (ss=1C,1P,...)
 *          -RLss    : select signal ss for GLO MSM (ss=1C,1P,...)
 *          -ELss    : select signal ss for GAL MSM (ss=1C,1B,...)
 *          -JLss    : select signal ss for QZS MSM (ss=1C,2C,...)
 *          -CLss    : select signal ss for BDS MSM (ss=2I,7I,...)
 *          -ILss    : select signal ss for IRN MSM (ss=5A,9A,...)
 *          -GALINAV : select I/NAV for Galileo ephemeris (default: all)
 *          -GALFNAV : select F/NAV for Galileo ephemeris (default: all)
 *
 *          supported RTCM 3 messages (ref [7][10][15][16][17][18])
 *
 *            TYPE       :  GPS   GLONASS Galileo  QZSS     BDS    SBAS    NavIC
 *         ----------------------------------------------------------------------
 *          OBS COMP L1  : 1001~   1009~     -       -       -       -       -
 *              FULL L1  : 1002    1010      -       -       -       -       -
 *              COMP L1L2: 1003~   1011~     -       -       -       -       -
 *              FULL L1L2: 1004    1012      -       -       -       -       -
 *
 *          NAV          : 1019    1020    1045**  1044    1042      -     1041
 *                           -       -     1046**    -       63*     -       -
 *
 *          MSM 1        : 1071~   1081~   1091~   1111~   1121~   1101~   1131~
 *              2        : 1072~   1082~   1092~   1112~   1122~   1102~   1132~
 *              3        : 1073~   1083~   1093~   1113~   1123~   1103~   1133~
 *              4        : 1074    1084    1094    1114    1124    1104    1134
 *              5        : 1075    1085    1095    1115    1125    1105    1135
 *              6        : 1076    1086    1096    1116    1126    1106    1136
 *              7        : 1077    1087    1097    1117    1127    1107    1137
 *
 *          SSR ORBIT    : 1057    1063    1240*   1246*   1258*     -       -
 *              CLOCK    : 1058    1064    1241*   1247*   1259*     -       -
 *              CODE BIAS: 1059    1065    1242*   1248*   1260*     -       -
 *              OBT/CLK  : 1060    1066    1243*   1249*   1261*     -       -
 *              URA      : 1061    1067    1244*   1250*   1262*     -       -
 *              HR-CLOCK : 1062    1068    1245*   1251*   1263*     -       -
 *              PHAS BIAS:   11*     -       12*     13*     14*     -       -
 *
 *          ANT/RCV INFO : 1007    1008    1033
 *          STA POSITION : 1005    1006
 *
 *          PROPRIETARY  : 4076 (IGS)
 *         ----------------------------------------------------------------------
 *                            (* draft, ** 1045:F/NAV,1046:I/NAV, ~ only encode)
 *
 *          for MSM observation data with multiple signals for a frequency,
 *          a signal is selected according to internal priority. to select
 *          a specified signal, use the input options.
 *
 *          RTCM 3 message format:
 *            +----------+--------+-----------+--------------------+----------+
 *            | preamble | 000000 |  length   |    data message    |  parity  |
 *            +----------+--------+-----------+--------------------+----------+
 *            |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
 *
 *-----------------------------------------------------------------------------*/
extern int input_rtcm3(rtcm_t *rtcm, uint8_t data)
{
    rtcm->moni_msg=0;

    /* synchronize frame */
    if (rtcm->nbyte==0) {
        if (data!=RTCM3PREAMB) return 0;
        rtcm->buff[rtcm->nbyte++]=data;
        return 0;
    }
    rtcm->buff[rtcm->nbyte++]=data;

    if (rtcm->nbyte==3) {
        rtcm->len=getbitu(rtcm->buff,14,10)+3; /* length without parity */
    }
    if (rtcm->nbyte<3||rtcm->nbyte<rtcm->len+3) return 0;
    rtcm->nbyte=0;

    /* check parity */
    if (rtk_crc24q(rtcm->buff,rtcm->len)!=getbitu(rtcm->buff,rtcm->len*8,24)) {
        log_trace(1,"rtcm3 parity error: len=%d\n",rtcm->len);
        return 0;
    }
    /* decode rtcm3 message */
    return decode_rtcm3(rtcm);
}

static uint8_t* sync_frame(rtcm_t *rtcm, uint8_t *buf, int len, int *rlen)
{
    int i;
    if (rtcm->nbyte>=3) return buf;

    for (i=0;i<len;i++) {
        (*rlen)--;
        if (rtcm->nbyte==0) {
            if (buf[i]!=RTCM3PREAMB) continue;
            rtcm->buff[rtcm->nbyte++]=buf[i];
            continue;
        }
        rtcm->buff[rtcm->nbyte++]=buf[i];
        if (rtcm->nbyte==3) {
            rtcm->len=getbitu(rtcm->buff,14,10)+6;
            return i==len-1?NULL:&buf[++i];
        }
    }
    return NULL;
}

extern int input_rtcm3x(rtcm_t *rtcm, uint8_t *buf, int len, int *rlen)
{
    uint8_t *p;
    int nr;

    rtcm->moni_msg=0;
    if (!(p=sync_frame(rtcm,buf,len,rlen))) return 0;
    nr=*rlen;

    if (nr+rtcm->nbyte<rtcm->len) {
        memcpy(rtcm->buff+rtcm->nbyte,p,nr);
        rtcm->nbyte+=nr;
        *rlen=*rlen-nr;
        return 0;
    }
    memcpy(rtcm->buff+rtcm->nbyte,p,rtcm->len-rtcm->nbyte);
    *rlen=*rlen-(rtcm->len-rtcm->nbyte);
    rtcm->nbyte=0;
    rtcm->len=rtcm->len-3;

    /* check parity */
    if (rtk_crc24q(rtcm->buff,rtcm->len)!=getbitu(rtcm->buff,(rtcm->len)*8,24)) {
        log_trace(1,"rtcm3 parity error: len=%d\n",rtcm->len);
        return 0;
    }
    /* decode rtcm3 message */
    return decode_rtcm3(rtcm);
}
/* generate RTCM 2 message -----------------------------------------------------
 * generate RTCM 2 message
 * args   : rtcm_t *rtcm     IO  rtcm control struct
 *          int    type      I   message type
 *          int    sync      I   sync flag (1:another message follows)
 * return : status (1:ok,0:error)
 *-----------------------------------------------------------------------------*/
extern int gen_rtcm2(rtcm_t *rtcm, int type, int sync)
{
    log_trace(3,"gen_rtcm2: type=%d sync=%d\n",type,sync);

    rtcm->nbit=rtcm->len=rtcm->nbyte=0;

    /* not yet implemented */
    return 0;
}
/* generate RTCM 3 message -----------------------------------------------------
 * generate RTCM 3 message
 * args   : rtcm_t *rtcm     IO  rtcm control struct
 *          int    type      I   message type
 *          int    subtype   I   message subtype
 *          int    sync      I   sync flag (1:another message follows)
 * return : status (1:ok,0:error)
 * notes  : For rtcm 3 msm, the {nsat} x {nsig} in rtcm->obs should not exceed
 *          64. If {nsat} x {nsig} of the input obs data exceeds 64, separate
 *          them to multiple ones and call gen_rtcm3() multiple times as user
 *          responsibility.
 *          ({nsat} = number of valid satellites, {nsig} = number of signals in
 *          the obs data)
 *-----------------------------------------------------------------------------*/
extern int gen_rtcm3(rtcm_t *rtcm, int type, int subtype, int sync)
{
    uint32_t crc;
    int i=0;

    log_trace(3,"gen_rtcm3: type=%d subtype=%d sync=%d\n",type,subtype,sync);

    rtcm->nbit=rtcm->len=rtcm->nbyte=0;

    /* set preamble and reserved */
    setbitu(rtcm->buff,i, 8,RTCM3PREAMB); i+= 8;
    setbitu(rtcm->buff,i, 6,0          ); i+= 6;
    setbitu(rtcm->buff,i,10,0          ); i+=10;

    /* encode rtcm 3 message body */
    if (!encode_rtcm3(rtcm,type,subtype,sync)) return 0;

    /* padding to align 8 bit boundary */
    for (i=rtcm->nbit;i%8;i++) {
        setbitu(rtcm->buff,i,1,0);
    }
    /* message length (header+data) (bytes) */
    if ((rtcm->len=i/8)>=3+1024) {
        log_trace(2,"generate rtcm 3 message length error len=%d\n",rtcm->len-3);
        rtcm->nbit=rtcm->len=0;
        return 0;
    }
    /* message length without header and parity */
    setbitu(rtcm->buff,14,10,rtcm->len-3);

    /* crc-24q */
    crc=rtk_crc24q(rtcm->buff,rtcm->len);
    setbitu(rtcm->buff,i,24,crc);

    /* length total (bytes) */
    rtcm->nbyte=rtcm->len+3;

    return 1;
}
