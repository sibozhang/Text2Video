/* ----------------------------------------------------------- */
/*                                                             */
/*                          ___                                */
/*                       |_| | |_/   SPEECH                    */
/*                       | | | | \   RECOGNITION               */
/*                       =========   SOFTWARE                  */ 
/*                                                             */
/*                                                             */
/* ----------------------------------------------------------- */
/* developed at:                                               */
/*                                                             */
/*      Speech Vision and Robotics group                       */
/*      Cambridge University Engineering Department            */
/*      http://svr-www.eng.cam.ac.uk/                          */
/*                                                             */
/*      Entropic Cambridge Research Laboratory                 */
/*      (now part of Microsoft)                                */
/*                                                             */
/* ----------------------------------------------------------- */
/*         Copyright: Microsoft Corporation                    */
/*          1995-2000 Redmond, Washington USA                  */
/*                    http://www.microsoft.com                 */
/*                                                             */
/*              2002  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*      File: HList.c: List a Speech File or Audio Source      */
/* ----------------------------------------------------------- */

char *hlist_version = "!HVER!HList:   3.4.1 [CUED 12/03/09]";
char *hlist_vc_id = "$Id: HList.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HAudio.h"
#include "HWave.h"
#include "HVQ.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"


/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */

/* ---------------------- Global Variables ----------------------- */

static Boolean srcHdr  = FALSE;  /* print source header info */
static Boolean tgtHdr  = FALSE;  /* print target header info */
static Boolean obsFmt  = FALSE;  /* print observation format */
static Boolean prData  = TRUE;   /* print data */
static Boolean rawOut = FALSE;   /* raw output i.e no numbering */
static Boolean replay = FALSE;   /* replay audio */
static Boolean frcDisc = FALSE;  /* List VQ symbols from cont file */
static FileFormat ff = UNDEFF;   /* Source File format */
static long gst = -1;             /* start sample to list */
static long gen = -1;             /* end sample to list */
static int numS = 1;             /* number of streams */
static int nItems  = 10;         /* num items per line */
static int barwidth;             /* width of printed bars */
static char barc = '-';          /* bar character */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */
static HTime sampPeriod;         /* raw audio input only */
static int audSignal;

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;
   double d;

   sampPeriod = 0.0; audSignal = NULLSIG;
   nParm = GetConfig("HLIST", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"AUDIOSIG",&i)) audSignal = i;
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfFlt(cParm,nParm,"SOURCERATE",&d)) sampPeriod = d;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HList [options] file ...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -d      Coerce observation to VQ symbols     off\n");
   printf(" -e N    End at sample N                      0\n");
   printf(" -h      Print source header info             off\n");
   printf(" -i N    Set items per line to N              10\n");
   printf(" -n N    Set num streams to N                 1\n");
   printf(" -o      Print observation structure          off\n");
   printf(" -p      Playback audio                       off\n");
   printf(" -r      Write raw output                     off\n");
   printf(" -s N    Start at sample N                    0\n");
   printf(" -t      Print target header info             off\n");
   printf(" -z      Suppress printing data               on\n");
   PrintStdOpts("F");
   printf("\n\n");
}
   
int main(int argc, char *argv[])
{
   char *s,buf[MAXSTRLEN];
   void ListSpeech(char *src);
   
   if(InitShell(argc,argv,hlist_version,hlist_vc_id)<SUCCESS)
      HError(1100,"HList: InitShell failed");
   InitMem();
   InitMath();  InitSigP();
   InitWave();  InitAudio();
   InitVQ(); InitLabel();
   InitModel();
   if(InitParm()<SUCCESS)  
      HError(1100,"HList: InitParm failed");

   SetConfParms();
   if (GetConfStr(cParm,nParm,"SOURCEFORMAT",buf))
      ff = Str2Format(buf);

   if (!InfoPrinted() && NumArgs() == 0 && ff != HAUDIO)
      ReportUsage();
   if (NumArgs() == 0 && ff != HAUDIO)
      Exit(0);

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s) !=1 )
         HError(1119,"HList: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'd':
         frcDisc = TRUE; break;
      case 'e':
         gen = GetChkedLong(gst,LONG_MAX,s); break;
      case 'h':
         srcHdr = TRUE; break;
      case 'i':
         nItems = GetChkedInt(1,100,s); break;
      case 'n':
         numS = GetChkedInt(1,SMAX-1,s); break;
      case 'o':
         obsFmt = TRUE; break;
      case 'p':
         replay = TRUE; break;
      case 'r':
         rawOut = TRUE; break;
      case 's':
         gst = GetChkedLong(0,LONG_MAX,s); break;
      case 't':
         tgtHdr = TRUE; break;
      case 'z':
         prData = FALSE; break;
      case 'F':
         if (NextArg() != STRINGARG)
            HError(1119,"HList: File format expected");
         if((ff = Str2Format(GetStrArg())) == ALIEN)
            HError(-1189,"HList: Warning ALIEN src file format set");
         break;
      case 'T':
         trace = GetChkedInt(0,077,s); break;
      default:
         HError(1119,"HList: Unknown switch %s",s);
      }
   }
   if (NumArgs() == 0 ) 
      ListSpeech(NULL);
   else
      while (NumArgs() > 0 ) {
         if (NextArg() != STRINGARG)
            HError(1119,"HList: List file name expected");
         ListSpeech(GetStrArg());
      }
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* -------------------- Display Routines  ------------------- */

typedef struct{   /* gather together basic header info */
   char *name;
   Boolean isSource;
   Boolean isAudio;
   FileFormat fmt;
   ParmKind kind;
   HTime period;
   int sampSize;
   int numComps;
   long nSamples;
}HeadInfo;

/* PrBar: print a horizantal bar of length n including title */
void PrBar(char *title)
{
   int i,len,n;
   
   if (rawOut) return;
   len = strlen(title);
   n = (barwidth - len)/2;
   for (i=1; i<n; i++) putchar(barc);
   printf(" %s ",title);
   n = barwidth - n - len;
   for (i=1; i<n; i++) putchar(barc);
   putchar('\n');
}
   
/* SetBarWidth: set bar width according to nItems and item type */
void SetBarWidth(int itemWidth)
{
   barwidth = itemWidth*(nItems+1);
}

/* PrintHeading: print the info in given HeadInfo record */
void PrintHeading(HeadInfo h)
{
   char buf[MAXSTRLEN];
   
   if (h.isSource){
      if (h.isAudio)
         strcpy(buf,"Source: Direct Audio");    
      else
         sprintf(buf,"Source: %s", h.name);
   }else
      strcpy(buf,"Target");
   PrBar(buf);
   printf("  Sample Bytes:  %-7d", h.sampSize);
   if (barwidth < 60 ) printf("\n");
   printf("  Sample Kind:   %s\n", ParmKind2Str(h.kind,buf));
   printf("  Num Comps:     %-7d", h.numComps);
   if (barwidth < 60 ) printf("\n");
   printf("  Sample Period: %.1f us\n", h.period/10.0);     
   if (!h.isAudio) {
      printf("  Num Samples:   %-7ld", h.nSamples);
      if (barwidth < 60 ) printf("\n");
      printf("  File Format:   %s\n", Format2Str(h.fmt));
   }
}

/* PrintWaveLine: print line of waveform samples */
void PrintWaveLine(short *data, int nItems, long idx)
{
   int i;
   
   if (!rawOut) printf("%5ld: ",idx);
   for (i=0; i<nItems; i++)  printf("%7d",*data++);
   printf("\n");
}

/* PrintRawVec: print vector components */
void PrintRawVec(Vector v)
{
   int i;
   
   for (i=1; i<=VectorSize(v); i++)  printf("%e ",v[i]);
   printf("\n");
}

/* PrintObsFmt: print observation structure */
void PrintObsFmt(Observation *o)
{
   PrBar("Observation Structure");
   ExplainObservation(o,nItems);
}

/* PrintDataBar: print bar before data */
void PrintDataBar(long st, long en)
{
   char buf[MAXSTRLEN];

   sprintf(buf,"Samples: %ld->%ld", st<0?0:st,en);
   PrBar(buf);
}

/* ---------------- Data Access Routines  ------------------- */

/* IsWave: check config parms to see if target is a waveform */
Boolean IsWave(char *srcFile)
{
   FILE *f;
   long nSamp,sampP, hdrS;
   short sampS,kind;
   Boolean isPipe,bSwap,isWave;
   char buf[MAXSTRLEN];
   ParmKind tgtPK=ANON;
   FileFormat srcFF=HTK;
   Boolean isEXF;               /* srcFile is extended file */
   char actfname[MAXFNAMELEN];  /* actual filename */
   long stIndex, enIndex;       /* start and end indices */
   
   if (ff!=UNDEFF) srcFF = ff;
   /* Read all configuration params and get target */
   if (GetConfStr(cParm,nParm,"TARGETKIND",buf))
      tgtPK = Str2ParmKind(buf);
   isWave = tgtPK == WAVEFORM;
   if (tgtPK == ANON){
      if ((srcFF == HTK || srcFF == ESIG) && srcFile != NULL){
         strncpy (actfname, srcFile, MAXFNAMELEN);
         isEXF = GetFileNameExt (srcFile, actfname, &stIndex, &enIndex);
         
         if ((f = FOpen (actfname, WaveFilter, &isPipe)) == NULL)
            HError(1110,"IsWave: cannot open File %s",srcFile);
         switch (srcFF) {
         case HTK:
            if (!ReadHTKHeader(f,&nSamp,&sampP,&sampS,&kind,&bSwap))
               HError(1113, "IsWave: cannot read HTK Header in File %s",
                      srcFile);
            break;
         case ESIG:
            if (!ReadEsignalHeader(f, &nSamp, &sampP, &sampS,
                                   &kind, &bSwap, &hdrS, isPipe))
               HError(1113, "IsWave: cannot read Esignal Header in File %s",
                      srcFile);
            break;
         }
         isWave = kind == WAVEFORM;
         FClose(f,isPipe);
      } else
         isWave = TRUE;
   }
   return isWave;
}

#define AUDIO_BSIZE 1000      /* Audio Buffer Size */

/* ListWavefromAudio: list wave from audio device */
void ListWavefromAudio(void)
{
   AudioIn a;
   AudioOut b;
   HeadInfo hi;
   int avail,bsize,nrows,idx=0,np;
   short buf[AUDIO_BSIZE],*p;
   static MemHeap x,y;
   static Boolean heapsCreated=FALSE;

   if (trace&T_TOP)
      printf(" Listing Waveform from Audio Source\n");
   if (!heapsCreated) {
      CreateHeap(&x,"AudioIn",  MSTAK, 1, 0.5, 100000, 5000000);
      CreateHeap(&y,"AudioOut", MSTAK, 1, 0.0, 100000, 500000);
      heapsCreated = TRUE;
   } else {
      ResetHeap(&x);
      ResetHeap(&y);
   }

   nrows = AUDIO_BSIZE / nItems; bsize = nrows * nItems;
   a = OpenAudioInput(&x,&sampPeriod, 0.0, 0.0);
   if (a==NULL)
      HError(1106,"ListWavefromAudio: No audio support");
   if (replay)
      AttachReplayBuf(a,100000);
   SetBarWidth(7);
   if (srcHdr) {
      hi.period = sampPeriod; hi.isAudio = TRUE; hi.isSource = TRUE;
      hi.kind = WAVEFORM; hi.sampSize = 2; hi.numComps = 1;
      PrintHeading(hi);
   }
   if (prData || gst>-1 || gen>-1) {
      StartAudioInput(a,audSignal);
      while (GetAIStatus(a) != AI_CLEARED) {
         avail = bsize;
         GetRawAudio(a, avail, buf);
         p = buf;
         while (avail>0){
            np = (avail>nItems)?nItems:avail;
            if (idx >= gst  && (idx <= gen || gen==-1))
               PrintWaveLine(p,np,idx);
            p += np; idx += np; avail -= np;
         }
      }
      if (replay){
         b = OpenAudioOutput(&y,&sampPeriod);
         PlayReplayBuffer(b,a);
         while (SamplesToPlay(b) > 0 );
         CloseAudioOutput(b);
      }
   }
   CloseAudioInput(a);
}

/* ListWavefromFile: list wave from file */
void ListWavefromFile(char *src)
{
   Wave w;
   HeadInfo hi;
   short *data;
   long ns,np,idx = 0;
   static MemHeap x;
   static Boolean heapsCreated = FALSE;
   
   if (trace&T_TOP)
      printf(" Listing Waveform from File %s\n",src);
   if (!heapsCreated) {
      CreateHeap(&x,"WaveIn", MSTAK, 1, 0.5, 100000, 5000000);   
      heapsCreated = TRUE;
   } else {
      ResetHeap(&x);
   }

   if((w = OpenWaveInput(&x,src,ff,0.0,0.0,&sampPeriod))==NULL)
      HError(1113,"ListWavefromFile: OpenWaveInput failed");
   data = GetWaveDirect(w,&(hi.nSamples));
   SetBarWidth(7);
   if (srcHdr) {
      hi.period = sampPeriod;
      hi.name = src; hi.fmt = WaveFormat(w);
      hi.isAudio = FALSE; hi.isSource = TRUE;
      hi.kind = WAVEFORM; hi.sampSize = 2; hi.numComps = 1;
      PrintHeading(hi);
   }
   if (prData || gst>-1 || gen >-1) {
      ns = hi.nSamples;
      if (gst>-1) {
         idx = gst; data += gst; ns -= gst;
      }
      if (gen>-1 && ns > gen-gst+1)
         ns = gen-gst+1;
      PrintDataBar(idx,idx+ns-1);
      while (ns>0){
         np = (ns>nItems)?nItems:ns;
         PrintWaveLine(data,np,idx);
         data += np; idx += np; ns -= np;
      }
   }
   CloseWaveInput(w);
}

/* ListParms: list param data derived from file or audio */
void ListParms(char *src)
{
   ParmBuf pbuf;
   AudioOut b;
   Observation o;
   HeadInfo hi,ho;
   long idx = 0;
   Boolean eSep;
   short swidth[SMAX];
   BufferInfo info;
   static MemHeap x,y;
   static Boolean heapsCreated = FALSE;

   if (trace&T_TOP)
      printf(" Listing Parameter Vectors from %s\n",
             src==NULL?"audio":src);
   if (!heapsCreated) {
      CreateHeap(&x,"BufferIn", MSTAK, 1, 0.5, 100000, 5000000);
      CreateHeap(&y,"AudioOut", MSTAK, 1, 0.0, 100000,  100000);
      heapsCreated = TRUE;
   } else {
      ResetHeap(&x);
      ResetHeap(&y);
   }

   if((pbuf = OpenBuffer(&x,src,50,ff,TRI_UNDEF,TRI_UNDEF))==NULL)
      HError(1150,"ListParms: Config parameters invalid");
   GetBufferInfo(pbuf,&info);
   SetBarWidth(8);
   hi.isAudio = src==NULL;
   if (replay && hi.isAudio)
      AttachReplayBuf(info.a,100000);
   if (srcHdr) {
      hi.name = hi.isAudio ? (char *) "audio" : src;
      hi.fmt = info.srcFF; hi.isSource = TRUE;
      hi.kind = info.srcPK;
      if (hi.kind == WAVEFORM){
         hi.nSamples = info.nSamples;
         hi.numComps = 1; hi.sampSize = 2;
      }else{
         hi.nSamples = hi.isAudio?0:ObsInBuffer(pbuf);
         hi.numComps = info.srcVecSize;
         hi.sampSize = hi.numComps*sizeof(float);
         if (HasCompx(info.srcPK) || BaseParmKind(info.srcPK) == IREFC ||
             BaseParmKind(info.srcPK) == DISCRETE) 
            hi.sampSize /= 2;
      }
      hi.period = info.srcSampRate;
      PrintHeading(hi);
   }
   ho.nSamples = hi.isAudio?0:ObsInBuffer(pbuf);
   if (tgtHdr) {
      ho.name = ""; ho.fmt = info.tgtFF;
      ho.isAudio = FALSE; ho.isSource = FALSE;
      ho.kind = info.tgtPK;
      ho.numComps = info.tgtVecSize;
      ho.sampSize = ho.numComps*sizeof(float);
      if (BaseParmKind(info.tgtPK) == DISCRETE) 
         ho.sampSize /= 2;
      ho.period = info.tgtSampRate;
      PrintHeading(ho);
   }
   ZeroStreamWidths(numS,swidth);
   SetStreamWidths(info.tgtPK,info.tgtVecSize,swidth,&eSep);
   o = MakeObservation(&gstack,swidth,info.tgtPK,frcDisc,eSep);
   if (obsFmt)
      PrintObsFmt(&o);
   if (prData || gst>-1 || gen >-1) {
      PrintDataBar(gst,gen);
      StartBuffer(pbuf);
      while (BufferStatus(pbuf) != PB_CLEARED){
         ReadAsBuffer(pbuf,&o);
         if (idx >= gst  && (idx <= gen || gen==-1)){
            if (rawOut)
               PrintRawVec(o.fv[1]);
            else
               PrintObservation((int)idx,&o,nItems);
            fflush(stdout);
         }
         if (gen>-1 && idx==gen && hi.isAudio)
            StopBuffer(pbuf);
         ++idx;
      }
      if (replay && hi.isAudio){
         b = OpenAudioOutput(&y,&sampPeriod);
         PlayReplayBuffer(b,info.a);
         while (SamplesToPlay(b) > 0 );
         CloseAudioOutput(b);
      }
   }
   CloseBuffer(pbuf);
}

/* ListSpeech: top level control routine */
void ListSpeech(char *src)
{
   if (IsWave(src)){
      if (src==NULL)
         ListWavefromAudio();
      else
         ListWavefromFile(src);
   } else
      ListParms(src);
   PrBar("END");
}

/* ----------------------------------------------------------- */
/*                      END:  HList.c                          */
/* ----------------------------------------------------------- */
