/* ----------------------------------------------------------- */
/*                                                             */
/*                          ___                                */
/*                       |_| | |_/   SPEECH                    */
/*                       | | | | \   RECOGNITION               */
/*                       =========   SOFTWARE                  */ 
/*                                                             */
/*                                                             */
/* ----------------------------------------------------------- */
/*         Copyright: Microsoft Corporation                    */
/*          1995-2000 Redmond, Washington USA                  */
/*                    http://www.microsoft.com                 */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*      File: HAudio.c: Audio Input/Output Interface           */
/* ----------------------------------------------------------- */

char *haudio_version = "!HVER!HAudio:   3.4.1 [CUED 12/03/09]";
char *haudio_vc_id = "$Id: HAudio.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

#include "HShell.h"        /* HTK Libraries */
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HAudio.h"

/* ----------------------------- Trace Flags ------------------------- */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */
#define T_STC  0002     /* Trace Audio State Changes */
#define T_DET  0004     /* Trace Detector State Changes */
#define T_AUD  0010     /* Trace device dependent audio code */
#define T_RUN  0020     /* Trace audio read/status */

/* -------------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];       /* config parameters */
static int numParm = 0;

/* ---------------------------------------------------------- */
/* 
   The type of audio device must be defined  - choices are
      NO_AUDIO    - none
      SGI_AUDIO   - SGI Indigo
      SUN16_AUDIO - Sun 16-bit linear
*/

#ifdef USS_AUDIO
#define OSS_AUDIO
#endif

#if !defined NO_AUDIO && !defined SGI_AUDIO && !defined SUN16_AUDIO && !defined SOLARIS16_AUDIO && !defined HPUX_AUDIO && !defined RS6000_AUDIO && !defined WIN95_AUDIO && !defined OSS_AUDIO && !defined HPRAW_AUDIO && !defined DEC_AUDIO && !defined WIN32_AUDIO && !defined EXT_FD_AUDIO 
#define NO_AUDIO
#endif

#ifdef WIN32_AUDIO
#define MMAPI_AUDIO
#include <windows.h>
#include <mmsystem.h>
#include <stdlib.h>    
#include <search.h>    
#endif
#ifdef DEC_AUDIO
#define MMAPI_AUDIO
#include <mme/mme_api.h>
#endif
#ifdef RS6000_AUDIO
#include <string.h>
#include <fcntl.h>
#include <UMSBAUDDevice.h>
/* function return codes */
#define rOK UMSAudioDevice_Success  
#define rUR UMSAudioDevice_UnderRun
#define rOR UMSAudioDevice_OverRun
#ifdef AIX_3_2_5
#define AUDIO_DEV "/dev/baud0"
#else
#define AUDIO_DEV "/dev/paud0"
#endif
#endif
#ifdef SGI_AUDIO
#include <audio.h>
#include <unistd.h>
#endif
#ifdef SUN16_AUDIO
#include <fcntl.h>
#include <stropts.h>
#include <sys/filio.h>
#include <sun/audioio.h>  /* May be in sys in local configuration */
#include <sys/ioctl.h>
#define AUDIO_IODEV  "/dev/audioctl"
#define AUDIO_IO     "/dev/audio"
#endif
#ifdef SOLARIS16_AUDIO
#include <fcntl.h>
#include <stropts.h>
#include <sys/filio.h>
#include <sys/audioio.h>  /* May be in sys in local configuration */
#include <sys/ioctl.h>
#define AUDIO_IODEV  "/dev/audioctl"
#define AUDIO_IO     "/dev/audio"
#define SUN16_AUDIO
#endif
#ifdef HPUX_AUDIO
#include <fcntl.h>
#include <sys/socket.h>
#include <time.h>
#include <sys/audio.h>
/* to include audio library compile with:
      -I/usr/include/audio for HPUX-9.x 
      -I/opt/audio/include for HPUX-10.x */
#include <Alib.h>
static Audio *audio_dev=NULL;
static int audio_cnt=0;
#endif
#ifdef OSS_AUDIO
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>
#define AUDIO_DEV "/dev/dsp"
#define MIXER_DEV "/dev/mixer"
#define AUDIO_RD 0x01
#define AUDIO_WR 0x02
#define BUF_SIZE 4096
static short zero_buf[BUF_SIZE];
static int audio_fd;
static int mixer_fd;
static unsigned short audio_io = 0x00;
static int frag_size;
static audio_buf_info audio_info;
#endif

/* Define the Audio Stream Information Records */
#define AUDBUFSIZE   640000     /* size of audio buffer */

typedef enum {
   v_peak=1,
   v_rms
} VolType;

static VolType volType = v_peak;
static Boolean lineOut = TRUE;
static Boolean phonesOut = TRUE;
static Boolean speakerOut = FALSE;
static Boolean lineIn = TRUE;
static Boolean micIn = FALSE;

static volatile Boolean stopSignalled;

typedef enum { ADS_INIT, ADS_OPEN, ADS_SAMPLING, 
               ADS_STOPPED, ADS_CLOSED } AudioDevStatus;

typedef struct {       /* circular buffer */
   Boolean isActive;     /* true if in use */
   short *data;          /* actual data buffer */
   int inx,outx;         /* in/out indices - wrap modulo size */
   int used,size;        /* used in data, size of data */
}ReplayBuf;


#ifdef MMAPI_AUDIO
#define MMAPI_BUFFER_DURATION 0.2
#define MMAPI_BUFFER_COUNT 12
static DWORD sMagic=-1;
typedef struct mmapibuf{
   int index;             /* Index of buffer */
   int size;              /* Size of buffer */
   int n;                 /* Number of valid samples in buffer */
   int cur;               /* Current sample index */
   LPWAVEHDR waveHdr;     /* Pointer to WAVEHDR */
   LPSTR waveData;        /* Data in buffer */
   struct mmapibuf *next;
   struct mmapibuf *prev;
} mmApiBuf;
#endif

typedef struct _AudioIn {
   /* -- Machine Independent Part -- */
   MemHeap *mem;             /* memory heap for this audio rec */
   HTime sampPeriod;         /* sampling period in 100ns units */
   int frSize;               /* num samples per speech frame */
   int frRate;               /* num samples between speech frames */
   short * frBuf;            /* buffer for constructing frames */
   Vector frOLap;            /* frame overlap buffer used by GetAudio */
   int inOLap;               /* num samples in frOLap */
   AudioInStatus status;     /* current status of this audio stream */
   short buffer[AUDBUFSIZE];
   int bufferSize;           /* Size of audio buffer */
   int nInBuffer;            /* Number of valid samples in buffer */
   int inBufPos;             /* Position to write in to buffer */
   int outBufPos;            /* Position to read out of buffer */
   int sig;                  /* signal if any */
   ReplayBuf rbuf;           /* replay buffer (if needed) */
   AudioDevStatus isActive;  /* indicates when device active */
   float curVol;             /* Current volume of input speech */
   /* -- Machine Dependent Part -- */
#ifdef MMAPI_AUDIO
   MMRESULT mmError;
   DWORD magic;             /* Magic number identifying this instance */
   HWAVEIN waveIn;
   LPPCMWAVEFORMAT waveFmt; /* Pointer to PCMWAVEFORMAT */
   LPMMTIME wavePos;        /* Pointer to MMTIME */
   int total;               /* Total number of samples queued */
   int current;             /* Index of current buffer */
   int bufSize;             /* Block size for each buffer */
   mmApiBuf *qHead;         /* Head of buffer waiting to be filled list */
   mmApiBuf *qTail;         /* Tail of buffer waiting to be filled list */
   mmApiBuf *fHead;         /* Head of filled buffer list */
   mmApiBuf *fTail;         /* Tail of filled buffer list */
#ifdef WIN32_AUDIO
   CRITICAL_SECTION c;
   HANDLE callBackEvent;
#endif
#endif
#ifdef RS6000_AUDIO
   UMSBAUDDevice adevin;
   UMSAudioDevice_ReturnCode rc;
   Environment *evin;
   long sw;
   long osamples;            /* Sample rate */
   char *obyte_order;
   long lgain;
   long rgain;
   long channels ;
   long bits ;
   char inConn[30];   
#endif
#ifdef SGI_AUDIO
   long params[2];           /* parameter array */
   ALport recPort;           /* SGI audio port */
   ALconfig config;          /* configuration record */
#endif
#ifdef SUN16_AUDIO
   int numSamples;
   int audio_ctld;
   audio_info_t audio_info;
#endif
#ifdef HPUX_AUDIO
   Audio *audio;
   ATransID tid;
   AudioAttributes attr;
   SSRecordParams parms;
   AGainEntry gains[4];
   SStream stream;
   int socket;
#endif
}AudioInRec;

typedef struct _AudioOut {
   /* -- Machine Independent Part -- */
   MemHeap *mem;             /* memory heap for this audio rec */
   float vol;                /* current volume */
   Boolean isActive;         /* true when device active */
   /* -- Machine Dependent Part -- */
#ifdef MMAPI_AUDIO
   MMRESULT mmError;
   DWORD magic;             /* Magic number identifying this instance */
   HWAVEOUT waveOut;
   UINT waveOutDev;         /* Device being used for wave output */
   LPPCMWAVEFORMAT waveFmt; /* Pointer to PCMWAVEFORMAT */
   LPMMTIME wavePos;        /* Pointer to MMTIME */
   int total;               /* Total number of samples queued */
   int current;             /* Index of current buffer */
   mmApiBuf *pHead;         /* Head of buffer list */
   mmApiBuf *pTail;         /* Tail of buffer list */
#ifdef WIN32_AUDIO
   CRITICAL_SECTION c;
   HANDLE callBackEvent;
#endif
#endif
#ifdef RS6000_AUDIO
   UMSBAUDDevice adevout;
   UMSAudioDevice_ReturnCode rc;
   Environment *evout;
   long sw;
   long osamples;            /* Sample rate */
   char *obyte_order;
   long lgain;
   long rgain;
   long channels ;
   long bits ;
   char outConn[30];   
#endif
#ifdef SGI_AUDIO
   long params[6];           /* parameter array */
   ALport playPort;          /* SGI audio port */
   ALconfig config;          /* configuration record */
#endif
#ifdef SUN16_AUDIO
   int numSamples;
   int numWrites;
   int audio_ctld;
   audio_info_t audio_info;
#endif
#ifdef HPUX_AUDIO
   Audio *audio;
   ATransID tid;
   int nToPlay;
   AudioAttributes attr;
   SSPlayParams parms;
   AGainEntry gains[4];
   SStream stream;
   int socket;
#endif
}AudioOutRec;

/* ------------------ Device Dependent Routines ----------------- */
/* All device dependent parts of this module are in this section  */
/* -------------------------------------------------------------- */

#ifdef SUN16_AUDIO
#define NUM_SAMP_FREQS 10
static float sampFreqs[NUM_SAMP_FREQS] = {
   8000, 9600, 11025, 16000, 18900, 22050, 32000, 37800, 44100, 48000
};

/* TrimSampFreq: find the nearest available sampling frequency */
static int TrimSampFreq(int f)
{
   int i, d;
   int min, mi;
  
   min = (int) fabs((double)(sampFreqs[0]-f)); mi = 0;
   for (i=0; i<NUM_SAMP_FREQS; i++) {
      d = (int) fabs((double)(sampFreqs[i]-f));
      if (d < min) {
         min = d; mi = i;
      }
   }
   return (int) sampFreqs[mi];
}
#endif

#ifdef OSS_AUDIO
/* IsVAXOrder: returns true if machine has VAX ordered bytes */
static Boolean IsVAXOrder(void)
{
   short x, *px;
   unsigned char *pc;
   
   px = &x;
   pc = (unsigned char *) px;
   *pc = 1; *(pc+1) = 0;         /* store bytes 1 0 */
   return x==1;          /* does it read back as 1? */
}
#endif

/* CalcVolume: calculate volume of data */
static float CalcVolume(short *data, int len)
{
   float vol;
   double sum, sqr;
   int i, minSamp, maxSamp;
   
   switch (volType) {
   case v_rms:
      sum=sqr=0.0;
      for(i = 0; i < len; i++) {
         sum += (double)data[i];
         sqr += ((double)data[i]) * ((double)data[i]);
      }
      sum/=len; sqr/=len;
      vol = sqrt(sqr-sum*sum);
      break;
   case v_peak:
   default:
      minSamp =maxSamp = data[0];
      for(i = 0; i < len; i++) {
         if ( data[i] > maxSamp )  maxSamp = data[i];
         else if ( data[i] < minSamp )  minSamp = data[i];
      }
      vol = (maxSamp-minSamp)/2.0;
      break;
   }
   return vol;
}


#ifdef MMAPI_AUDIO
#ifdef WIN32_AUDIO
void *mmeAllocMem(size_t size)
{
   void *ptr;
   ptr=GlobalAlloc(GMEM_FIXED,size);
   if (ptr==NULL)
      HError(6006,"StartAudi: Cannot allocate memory for mme structure");
   return(ptr);
}

void *mmeAllocBuffer(size_t size)
{
   void *ptr;
   ptr=GlobalAlloc(GMEM_FIXED,size);
   if (ptr==NULL)
      HError(6006,"StartAudi: Cannot allocate memory for mme structure");
   return(ptr);
}

Boolean mmeFreeMem(void *ptr)
{
   ptr=GlobalFree(ptr);
   return(TRUE);
}

Boolean mmeFreeBuffer(void *ptr)
{
   ptr=GlobalFree(ptr);
   return(TRUE);
}

void mmeProcessCallbacks(void)
{
}
void mmeWaitForCallbacks(void)
{
}
Boolean mmeCheckForCallbacks(void)
{
   return(FALSE);
}
#endif

static AudioIn sAudioIn=NULL;

void CALLBACK callBackIn(HWAVE hwaveIn, UINT msg, DWORD magic, 
                         LPARAM param1, LPARAM param2)
{
   AudioIn a;
   LPWAVEHDR curHdr;
   mmApiBuf *cur,*p,*r;

   a = sAudioIn;
   if (a==NULL || a->magic!=magic) return;

   switch(msg) {
   case MM_WIM_DATA:
#ifdef WIN32_AUDIO
      EnterCriticalSection(&a->c);
#endif
      curHdr=(LPWAVEHDR)param1;
      if (a->qHead==NULL || a->qHead->index!=curHdr->dwUser) {
         a->current=-1;    /* Buffer underran */
      }
      else {
         cur=a->qHead;
         if (cur->next==NULL) {
            a->qHead=a->qTail=NULL;  /* Buffer about to underrun */
         }
         else {
            /* Remove from head of queue */
            cur->next->prev=NULL; a->qHead=cur->next;
            cur->next->index=cur->index+1;
         }
         /* Set it up */
         cur->cur=0; cur->n=curHdr->dwBytesRecorded/sizeof(short);
         /* Add to tail of filled list */
         if (a->fTail==NULL)
            cur->next=cur->prev=NULL,a->fHead=a->fTail=cur;
         else {
            cur->prev=a->fTail; cur->next=NULL;
            a->fTail->next=cur; a->fTail=cur;
         }
      }
#ifdef WIN32_AUDIO
      LeaveCriticalSection(&a->c);
      SetEvent(a->callBackEvent);
#endif
      break;
   default:
      break;
   }
}
#endif


/* InitAudi: initialise the given audio input device */
static void InitAudi(AudioIn a, HTime *sampPeriod)
{
   if (a->isActive!=ADS_INIT) return;
   if (trace&T_AUD) {
      printf("Initialising Audio Input @%.0f\n",*sampPeriod);
      fflush(stdout);
   }
#ifdef MMAPI_AUDIO 
   {
      int i=0;

      /* Initialise */
      if (sAudioIn!=NULL)
         HError(6006,"InitAudi: MMAPI audio input already in use");
      sAudioIn=a;
      a->current=-1; a->fHead=a->fTail=NULL; a->qHead=a->qTail=NULL; 
      a->magic=~sMagic; sMagic+=12345;

      /* Allocate special structures */
      a->waveFmt = mmeAllocMem(sizeof(PCMWAVEFORMAT));
      a->wavePos = mmeAllocMem(sizeof(MMTIME));

      /* Set up required format */
      a->waveFmt->wf.wFormatTag = WAVE_FORMAT_PCM;
      a->waveFmt->wf.nChannels = 1;
      if(*sampPeriod == 0.0){
         *sampPeriod = 1.0E+07 / (float)16000;
         a->waveFmt->wf.nSamplesPerSec = 16000;
      }else
         a->waveFmt->wf.nSamplesPerSec = 1.0E+07 / *sampPeriod;
      a->waveFmt->wf.nBlockAlign = sizeof(short);
      a->waveFmt->wf.nAvgBytesPerSec = 
         a->waveFmt->wf.nBlockAlign*a->waveFmt->wf.nSamplesPerSec;
      a->waveFmt->wBitsPerSample = 16;

      /* Set up position query */
      a->wavePos->wType = TIME_SAMPLES;

      a->bufSize = MMAPI_BUFFER_DURATION*a->waveFmt->wf.nAvgBytesPerSec;
#ifdef WIN32_AUDIO
      InitializeCriticalSection( &(a->c) );
      a->callBackEvent = CreateEvent( NULL, TRUE, FALSE, "callBack" );
#endif
      if (trace & T_AUD) {
         printf(" Initialised MMAPI audio input at %.2fkHz\n",
                a->waveFmt->wf.nSamplesPerSec*1E-3);
         fflush(stdout);
      }
   }
#endif
#ifdef RS6000_AUDIO
   {
      long sampRate;

      a->evin = somGetGlobalEnvironment();
      a->adevin = UMSBAUDDeviceNew();
      if ((a->rc = UMSAudioDevice_open(a->adevin,a->evin,AUDIO_DEV,"RECORD",
                                       UMSAudioDevice_BlockingIO))!=rOK)
         HError(6006,"InitAudi: Cannot open RS6000 audio input device"); 
      
      a->channels = 1 ;
      a->bits  = 16 ; 
      if (*sampPeriod == 0.0){      
         UMSAudioDevice_get_sample_rate(a->adevin,a->evin,&sampRate);
         *sampPeriod= 1.0E+07 / sampRate;                  
      }
      else
         sampRate = 1.0E+07 / *sampPeriod;
      a->rc=UMSAudioDevice_set_sample_rate(a->adevin,a->evin,sampRate,&a->osamples);
      a->rc=UMSAudioDevice_set_bits_per_sample(a->adevin,a->evin,a->bits);
      a->rc=UMSAudioDevice_set_number_of_channels(a->adevin,a->evin,a->channels);
      a->rc=UMSAudioDevice_set_audio_format_type(a->adevin,a->evin,"PCM");
      a->rc=UMSAudioDevice_set_byte_order(a->adevin,a->evin,"MSB");
      a->rc=UMSAudioDevice_set_number_format(a->adevin,a->evin,"TWOS COMPLEMENT");
      a->rc = UMSAudioDevice_get_byte_order(a->adevin,a->evin,&a->obyte_order);
      /* you have to free the string after the query */
      if (a->obyte_order) free(a->obyte_order);
      a->rc = UMSAudioDevice_set_volume(a->adevin,a->evin,100);
      a->rc = UMSAudioDevice_set_balance(a->adevin,a->evin,0);
      a->rc=UMSAudioDevice_set_time_format(a->adevin,a->evin,UMSAudioTypes_Samples);
      a->lgain = 100; /*maximum left input gain*/
      a->rgain = 100; /*maimum right input gain*/
      if (micIn)
         strcpy(a->inConn, "HIGH_GAIN_MIC");
      else
         strcpy(a->inConn, "LINE_IN");
      a->rc = UMSAudioDevice_enable_input(a->adevin,a->evin,a->inConn,
                                          &a->lgain,&a->rgain);
      a->rc = UMSAudioDevice_set_monitor(a->adevin,a->evin,TRUE);
      a->rc = UMSAudioDevice_initialize(a->adevin,a->evin);
   }
#endif
#ifdef SGI_AUDIO
   a->params[0] = AL_INPUT_RATE;
   ALgetparams(AL_DEFAULT_DEVICE, a->params, 2);
   if (*sampPeriod == 0.0 ) 
      *sampPeriod =  1.0E+07 / (float) a->params[1] + 0.5;
   else {
      a->params[1] = (long int) (1.0E+07 / *sampPeriod + 0.5);
      ALsetparams(AL_DEFAULT_DEVICE,a->params,2);
   }
   a->config = ALnewconfig();
   ALsetchannels(a->config, AL_MONO);
   ALsetwidth(a->config, AL_SAMPLE_16);
   ALsetqueuesize(a->config, a->params[1] * 5);
#endif
#ifdef SUN16_AUDIO
   {
      int f,g,i=0;
      
      if ((a->audio_ctld = open(AUDIO_IO, O_RDONLY + O_NDELAY)) < 0) 
         HError(6006,"InitAudi: Cannot open Sun audio input%s",
                (errno==EBUSY?" [Already in use]":""));
      if (ioctl(a->audio_ctld, AUDIO_GETINFO, &a->audio_info) < 0)
         HError(6006,"InitAudi: Cannot interrogate Sun audio input");
      if (*sampPeriod == 0 )
         *sampPeriod = 1.0E+07 / (float) a->audio_info.record.sample_rate;
      AUDIO_INITINFO(&a->audio_info);
      f = (int) (1.0E+07 / *sampPeriod);
      a->audio_info.record.sample_rate = f;
      a->audio_info.record.channels = 1;
      a->audio_info.record.precision = 16;
      a->audio_info.record.encoding = AUDIO_ENCODING_LINEAR;
      a->audio_info.record.pause = 1;
      a->audio_info.record.port = 
         (lineIn?AUDIO_LINE_IN:0)+
         (micIn?AUDIO_MICROPHONE:0);
      if (ioctl(a->audio_ctld, AUDIO_SETINFO, &a->audio_info) < 0) {
         g = TrimSampFreq(f);
         if (f==g)
            HError(6006,"InitAudi: Cannot initialise Sun audio input");
         a->audio_info.record.sample_rate = g;
         if (ioctl(a->audio_ctld, AUDIO_SETINFO, &a->audio_info) < 0)
            HError(6006,"InitAudi: Cannot initialise Sun audio input");
         HError(-6006,"InitAudi: adjusting sampling frequency %d -> %d", f, g);
      }
      ioctl(a->audio_ctld, FIONBIO, &i);
      ioctl(a->audio_ctld, I_FLUSH, FLUSHR);
      a->numSamples=a->audio_info.record.samples;
   }
#endif
#ifdef HPUX_AUDIO
   {
      long st;
      AudioAttrMask mask;
      
      if (audio_dev==NULL)
         audio_dev=AOpenAudio(NULL,&st);
      else
         st=AENoError;
      a->audio=audio_dev;
      audio_cnt++;
      if (a->audio==NULL || st!=AENoError)
         HError(6006,"InitAudi: Cannot open HP audio input ERR=%d",st);
      a->attr=*ABestAudioAttributes(a->audio);
      a->attr.type=ATSampled;
      a->attr.attr.sampled_attr.data_format = ADFLin16;
      a->attr.attr.sampled_attr.bits_per_sample = 16;
      if (*sampPeriod == 0.0)
         *sampPeriod= 1.0E+07 / a->attr.attr.sampled_attr.sampling_rate;
      else
         a->attr.attr.sampled_attr.sampling_rate = (long unsigned int) (1.0E+07 / *sampPeriod);
      a->attr.attr.sampled_attr.channels = 1;
      a->attr.attr.sampled_attr.interleave = 1;
      
      a->gains[0].u.i.in_ch = AICTMono;
      a->gains[0].gain = AUnityGain;
      if (micIn)
         a->gains[0].u.i.in_src = AISTMonoMicrophone;
      if (lineIn)
         a->gains[0].u.i.in_src = AISTMonoAuxiliary;
      
      a->parms.gain_matrix.type = AGMTInput;
      a->parms.gain_matrix.num_entries = 1;
      a->parms.gain_matrix.gain_entries = a->gains;
      a->parms.record_gain = AUnityGain;
      a->parms.event_mask = 0;
      a->parms.pause_first = True;
      
      if ((a->socket = socket(AF_INET,SOCK_STREAM,0))<0)
         HError(6006,"InitAudi: Cannot create HP audio socket");
      mask=ASDataFormatMask|ASBitsPerSampleMask|ASSamplingRateMask|ASChannelsMask|
         ASInterleaveMask;
      a->tid = ARecordSStream(a->audio,mask,&a->attr,&a->parms,&a->stream,&st);
      if (st!=0)
         HError(6006,"InitAudi: Invalid HP audio parameters");
      st=connect(a->socket,&a->stream.tcp_sockaddr,sizeof(struct sockaddr_in));
   }
#endif
#ifdef OSS_AUDIO
   {
      int f, g;
      short *p;
      
      if (*sampPeriod==0) {
         *sampPeriod = 1.0E+07 / 16000.0;
      }
      if (audio_io==0) {
         if ((audio_fd = open(AUDIO_DEV, O_RDONLY, 0)) < 0)
            HError(6006, "InitAudi: Cannot open OSS audio device %s", AUDIO_DEV);
         if ((mixer_fd = open(MIXER_DEV, O_RDONLY, 0)) < 0)
            HError(6006, "InitAudi: unable to open OSS audio mixer %s", MIXER_DEV);
         for (f=0, p=zero_buf; f<BUF_SIZE; f++, p++) *p=0; 
         f = 2;
         if (ioctl(audio_fd, SNDCTL_DSP_SUBDIVIDE, &f) < 0)
            HError(-6006, "InitAudi: error dividing buffer");
      }
      audio_io = audio_io | AUDIO_RD;
      if (ioctl(audio_fd, SNDCTL_DSP_RESET, 0)==-1)
         HError(6006, "InitAudi: error resetting audio device");
      f = g = (IsVAXOrder ()) ? AFMT_S16_LE : AFMT_S16_BE;
      if (ioctl(audio_fd, SNDCTL_DSP_SETFMT, &f) < 0)
         HError(6006, "InitAudi: error setting sample format");
      if (f != g)
         HError(6006, "InitAudi: unable to set 16 bit sample format");
      f = 0;
      if (ioctl(audio_fd, SNDCTL_DSP_STEREO, &f) < 0)
         HError(6006, "InitAudi: error setting mono audio channel");
      f = g = (int) (1.0E+07 / (float) (*sampPeriod));
      if (ioctl(audio_fd, SNDCTL_DSP_SPEED, &f)==-1)
         HError(6006, "InitAudi: error setting sampling rate");
      if (ioctl(audio_fd, SNDCTL_DSP_GETISPACE, &audio_info) < 0)
         HError(6006, "InitAudi: error getting fragment size");
      frag_size = audio_info.fragsize / sizeof(short);
      if (trace&T_AUD) {
         printf("InitAudi: info.fragments   %d\n", audio_info.fragments);
         printf("InitAudi: info.fragsize    %d\n", audio_info.fragsize);
         printf("InitAudi: info.bytes       %d\n", audio_info.bytes);
      }
   }
#endif
#ifdef NO_AUDIO
   *sampPeriod = 0.0;
#endif
   a->isActive = ADS_OPEN;
}

/* CloseAudi: close the given audio input device */
static void CloseAudi(AudioIn a)
{
   if (a->isActive<ADS_OPEN || a->isActive==ADS_CLOSED) return;
   if (trace&T_AUD) {
      printf("Closing Audio Input from %d\n",a->isActive);
      fflush(stdout);
   }
#ifdef MMAPI_AUDIO
   {
      mmApiBuf *p;
      
#ifdef DEC_AUDIO
      /* Only needed with DEC API */
      if (mmeCheckForCallbacks())
         mmeProcessCallbacks();
#endif
      for (p=a->fHead;p!=NULL;p=p->next) {
#ifdef WIN32_AUDIO
         if ((a->mmError=waveInUnprepareHeader(a->waveIn, p->waveHdr,
                                               sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
            HError(6006,"CloseAudi: Header Unpreparation failed");
#endif
         if (mmeFreeMem(p->waveHdr)!=TRUE)
            HError(6006,"CloseAudi: MMAPI Header Free failed");
         if (mmeFreeBuffer(p->waveData)!=TRUE)
            HError(6006,"CloseAudi: MMAPI Data Free failed");
         /* Note mmApiBufs will be freed with AudioOut structure */
      }
      if (mmeFreeMem(a->waveFmt)!=TRUE)
         HError(6006,"CloseAudi: MMAPI wave foramt free failed");
      if (mmeFreeBuffer(a->wavePos)!=TRUE)
         HError(6006,"CloseAudi: MMAPI wave position free failed");
#ifdef DEC_AUDIO
      /* Only needed with DEC API */
      if (mmeCheckForCallbacks())
         mmeProcessCallbacks();
#endif
      if (a->isActive>ADS_SAMPLING)
         if ((a->mmError = waveInClose(a->waveIn))!=MMSYSERR_NOERROR)
            HError(6006,"StopAudi: Could not close MMAPI audio [ERR=%d]",a->mmError);
      if (trace & T_AUD) {
         printf(" Closing MMAPI audio input\n");
         fflush(stdout);
      }
      sAudioIn=NULL;
   }
#endif
#ifdef RS6000_AUDIO          
   if ((a->rc = UMSAudioDevice_close(a->adevin,a->evin))!=rOK)
      HError(6006,"CloseAudi: Cannot close RS6000 audio device");
   _somFree(a->adevin);
#endif   
#ifdef SGI_AUDIO
   ALfreeconfig(a->config);
#endif
#ifdef SUN16_AUDIO
   ioctl(a->audio_ctld, I_FLUSH, FLUSHR);
   close(a->audio_ctld);
   a->numSamples=a->audio_info.record.samples;
   a->audio_ctld=-1;
#endif
#ifdef HPUX_AUDIO
   {
      long st;
      ATransStatus trst;

      close(a->socket);
      AStopAudio(a->audio,a->tid,ASMThisTrans,&trst,&st);      
      a->audio=NULL;
      audio_cnt--;
      if (audio_cnt<=0) {
         ASetCloseDownMode(audio_dev,AKeepTransactions,NULL);
         ACloseAudio(audio_dev,&st);
         audio_cnt=0;audio_dev=NULL;
      }
   }
#endif
#ifdef OSS_AUDIO
   if (audio_io&AUDIO_RD) {
      audio_io^=AUDIO_RD;
      if (audio_io==0) {
         close(audio_fd); 
         close(mixer_fd);
      }
   }
#endif
   a->isActive = ADS_CLOSED;
   a->curVol = 0.0;
}

/* InSamples: return number of bytes in audio device input buffer */
static int InSamples(AudioIn a)
{
   if (a->isActive!=ADS_SAMPLING) return(0);
#ifdef MMAPI_AUDIO
   {
      mmApiBuf *p;
      int inSamps;
      
#ifdef DEC_AUDIO
      /* Only needed with DEC API */
      if (mmeCheckForCallbacks())
         mmeProcessCallbacks();
#endif
#ifdef WIN32_AUDIO
      EnterCriticalSection(&(a->c));
#endif
      for (inSamps=0,p=a->fHead;p!=NULL;p=p->next) inSamps+=p->n;
#ifdef WIN32_AUDIO
      LeaveCriticalSection(&(a->c));
#endif
      return inSamps;
   }
#endif
#ifdef RS6000_AUDIO
   {
      long samps;
      
      samps=0;
      if ((a->rc = UMSAudioDevice_read_buff_used(a->adevin, 
                                                 a->evin,&samps))!=rOK)
         HError(6006,"InSamples: Error accessing RS6000 read buffer");
      return ((int)samps);
   }
#endif
#ifdef SGI_AUDIO
   {
      return ALgetfilled(a->recPort);
   }
#endif
#ifdef SUN16_AUDIO
   {
      int bytes=0;
      
      ioctl(a->audio_ctld, FIONREAD, &bytes);
      return(bytes/2);
   }
#endif
#ifdef HPUX_AUDIO
   {
      int bytes=0;
      
      ioctl(a->socket, FIONREAD, &bytes);
      return(bytes/2);
   }
#endif
#ifdef OSS_AUDIO
   {
      if (ioctl(audio_fd, SNDCTL_DSP_GETISPACE, &audio_info)==-1)
         HError(6006, "InSamples: unable to obtain OSS buffer info.");
      return (audio_info.bytes)/sizeof(short);
   }
#endif
#ifdef NO_AUDIO
   return 0;
#endif
}

/* EXPORT->GetCurrentVol: obtain volume of input source */
float GetCurrentVol(AudioIn a)
{
   return a->curVol;
}

/* EXPORT->SetAudioInput: query/set audio device input */
int AudioDevInput(int *mask) 
{
   int in = HA_IN_NONE;

   if (mask==NULL) {
      if (lineIn) in|=HA_IN_LINE;
      if (micIn)  in|=HA_IN_MIC;
   } else {
      in=*mask;
      micIn = ((in&HA_IN_MIC)?TRUE:FALSE);
      lineIn = ((in&HA_IN_LINE)?TRUE:FALSE);
   }
   return in;
}

/* EXPORT->AudioDevOutput: query/set audio device output */
int AudioDevOutput(int *mask) 
{
   int out = HA_OUT_NONE;

   if (mask==NULL) {
      if (speakerOut) out|=HA_OUT_SPEAKER;
      if (phonesOut)  out|=HA_OUT_PHONES;
      if (lineOut)    out|=HA_OUT_LINE;
   } else {
      out=*mask;
      speakerOut = ((out&HA_OUT_SPEAKER)?TRUE:FALSE);
      phonesOut = ((out&HA_OUT_PHONES)?TRUE:FALSE);
      lineOut = ((out&HA_OUT_LINE)?TRUE:FALSE);
   }
   return out;
}

/* ReadAudio: read nSamples from audio device into buf */
static void ReadAudio(AudioIn a, short *buf, int nSamples)
{
   int i;

   if (a->isActive!=ADS_SAMPLING && a->isActive!=ADS_STOPPED) { 
      for (i=0; i<nSamples; i++) *buf++=0;
      return;
   }
#ifdef MMAPI_AUDIO
   {
      mmApiBuf *cur;
      int n,needed=nSamples;
      short *s=buf;
      
      if (trace & T_AUD) {
         printf(" Reading %d samples from MMAPI audio input\n",nSamples);
         fflush(stdout);
      }
      do {
#ifdef WIN32_AUDIO
         EnterCriticalSection(&a->c);
#endif
         while (a->fHead==NULL) {
            if (needed<=0) break;
            /* Need to wait for callback */
#ifdef WIN32_AUDIO
            LeaveCriticalSection(&a->c);
            WaitForSingleObject(a->callBackEvent, INFINITE);
            ResetEvent(a->callBackEvent);
            EnterCriticalSection(&a->c);
#endif
#ifdef DEC_AUDIO
            mmeWaitForCallbacks();
            mmeProcessCallbacks();
#endif
         }
         cur=a->fHead;
#ifdef WIN32_AUDIO
         /* Contents of fHead are not going to change */
         LeaveCriticalSection(&a->c);
#endif
         if (cur==NULL || needed<=0) break; /* End or have enough already */
         if (cur->n<needed) n=cur->n;
         else n=needed;
         memcpy(s,((short*)cur->waveData)+cur->cur,n*sizeof(short));
         s+=n; cur->cur+=n; cur->n-=n; needed-=n;
         if (cur->n<=0) {
#ifdef WIN32_AUDIO
            EnterCriticalSection(&a->c);
#endif
            /* Remove from filled list */
            if (cur->next==NULL) {
               a->fHead=a->fTail=NULL;  /* No more to process */
            }
            else {
               a->fHead=cur->next; cur->next->prev=NULL;
            }
            /* Add to ready queue */
            cur->index=-1;
            if (a->qTail==NULL) {
               cur->next=cur->prev=NULL;
               a->qTail=a->qHead=cur;
            }
            else {
               cur->next=NULL; cur->prev=a->qTail;
               a->qTail->next=cur; a->qTail=cur;
            }
            /* This may be necessary */
#ifdef WIN32_AUDIO
            if ((a->mmError=waveInUnprepareHeader(a->waveIn, cur->waveHdr,
                                                  sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
               HError(6006,"CloseAudi: Header Unpreparation failed");
#endif
            cur->waveHdr->lpData = cur->waveData;
            cur->waveHdr->dwBufferLength = cur->size;
            cur->waveHdr->dwBytesRecorded = 0;
            cur->waveHdr->dwUser = ++a->total;
            cur->waveHdr->dwFlags = 0;
            cur->waveHdr->dwLoops = 0;
#ifdef WIN32_AUDIO
            if ((a->mmError=waveInPrepareHeader(a->waveIn, cur->waveHdr,
                                                sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
               HError(6006,"CloseAudi: Header Preparation failed");
#endif
            
            if ((a->mmError=waveInAddBuffer(a->waveIn, cur->waveHdr,
                                            sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
               HError(6006,"ReadAudio: Buffer addition failed");
#ifdef WIN32_AUDIO
            /* Contents of fHead are not going to change */
            LeaveCriticalSection(&a->c);
#endif
         }
      }
      while(needed>0);
   }
#endif
#ifdef RS6000_AUDIO
   {
      UMSAudioTypes_Buffer ainBuf; 
      
      /* NOTE, ainBuf stores internally as unsigned char hence cast below */
      ainBuf._length = nSamples*sizeof(short);   
      ainBuf._maximum = nSamples*sizeof(short);
      ainBuf._buffer = (unsigned char *)buf;
      if ((a->rc=UMSAudioDevice_read(a->adevin,a->evin,&ainBuf,
                                     nSamples,&(a->sw)))!=rOK)
         HError(6006,"ReadAudio: Cannot read from RS6000 audio device");
      if ((a->sw!=nSamples) && !stopSignalled)
         HError(-6006,
                "ReadAudio: Failed to read all %d samples from RS6000 audio",nSamples);
   }
#endif
#ifdef SGI_AUDIO
   ALreadsamps(a->recPort, buf, nSamples);
#endif
#ifdef SUN16_AUDIO
   {
      int i,n;
      char *ptr;
      
      n=nSamples*sizeof(short);ptr=(char*)buf;
      do {
         if ((i = read(a->audio_ctld,ptr,n)) <= 0)
            break;
         ptr+=i;
         n-=i;
      }
      while (n!=0);
      if (i<=0 && !stopSignalled)
         HError(-6006,"ReadAudio: Failed to read all %d samples from Sun audio",
                nSamples);
      a->numSamples+=nSamples;
   }
#endif
#ifdef HPUX_AUDIO
   {
      int n;
      char *ptr;
      
      n=nSamples*sizeof(short);ptr=(char*)buf;
      do {
         if ((i = read(a->socket,ptr,n)) <= 0) {
            break;
         }
         ptr+=i;
         n-=i;
      }
      while (n!=0);
      if (i<=0 && !stopSignalled)
         HError(-6006,"ReadAudio: Failed to read all %d samples from HP audio",
                nSamples);
      while(n>0) *ptr++=0,n--;
   }
#endif
#ifdef OSS_AUDIO
   {
      int n;
      char *ptr;
      
      n = nSamples*sizeof(short); ptr=(char*)buf;
      do {
         if ((i = read(audio_fd, ptr, n)) <= 0) {
            break;
         }
         ptr += i; n -= i;
      } while (n!=0);
      if (i<=0 && !stopSignalled)
         HError(-6006, "ReadAudio: Failed to read all %d samples from OSS audio", nSamples);
      while(n>0) *ptr++=0,n--;
   }
#endif
   if (nSamples > 64) 
      a->curVol = CalcVolume(buf, nSamples);
}

/* StartAudi: start the audio input device sampling */
static void StartAudi(AudioIn a)
{
   a->curVol = 0;
   if (a->isActive!=ADS_OPEN) return;
   if (trace&T_AUD) {
      printf("Starting Audio Input from %d\n",a->isActive);
      fflush(stdout);
   }
#ifdef MMAPI_AUDIO
   {
      int i;
      mmApiBuf *p;
      
      if ((a->mmError=waveInOpen(NULL, WAVE_MAPPER, (LPWAVEFORMAT)a->waveFmt,
                                 NULL,0,WAVE_FORMAT_QUERY))!=MMSYSERR_NOERROR)
         HError(6006,"StartAudi: Requested data format is not supported [ERR=%d]",a->mmError);
      if ((a->mmError=waveInOpen(&a->waveIn, WAVE_MAPPER,(LPWAVEFORMAT)a->waveFmt,
                                 callBackIn,a->magic,CALLBACK_FUNCTION))!=MMSYSERR_NOERROR)
         HError(6006,"StartAudi: Cannot open MMAPI audio input [ERR=%d]",a->mmError);
      
      a->total=0;
      for (i=0;i<MMAPI_BUFFER_COUNT;i++) {
         /* Add new buffer to end of list */
         p=New(a->mem,sizeof(mmApiBuf));
         p->waveHdr = mmeAllocMem(sizeof(WAVEHDR));
         p->cur=p->n=0; p->next=p->prev=NULL;
         
         p->size = a->bufSize;
         p->waveData = mmeAllocBuffer(p->size);
         if(p->waveData==NULL)
            HError(6006,"StartAudi: Unable to allocate audio buffer");
         p->index = -1; p->cur=-1; p->n=0;
         
         /* Set up header */
         p->waveHdr->lpData = p->waveData;
         p->waveHdr->dwBufferLength = p->size;
         p->waveHdr->dwBytesRecorded = 0; /* Unused */
         p->waveHdr->dwUser = ++a->total;
         p->waveHdr->dwFlags = 0;
         p->waveHdr->dwLoops = 0;
#ifdef WIN32_AUDIO
         if ((a->mmError=waveInPrepareHeader(a->waveIn, p->waveHdr, 
                                             sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
            HError(6006,"StartAudi: Header preparation failed");
#endif
         
         /* Add in */
         if (a->qTail==NULL) a->qHead=a->qTail=p;
         else a->qTail->next=p,p->prev=a->qTail,a->qTail=p;
         
         if ((a->mmError=waveInAddBuffer(a->waveIn, p->waveHdr, 
                                         sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
            HError(6006,"StartAudi: Cannot add input buffer %d",a->mmError);
      }
      
      /* And record */
      a->qHead->index=a->current=1;
   
#ifdef DEC_AUDIO
      /* Only needed with DEC API */
      if (mmeCheckForCallbacks())
         mmeProcessCallbacks();
#endif
      if ((a->mmError=waveInStart( a->waveIn ))!=MMSYSERR_NOERROR) 
         HError(6006,"StartAudi: Cannot start MMAPI input audio port [ERR=%d]",a->mmError);
#ifdef WIN32_AUDIO
      ResetEvent(a->callBackEvent);
#endif
   }
#endif
#ifdef RS6000_AUDIO   
   {
      if ((a->rc = UMSAudioDevice_start(a->adevin,a->evin))!=rOK)
         HError(6006,"StartAudi: Cannot start RS6000 audio input device");
   }
#endif
#ifdef SGI_AUDIO
   {
      if ((a->recPort=ALopenport("HAudio_input","r",a->config))==NULL)
         HError(6006,"StartAudi: Cannot open SGI input audio port");
   }
#endif
#ifdef SUN16_AUDIO
   {
      ioctl(a->audio_ctld, I_FLUSH, FLUSHR);
      ioctl(a->audio_ctld, AUDIO_GETINFO, &a->audio_info);
      a->audio_info.record.pause=0;
      ioctl(a->audio_ctld, AUDIO_SETINFO, &a->audio_info);
   }
#endif
#ifdef HPUX_AUDIO
   {
      long st;
      ATransStatus trst;
      AGetTransStatus(a->audio,a->tid,&trst,&st);
      
      AResumeAudio(a->audio,a->tid,&trst,&st);
   }
#endif
#ifdef OSS_AUDIO
   { 
      short sample;
      
      if (read(audio_fd, &sample, sizeof(short))!=sizeof(short))
         HError(6006, "StartAudi: Unable to start OSS audio port");
   }
#endif
   a->isActive = ADS_SAMPLING;
}

/* StopAudi: stop the audio input device sampling */
static void StopAudi(AudioIn a)
{
   if (a->isActive!=ADS_SAMPLING) return;
   if (trace&T_AUD) {
      printf("Stopping Audio Input from %d\n",a->isActive);
      fflush(stdout);
   }
#ifdef MMAPI_AUDIO
   {
      if ((a->mmError=waveInStop(a->waveIn))!=MMSYSERR_NOERROR ||
          (a->mmError=waveInReset(a->waveIn))!=MMSYSERR_NOERROR)
         HError(6006,"StopAudi: Cannot stop MMAPI input audio port [ERR=%d]",a->mmError);
   }
#endif
#ifdef RS6000_AUDIO         
   if ((a->rc = UMSAudioDevice_stop(a->adevin,a->evin))!=rOK)
      HError(6006,"StopAudi: Cannot stop RS6000 audio input device");
#endif
#ifdef SGI_AUDIO
   ALcloseport(a->recPort);
#endif
#ifdef SUN16_AUDIO
   ioctl(a->audio_ctld, AUDIO_GETINFO, &a->audio_info);
   a->audio_info.record.pause=1;
   ioctl(a->audio_ctld, AUDIO_SETINFO, &a->audio_info);
#endif
#ifdef HPUX_AUDIO
   {
      long st;
      ATransStatus trst;
      APauseAudio(a->audio,a->tid,&trst,&st);
      /*
        AStopAudio(a->audio,a->tid,ASMThisTrans,&trst,&st);
      */
   }
#endif
#ifdef OSS_AUDIO
   if (ioctl(audio_fd, SNDCTL_DSP_RESET, 0) < 0)
      HError(6006, "StopAudi: error stopping OSS input audio port");
#endif
   a->isActive = ADS_STOPPED;
   a->curVol = 0;
}


#ifdef MMAPI_AUDIO
static int OutSamples(AudioOut a);
static AudioOut sAudioOut=NULL;

void CALLBACK callBackOut(HWAVE hwaveIn, UINT msg, 
                          DWORD magic, DWORD fullBufHdr, DWORD c)
{
   AudioOut a;
   mmApiBuf *p;

   a = sAudioOut;
   if (a==NULL || a->magic!=magic) return;
   switch(msg) {
   case MM_WOM_DONE:
      for (p=a->pHead;p!=NULL;p=p->next)
         if (p->index==a->current) break;
      if (p!=NULL) a->total-=p->n;
      if (p!=NULL && p->next!=NULL) a->current=p->next->index;
      else a->current=-1; /* Done or fail */
      break;
   default:
      break;
   }
}
#endif

/* InitAudo: initialise the given audio output device */
static void InitAudo(AudioOut a, HTime *sampPeriod)
{
#ifdef MMAPI_AUDIO
   {
      /* Initialise */
      if (sAudioOut!=NULL)
         HError(6006,"InitAudo: MMAPI audio output already in use");
      sAudioOut=a;
      a->total=0; a->current=-1; a->pHead=a->pTail=NULL; 
      a->magic=sMagic; sMagic+=12345;

      /* Allocate special structures */
      a->waveFmt = mmeAllocMem(sizeof(PCMWAVEFORMAT));
      a->wavePos = mmeAllocMem(sizeof(MMTIME));

      /* Set up required format */
      a->waveFmt->wf.wFormatTag = WAVE_FORMAT_PCM;
      a->waveFmt->wf.nChannels = 1;
      if(*sampPeriod == 0.0){
         *sampPeriod = 1.0E+07 / (float)16000;
         a->waveFmt->wf.nSamplesPerSec = 16000;
      }else
         a->waveFmt->wf.nSamplesPerSec = 1.0E+07 / *sampPeriod;
      a->waveFmt->wf.nBlockAlign = sizeof(short);
      a->waveFmt->wf.nAvgBytesPerSec = 
         a->waveFmt->wf.nBlockAlign*a->waveFmt->wf.nSamplesPerSec;
      a->waveFmt->wBitsPerSample = 16;

      /* Set up position query */
      a->wavePos->wType = TIME_SAMPLES;

      /* Open wave device */
      if ((a->mmError=waveOutOpen(&a->waveOut, WAVE_MAPPER, 
                                  (LPWAVEFORMAT)a->waveFmt, callBackOut, a->magic,
                                  CALLBACK_FUNCTION))!=MMSYSERR_NOERROR)
         HError(6006,"InitAudo: Cannot open MMAPI audio output [ERR=%d]",a->mmError);
#ifdef DEC_AUDIO
      if ((a->mmError=waveOutGetID(a->waveOut, 
                                   &a->waveOutDev))!=MMSYSERR_NOERROR)
         HError(6006,"InitAudo: Cannot get MMAPI audio output device number [ERR=%d]",a->mmError);
      /* Only needed with DEC API */
      if (mmeCheckForCallbacks())
         mmeProcessCallbacks();
#endif
      if (trace & T_AUD) {
         printf(" Initialised MMAPI audio output at %.2fkHz\n",
                a->waveFmt->wf.nSamplesPerSec*1E-3);
         fflush(stdout);
      }
   }
#endif
#ifdef RS6000_AUDIO                            
   {
      long sampRate;
      
      a->evout = somGetGlobalEnvironment();
      a->adevout = UMSBAUDDeviceNew();
      if ((a->rc = UMSAudioDevice_open(a->adevout,a->evout,AUDIO_DEV,"PLAY",
                                       UMSAudioDevice_BlockingIO))!=rOK)
         HError(6006,"InitAudo: Cannot open RS6000 audio output device");
      a->channels = 1;
      a->bits  = 16 ;
      if (*sampPeriod == 0.0){
         UMSAudioDevice_get_sample_rate(a->adevout,a->evout,&sampRate); 
         *sampPeriod= 1.0E+07 / sampRate;                  
      }
      else
         sampRate = 1.0E+07 / *sampPeriod;
      a->rc = UMSAudioDevice_set_sample_rate(a->adevout,a->evout,sampRate,&a->osamples);
      a->rc = UMSAudioDevice_set_bits_per_sample(a->adevout,a->evout,a->bits);
      a->rc = UMSAudioDevice_set_number_of_channels(a->adevout,a->evout,a->channels);
      a->rc = UMSAudioDevice_set_audio_format_type(a->adevout,a->evout,"PCM");
      a->rc = UMSAudioDevice_set_byte_order(a->adevout,a->evout,"MSB");
      a->rc=UMSAudioDevice_set_number_format(a->adevout,a->evout,"TWOS COMPLEMENT");
      a->rc = UMSAudioDevice_get_byte_order(a->adevout,a->evout,&a->obyte_order);
      /* you have to free the string after the query */
      if (a->obyte_order) free(a->obyte_order);
      a->rc = UMSAudioDevice_set_balance(a->adevout,a->evout,0);
      a->rc=UMSAudioDevice_set_time_format(a->adevout,a->evout,UMSAudioTypes_Samples);
      a->lgain = 100; /*maximum left input gain*/
      a->rgain = 100; /*maimum right input gain*/
      if (lineOut)
         strcpy(a->outConn, "LINE_OUT");
      if (phonesOut)
         strcpy(a->outConn, "LINE_OUT"); /* Don't have a phones out in RS6000 */
      if (speakerOut)
         strcpy(a->outConn, "INTERNAL_SPEAKER");
      a->rc = UMSAudioDevice_enable_output(a->adevout,a->evout,a->outConn,
                                           &a->lgain,&a->rgain);
      a->rc = UMSAudioDevice_initialize(a->adevout,a->evout);
   }
#endif   
#ifdef SGI_AUDIO
   {
      a->params[0] = AL_OUTPUT_RATE;
      a->params[2] = AL_LEFT_SPEAKER_GAIN;
      a->params[4] = AL_RIGHT_SPEAKER_GAIN;
      
      ALgetparams(AL_DEFAULT_DEVICE, a->params, 6);
      if (*sampPeriod == 0 ) 
         *sampPeriod =  1.0E+07 / (float) a->params[1] + 0.5;
      else {
         a->params[1] = (long int) (1.0E+07 / *sampPeriod + 0.5);
         ALsetparams(AL_DEFAULT_DEVICE,a->params,2);
      }
      a->config = ALnewconfig();
      ALsetchannels(a->config, AL_MONO);
      ALsetwidth(a->config, AL_SAMPLE_16);
      if ((a->playPort = ALopenport("HAudio_output", "w", a->config)) == NULL)
         HError(6006,"InitAudo: Cannot initialise SGI output audio port");
   }
#endif
#ifdef SUN16_AUDIO
   {
      int f,g;
      
      AUDIO_INITINFO(&a->audio_info);
      if ((a->audio_ctld = open(AUDIO_IO, O_WRONLY + O_NDELAY)) < 0) 
         HError(6006,"InitAudo: Cannot open Sun audio output%s",
                (errno==EBUSY?" [Already in use]":""));
      if (ioctl(a->audio_ctld, AUDIO_GETINFO, &a->audio_info) < 0)
         HError(6006,"InitAudo: Cannot interrogate Sun audio output");
      if (*sampPeriod == 0 )
         *sampPeriod = 1.0E+07 / (float) a->audio_info.play.sample_rate;
      AUDIO_INITINFO(&a->audio_info);
      f = (int) (1.0E+07 / *sampPeriod);
      a->audio_info.play.sample_rate = f;
      a->audio_info.play.channels = 1;
      a->audio_info.play.precision = 16;
      a->audio_info.play.balance = AUDIO_MID_BALANCE;
      a->audio_info.play.encoding = AUDIO_ENCODING_LINEAR;
      a->audio_info.play.port = 
         (lineOut?AUDIO_LINE_OUT:0)+
         (speakerOut?AUDIO_SPEAKER:0)+
         (phonesOut?AUDIO_HEADPHONE:0);
      if (ioctl(a->audio_ctld, AUDIO_SETINFO, &a->audio_info) < 0) {
         g = TrimSampFreq(f);
         if (f==g)
            HError(6006,"InitAudo: Cannot initialise Sun audio output [%d]",errno);
         a->audio_info.play.sample_rate = g;
         if (ioctl(a->audio_ctld, AUDIO_SETINFO, &a->audio_info) < 0)
            HError(6006,"InitAudo: Cannot initialise Sun audio output [%d]",errno);
         HError(-6006,"InitAudo: adjusting sampling frequency %d -> %d", f, g);
      }
      a->numSamples = a->numWrites = 0;
   }
#endif
#ifdef HPUX_AUDIO
   {
      long st;
      int i;
      ATransStatus atst;
      AudioAttributes *attr;
      AudioAttrMask mask;
      
      if (audio_dev==NULL)
         audio_dev=AOpenAudio(NULL,&st);
      else
         st=AENoError;
      a->audio=audio_dev;
      audio_cnt++;
      if (a->audio==NULL || st!=AENoError)
         HError(6006,"InitAudo: Cannot open HP audio output ERR=%d",st);
      ASetCloseDownMode(a->audio,AKeepTransactions,NULL);
      a->attr=*ABestAudioAttributes(a->audio);
      a->attr.type=ATSampled;
      a->attr.attr.sampled_attr.data_format = ADFLin16;
      a->attr.attr.sampled_attr.bits_per_sample = 16;
      if (*sampPeriod == 0.0)
         *sampPeriod= 1.0E+07 / a->attr.attr.sampled_attr.sampling_rate;
      else
         a->attr.attr.sampled_attr.sampling_rate = (long unsigned int) (1.0E+07 / *sampPeriod);
      a->attr.attr.sampled_attr.channels = 1;
      a->attr.attr.sampled_attr.interleave = 1;
      
      i=0;
      if (speakerOut) {
         a->gains[i].u.o.out_ch = AOCTMono;
         a->gains[i].gain = AUnityGain;
         a->gains[i].u.o.out_dst = AODTMonoIntSpeaker;
         i++;
      }
      if (phonesOut) {
         a->gains[i].u.o.out_ch = AOCTMono;
         a->gains[i].gain = AUnityGain;
         a->gains[i].u.o.out_dst = AODTMonoHeadphone;
         i++;
      }
      if (lineOut || i==0) {
         a->gains[i].u.o.out_ch = AOCTMono;
         a->gains[i].gain = AUnityGain;
         a->gains[i].u.o.out_dst = AODTMonoJack;
         i++;
      }
      a->parms.gain_matrix.type = AGMTOutput;
      a->parms.gain_matrix.num_entries = i;
      a->parms.gain_matrix.gain_entries = a->gains;
      a->parms.play_volume = AUnityGain;
      a->parms.priority = APriorityNormal;
      a->parms.event_mask = 0;
   }
#endif
#ifdef OSS_AUDIO
   {
      int f, g;
      short *p;
      
      if (audio_io==0) {
         if ((audio_fd = open(AUDIO_DEV, O_WRONLY, 0)) < 0)
            HError(6006, "InitAudo: Cannot open OSS audio device %s", AUDIO_DEV);
         if ((mixer_fd = open(MIXER_DEV, O_WRONLY, 0)) < 0)
            HError(6006, "InitAudo: unable to open OSS audio mixer %s", MIXER_DEV);
         for (f=0, p=zero_buf; f<BUF_SIZE; f++, p++) *p=0;
         f = 2;
         if (ioctl(audio_fd, SNDCTL_DSP_SUBDIVIDE, &f) < 0)
            HError(-6006, "InitAudo: error dividing buffer");
      }
      audio_io = audio_io | AUDIO_WR;
      if (ioctl (audio_fd, SNDCTL_DSP_SYNC, NULL) < 0) {
         HError(6006, "InitAudo: unable to sync audio device");
      }     
      f = g = (IsVAXOrder ()) ? AFMT_S16_LE : AFMT_S16_BE;
      if (ioctl(audio_fd, SNDCTL_DSP_SETFMT, &f) < 0)
         HError(6006, "InitAudo: error setting sample format");
      if (f != g)
         HError(6006, "InitAudo: unable to set 16 bit sample format");
      f = 0;
      if (ioctl(audio_fd, SNDCTL_DSP_STEREO, &f) < 0)
         HError(6006, "InitAudo: error setting audio channel");
      f = g = (int) (1.0E+07 / (float) (*sampPeriod));
      if (ioctl(audio_fd, SNDCTL_DSP_SPEED, &f)==-1)
         HError(6006, "InitAudo: error setting sampling rate");
      if (ioctl(audio_fd, SNDCTL_DSP_GETOSPACE, &audio_info) < 0)
         HError(6006, "InitAudo: error getting fragment size");
      frag_size = audio_info.fragsize / sizeof(short);
      if (trace&T_AUD) {
         printf("InitAudo: info.fragments   %d\n", audio_info.fragments);
         printf("InitAudo: info.fragsize    %d\n", audio_info.fragsize);
         printf("InitAudo: info.bytes       %d\n", audio_info.bytes);
      }
   }
#endif
#ifdef NO_AUDIO
   *sampPeriod = 0.0;
#endif
   a->isActive = FALSE;
}

/* CloseAudo: close the given audio output device */
static void CloseAudo(AudioOut a)
{
#ifdef MMAPI_AUDIO 
   {
      mmApiBuf *p;

      while(a->current>=0) {
#ifdef DEC_AUDIO
         /* Only needed with DEC API */
         mmeWaitForCallbacks();
         mmeProcessCallbacks();
#endif
         if (OutSamples(a)==0) break;
      }  /* Block until finished playing */
      if((a->mmError=waveOutReset(a->waveOut))!=MMSYSERR_NOERROR)
         HError(6006,"CloseAudo: Cannot reset MMAPI output audio device [ERR=%d]",a->mmError);
      for(p=a->pHead;p!=NULL;p=p->next) {
#ifdef WIN32_AUDIO
         if((a->mmError=waveOutUnprepareHeader(a->waveOut, p->waveHdr, 
                                               sizeof(WAVEHDR))) != MMSYSERR_NOERROR)
            HError(6006,"CloseAudo: MMAPI Header unpreparation failed [ERR=%d]",a->mmError);
#endif
         if (mmeFreeMem(p->waveHdr)!=TRUE)
            HError(6006,"CloseAudo: MMAPI Header Free failed");
         if (mmeFreeMem(p->waveData)!=TRUE)
            HError(6006,"CloseAudo: MMAPI Data Free failed");
         /* Note mmApiBufs will be freed with AudioOut structure */
      }
      a->pHead=a->pTail=NULL;
      if((a->mmError=waveOutClose( a->waveOut )) != MMSYSERR_NOERROR) 
         HError(6006,"CloseAudo: Cannot close MMAPI output audio device [ERR=%d]",a->mmError);
      if (mmeFreeMem(a->waveFmt)!=TRUE)
         HError(6006,"CloseAudo: MMAPI wave foramt free failed");
      if (mmeFreeBuffer(a->wavePos)!=TRUE)
         HError(6006,"CloseAudo: MMAPI wave position free failed");
#ifdef DEC_AUDIO
      /* Only needed with DEC API */
      if (mmeCheckForCallbacks())
         mmeProcessCallbacks();
#endif
      if (trace & T_AUD) {
         printf(" Closing MMAPI audio output\n");
         fflush(stdout);
      }
      sAudioOut=NULL;
   }
#endif
#ifdef RS6000_AUDIO     
   {
      UMSAudioDevice_stop(a->adevout,a->evout);
      UMSAudioDevice_close(a->adevout,a->evout);
      _somFree(a->adevout);
   }
#endif   
#ifdef SGI_AUDIO
   {
      ALfreeconfig(a->config);
      ALcloseport(a->playPort);
   }
#endif
#ifdef SUN16_AUDIO
   {
      close(a->audio_ctld);
      a->audio_ctld=-1;
   }
#endif
#ifdef HPUX_AUDIO
   {
      long st;
      a->audio=NULL;
      audio_cnt--;
      if (audio_cnt<=0) {
         ASetCloseDownMode(audio_dev,AKeepTransactions,NULL);
         ACloseAudio(audio_dev,&st);
         audio_cnt=0;audio_dev=NULL;
      }
   }
#endif
#ifdef OSS_AUDIO
   {
      if (audio_io&AUDIO_WR) {
         audio_io^=AUDIO_WR;
         if (audio_io==0) {
            if (ioctl (audio_fd, SNDCTL_DSP_SYNC, NULL) < 0) {
               HError(6006, "CloseAudo: unable to sync audio device");
            }     
            close(audio_fd); 
            close(mixer_fd);
         }
      }
   }
#endif
   a->isActive = FALSE;
}

/* OutSamples: return num samples left to play in output device */
static int OutSamples(AudioOut a)
{
   if (!a->isActive) return 0;
#ifdef MMAPI_AUDIO
   {
#ifdef DEC_AUDIO
      /* Only needed with DEC API */
      if (mmeCheckForCallbacks())
         mmeProcessCallbacks();
#endif
      if (a->current<0) return(0);
      if((a->mmError=waveOutGetPosition( a->waveOut, a->wavePos,
                                         sizeof(MMTIME))) != MMSYSERR_NOERROR) 
         HError(6006,"OutSamples: Cannot get current play back position");
      return(a->total - a->wavePos->u.sample);
   }
#endif
#ifdef RS6000_AUDIO
   {
      long samps=0;
      
      if ((a->rc=UMSAudioDevice_write_buff_used(a->adevout,a->evout,&samps))!=rOK){
         if (a->rc==rUR)
            samps=0; 
         else
            if (a->rc==rOR)
               samps=0;
            else 
               HError(6006,"OutSamples: Cannot access write buffer on RS6000 audio output device");
      }
      return ((int)samps);
   }
#endif
#ifdef SGI_AUDIO
   return ALgetfilled(a->playPort);
#endif
#ifdef SUN16_AUDIO
   ioctl(a->audio_ctld, AUDIO_GETINFO, &a->audio_info);
   /* Since play.samples can be wrong we need to fix it at the end */
   if (a->audio_info.play.eof==a->numWrites)
      a->numSamples=a->audio_info.play.samples;
   return(a->numSamples-a->audio_info.play.samples);
#endif
#ifdef HPUX_AUDIO
   {
      long st;
      ATransStatus atst;
      
      atst.time.type=ATTSamples;
      AGetTransStatus(a->audio,a->tid,&atst,&st);
      if (st!=AENoError || atst.state==ATSStopped) return(0);
      return(a->nToPlay-atst.time.u.samples);
   }
#endif
#ifdef OSS_AUDIO
   if (ioctl (audio_fd, SNDCTL_DSP_SYNC, NULL) < 0)
      HError(6006, "InitAudo: unable to sync audio device");
   return 0;
#endif
#ifdef NO_AUDIO
   return(0);
#endif
}

/* PlayAudio: play nSamples from buf thru output device a */
static void PlayAudio(AudioOut a, short *buf, int nSamples)
{
#ifdef MMAPI_AUDIO
   {
      mmApiBuf *p,*c;
      int i;
      
      if (trace & T_AUD) {
         printf(" Playing %d samples with MMAPI audio output\n",nSamples);
         fflush(stdout);
      }
      /* Find end of current list */
      for (c=a->pHead;c!=NULL && c->next!=NULL;c=c->next);

      /* Add new buffer to end of list */
      p=New(a->mem,sizeof(mmApiBuf));
      p->waveHdr = mmeAllocMem(sizeof(WAVEHDR));
      p->cur=0; p->n=nSamples; p->next=p->prev=NULL;
      p->size = nSamples*sizeof(short);
      p->waveData = mmeAllocMem(p->size);
      memcpy(p->waveData,buf,p->size);
      p->index = (c==NULL ? 1 : c->index+1); 

      /* Set up header */
      p->waveHdr->lpData = p->waveData;
      p->waveHdr->dwBufferLength = p->size;
      p->waveHdr->dwBytesRecorded = 0; /* Unused */
      p->waveHdr->dwUser = p->index;
      p->waveHdr->dwFlags = 0;
      p->waveHdr->dwLoops = 0;
#ifdef WIN32_AUDIO
      if ((a->mmError=waveOutPrepareHeader(a->waveOut, p->waveHdr, 
                                           sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
         HError(6006,"PlayAudo: Header preparation failed");
#endif

      /* Add in */
      a->total+=nSamples;
      if (c==NULL) a->pHead=a->pTail=p;
      else c->next=p,a->pTail=p;

      /* And play */
      if (a->current<0) a->current=p->index; /* Playing starts now */
      if((a->mmError=waveOutWrite(a->waveOut, p->waveHdr, 
                                  sizeof(WAVEHDR)))!=MMSYSERR_NOERROR)
         HError(6006,"PlayAudo: Failed to write to audio output device");
   }
#endif
#ifdef RS6000_AUDIO
   {
      int i; 
      long samps=0;
      UMSAudioTypes_Buffer aoutBuf; 
      
      /* Note that the data is stored internally as unsigned char */    
      aoutBuf._length = nSamples*sizeof(short);
      aoutBuf._maximum = nSamples*sizeof(short);
      aoutBuf._buffer = (unsigned char *)buf;
      UMSAudioDevice_start(a->adevout, a->evout);   
      
      if ((a->rc = UMSAudioDevice_write(a->adevout,a->evout,&aoutBuf,
                                        nSamples,&a->sw))!=rOK)
         HError(6006,"PlayAudi: Cannot write to RS6000 audio output device");
   }
#endif
#ifdef SGI_AUDIO
   ALwritesamps(a->playPort, buf, nSamples);
#endif
#ifdef SUN16_AUDIO
   {
      int n,i;
      char *ptr;
      ptr = (char *) buf; n = nSamples*sizeof(short);
      while (n>0) {
         if ((i=write(a->audio_ctld,ptr,n))<=0)
            HError(6006,"PlayAudio: Failed to write to Sun audio");
         ptr+=i;n-=i;
      }
      write(a->audio_ctld,buf,0);  /* Increments eof counter */
      a->numSamples+=nSamples;a->numWrites++;
   }
#endif
#ifdef HPUX_AUDIO
   {
      int n,i;
      long st;
      AudioAttrMask mask;
      ATransStatus atst;
      
      if ((a->socket = socket(AF_INET,SOCK_STREAM,0))<0)
         HError(6006,"PlayAudio: Cannot create HP audio socket");
      mask=ASDataFormatMask|ASBitsPerSampleMask|ASSamplingRateMask|ASChannelsMask|
         ASInterleaveMask;
      a->tid = APlaySStream(a->audio,mask,&a->attr,&a->parms,&a->stream,&st);
      if (st!=0)
         HError(6006,"PlayAudio: Invalid HP audio parameters");
      st=connect(a->socket,&a->stream.tcp_sockaddr,sizeof(struct sockaddr_in));
      n=nSamples*sizeof(short);
      while(n>0) {
         if ((i=write(a->socket,buf,n))<0)
            HError(6006,"PlayAudio: Only wrote %d/%d samples to HP audio",
                   n/2,nSamples);
         buf+=i;
         n-=i;
      }
      close(a->socket);
      a->nToPlay=nSamples;
   }
#endif   
#ifdef OSS_AUDIO
   {
      char *ptr;
      int k, n;
      
      n=nSamples*sizeof(short); ptr=(char*)buf;
      while(n>0) {
         if ((k=write(audio_fd, ptr, n))<=0)
            HError(6006,"PlayAudio: Could not write to OSS audio");
         ptr+=k; n-=k;
      }
   }
#endif 
#ifdef NO_AUDIO
   if(nSamples && buf[0]); /* keep compiler quiet */
#endif
   a->isActive = TRUE;
}

/* SetVol: set output play level (0-1) of output device */
static void SetVol(AudioOut a, float volume)
{
   if (a==NULL) return;
   if (volume>1.0) volume = 1.0;
#ifdef MMAPI_AUDIO
   {
      DWORD vol;
      vol=65535*volume;vol=((vol<<16)&0xffff0000)|(vol&0x0000ffff);
#ifdef DEC_AUDIO
      if ((a->mmError=waveOutSetVolume(a->waveOutDev,
                                       (DWORD)vol))!=MMSYSERR_NOERROR)
         HError(6006,"SetVol: Failed to set MMAPI volume [ERR=%d]",a->mmError);
#endif
#ifdef WIN32_AUDIO
      /* Windows lets you set the instance volume */
      if ((a->mmError=waveOutSetVolume(a->waveOut,
                                       (DWORD)vol))!=MMSYSERR_NOERROR)
         HError(6006,"SetVol: Failed to set MMAPI volume [ERR=%d]",a->mmError);
#endif
   }
#endif
#ifdef RS6000_AUDIO   
   UMSAudioDevice_set_volume(a->adevout,a->evout,100*volume);
#endif   
#ifdef SGI_AUDIO
   a->params[3] = a->params[5] = (int)(volume*255.0);
   ALsetparams(AL_DEFAULT_DEVICE,a->params,6);
#endif
#ifdef SUN16_AUDIO
   a->audio_info.play.gain = (uint_t) (AUDIO_MIN_GAIN +
                                       (AUDIO_MAX_GAIN-AUDIO_MIN_GAIN)*volume);
   ioctl(a->audio_ctld, AUDIO_SETINFO, &a->audio_info);
#endif
#ifdef HPUX_AUDIO
   a->parms.play_volume = (AGainDB) (-100*(1.0-volume));
#endif
#ifdef OSS_AUDIO
   {
      int f, v;
      
      v = (int) (100.0*volume); f = (v & 0xff) | ((v & 0xff) << 8);
      if (ioctl(mixer_fd, MIXER_WRITE(SOUND_MIXER_PCM), &f)==-1)
         HError(6006, "SetVol: unable to set OSS mixer volume");
   }
#endif
}



/* -------------------- End of Device Dependent Code ----------------- */

/* InitAudio: initialise this module */
void InitAudio(void)
{
   int i;
   Boolean b;

   Register(haudio_version,haudio_vc_id);
   numParm = GetConfig("HAUDIO", TRUE, cParm, MAXGLOBS);
   if (numParm>0){
      if (GetConfInt(cParm,numParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,numParm,"LINEOUT",&b)) lineOut = b;
      if (GetConfBool(cParm,numParm,"PHONESOUT",&b)) phonesOut = b;
      if (GetConfBool(cParm,numParm,"SPEAKEROUT",&b)) speakerOut = b;
      if (GetConfBool(cParm,numParm,"LINEIN",&b)) lineIn = b;
      if (GetConfBool(cParm,numParm,"MICIN",&b)) micIn = b;
      if (GetConfInt(cParm,numParm,"VOLUMETYPE",&i)) volType = (VolType) i;
   }
#ifdef MMAPI_AUDIO
   sMagic=(65535 - getpid()<<12) ^ (time(NULL));
#endif
}

/* --------------------------- Status Handling ------------------- */

static char * aiStatMap[] = { "AI_CLEARED","AI_WAITSIG",
                              "AI_SAMPLING","AI_STOPPED",
                              "AI_ERROR" };

/* ChangeState: change state of audio input and trace if enabled */
static void ChangeState(AudioIn a, AudioInStatus newState)
{
   if (a->status != newState) {
      if (trace&T_STC)
         printf("HAudio:  %s -> %s\n",aiStatMap[a->status],aiStatMap[newState]);
      a->status = newState;
   }
}

/* ---------------------- Replay Buffer Routines --------------------- */

/* EXPORT->AttachReplayBuf: create a replay buffer in a of given size */
void AttachReplayBuf(AudioIn a, int bufSize)
{
   if (trace&T_TOP)
      printf("HAudio: attaching replay buffer\n");
   if (a==NULL) HError(6015,"AttachReplayBuf: null audio device");
   a->rbuf.isActive = TRUE;
   a->rbuf.inx = a->rbuf.outx = a->rbuf.used = 0;
   a->rbuf.size = bufSize;
   a->rbuf.data = (short *)New(a->mem,bufSize*sizeof(short));
}

/* ResetReplayBuf: reset replay buf to empty */
static void ResetReplayBuf(AudioIn a)
{
   a->rbuf.inx = a->rbuf.outx = a->rbuf.used = 0;
}

/* SaveInReplay: save a sample in replay buf, overwrite if nec. */
static void SaveInReplay(AudioIn a, short x)
{
   if (a->rbuf.isActive==FALSE) return;
   a->rbuf.data[a->rbuf.inx] = x;
   a->rbuf.inx = (a->rbuf.inx + 1) % a->rbuf.size;
   if (a->rbuf.used==a->rbuf.size)
      a->rbuf.outx = a->rbuf.inx;
   else  
      ++a->rbuf.used;
}

/* EXPORT->GetReplayBuf: Get upto nSamples from replay buffer */
int GetReplayBuf(AudioIn a, int nSamples, short *buf)
{
   int i,n;
   
   if (trace&T_TOP)
      printf("HAudio: getting replay buffer\n");
   if (a==NULL) HError(6015,"GetReplayBuf: null audio device");
   if (!a->rbuf.isActive)
      HError(6020,"GetReplayBuf: replay buf not active");
   n = (a->rbuf.used<nSamples)?a->rbuf.used:nSamples;
   for (i=0; i<n; i++){
      buf[i] = a->rbuf.data[a->rbuf.outx];
      a->rbuf.outx = (a->rbuf.outx + 1) % a->rbuf.size;
      --a->rbuf.used;
   }
   return n;
}

/* EXPORT-> PlayReplayBuffer: Output nSamples from ai's replay buffer to ao */
void PlayReplayBuffer(AudioOut ao, AudioIn ai)
{
   if (trace&T_TOP)
      printf("HAudio: playing replay buffer\n");
   if (ai==NULL) HError(6015,"PlayReplayBuffer: null audio in device");
   if (ao==NULL) HError(6015,"PlayReplayBuffer: null audio out device");
   if (!ai->rbuf.isActive)
      HError(6020,"PlayReplayBuffer: replay buf not active");
   if (ao->vol >= 0) SetVol(ao,ao->vol);
   if (ai->rbuf.used == 0) return;
   if (ai->rbuf.inx <= ai->rbuf.outx){
      PlayAudio(ao,ai->rbuf.data+ai->rbuf.outx,ai->rbuf.size - ai->rbuf.outx);
      if (ai->rbuf.inx>0) 
         PlayAudio(ao,ai->rbuf.data,ai->rbuf.inx);
   } else
      PlayAudio(ao,ai->rbuf.data+ai->rbuf.outx,ai->rbuf.inx - ai->rbuf.outx);
}

/* ----------------- Low level input buffering ------------------ */

static AudioIn sigAudio;  /* Globals used for signalling */
static int sigNum=NULLSIG;
static volatile Boolean waitForSigH;
static volatile Boolean alreadyFilling;
static int nz=0;

/* ProtectedFillBufferFromAudio:  Read samples from audio, try to 
   block until min samples are available from buffer.  
   Semaphores guarantee that this is not exectuted whilst signal 
   handler is already in this function */
static void ProtectedFillBufferFromAudio(AudioIn a,int min)
{
   int n,m,avail,wanted,freeSpace;

   if (a->status!=AI_SAMPLING) return;

   avail = InSamples(a); 
   wanted = min - a->nInBuffer;
   n = (avail<wanted)?wanted:avail;
   if (n <= 0) return;
 
   freeSpace = a->bufferSize - a->nInBuffer;
   if (n > freeSpace) n = freeSpace;

   if (a->inBufPos+n>a->bufferSize) {
      /* Need to roll over end of buffer */
      m = a->bufferSize - a->inBufPos;
      ReadAudio(a,a->buffer+a->inBufPos,m);
      n -= m;
      a->inBufPos=0;
   }
   ReadAudio(a,a->buffer+a->inBufPos,n);
   a->inBufPos  += n;
   a->nInBuffer += n;
}

/* StopAudioNow: stop the audio device immediately */
static void StopAndFlushAudio(AudioIn a, Boolean deferred)
{
   if (trace&T_TOP)
      printf("HAudio: stopping audio input %s\n",deferred?"deferred":"");
   ProtectedFillBufferFromAudio(a,0);
   StopAudi(a);
   ProtectedFillBufferFromAudio(a,0);
   ChangeState(a,AI_STOPPED);
   CloseAudi(a);
}

/* FillBufferFromAudio: called by main process as often as possible */
static void FillBufferFromAudio(AudioIn a,int min)
{
   if (a->sig<0 && KeyPressed(0))
      StopAndFlushAudio(a,FALSE);
   alreadyFilling=TRUE;
   while(waitForSigH);
   ProtectedFillBufferFromAudio(a,min);
   if(stopSignalled)  {
      StopAndFlushAudio(a,TRUE);
      stopSignalled = FALSE;
   }
   alreadyFilling=FALSE;
}

/* SignalFillBufferAndStopAudio: called by stop signal */
static void SignalFillBufferAndStopAudio(AudioIn a)
{
   waitForSigH=TRUE;
   if (alreadyFilling) {
      if (trace&T_TOP)
         printf("HAudio: stopping audio input later\n");
      stopSignalled = TRUE;
   }
   else
      StopAndFlushAudio(a,FALSE);
   waitForSigH=FALSE;
}

/* GetSampleFromBuffer: primary access point to audio buffer */
static short GetSampleFromBuffer(AudioIn a)
{
   short x;

#ifdef UNIX
   while (a->status==AI_WAITSIG)
      pause();
#endif
   if (a->nInBuffer==0)
      FillBufferFromAudio(a,1);
   if (a->nInBuffer==0) {
      x=0;
      nz++;
   }
   else {
      while(waitForSigH);
      a->nInBuffer--;
      x=a->buffer[a->outBufPos];
      a->outBufPos=(a->outBufPos+1)%a->bufferSize;
   }
   if (a->nInBuffer==0 && a->status==AI_STOPPED)
      ChangeState(a,AI_CLEARED);
   
   return(x);
}

/* ---------------------- Audio Input Routines -------------------- */

/* EXPORT->OpenAudioInput:  Initialise and return an audio stream */
AudioIn OpenAudioInput(MemHeap *x, HTime *sampPeriod, HTime winDur, HTime frPeriod)
{
   AudioIn a;
   int olap;
   HTime t;

#ifdef NO_AUDIO
   return NULL;
#endif
   if (trace&T_TOP)
      printf("HAudio: opening audio input - sampP=%.0f wDur=%.0f frP=%.0f\n",
             *sampPeriod,winDur,frPeriod);
   if (x->type != MSTAK) 
      HError(6016,"OpenAudioInput: memory must be an MSTAK");
   a = (AudioIn)New(x,sizeof(AudioInRec));
   a->mem = x; a->isActive=ADS_INIT;
   if (*sampPeriod <= 0.0  && GetConfFlt(cParm,numParm,"SOURCERATE",&t))
      *sampPeriod = t;

   InitAudi(a,sampPeriod);
   a->sampPeriod = *sampPeriod;
   if (frPeriod > 0.0){   /* frame mode */
      if (frPeriod > winDur)
         HError(6070,"OpenAudioInput: frame period > window duration");
      a->frSize = (int) (winDur / *sampPeriod);
      if (a->frSize==0)
         HError(6070,"OpenAudioInput: frame period requested too small");
      a->frRate = (int) (frPeriod / *sampPeriod);
      if (a->frRate==0)
         HError(6070,"OpenAudioInput: frame rate requested too small");
      a->frBuf  = (short *)New(x,a->frSize*sizeof(short));
      a->inOLap = 0;
      olap = a->frSize - a->frRate;
      a->frOLap = (olap>0)?CreateVector(x,olap):NULL;
   } else {
      a->frSize = a->frRate = 0;
      a->frBuf = NULL; a->frOLap = NULL;
   }
   a->status = AI_CLEARED;
   a->sig = NULLSIG;
   a->bufferSize=AUDBUFSIZE;
   a->nInBuffer=0;
   a->inBufPos=a->outBufPos=0;
   a->rbuf.isActive = FALSE;

   return a;
}

/* StartAudioSignal: start signal received */
static void StartAudioSignal(void)
{
   if (trace&T_TOP)
      printf("HAudio: starting audio input\n");
   StartAudi(sigAudio);
   ChangeState(sigAudio,AI_SAMPLING);
}

/* StopAudioSignal: called via audio signal handler */
static void StopAudioSignal(void)
{
   SignalFillBufferAndStopAudio(sigAudio);
}

/* AudioSigHandler: used to call Start/StopAudio via signals */
static void AudioSigHandler(void)
{
   if (sigAudio->status == AI_WAITSIG){
      StartAudioSignal();
      signal(sigNum,(void (*) (int))AudioSigHandler);
   }else if (sigAudio->status == AI_SAMPLING){
      StopAudioSignal();
      signal(sigNum,SIG_DFL);
   }
}

/* EXPORT->StartAudioInput: start sampling on given stream */
void StartAudioInput(AudioIn a, int sig)
{
   char c;
   
   if (a==NULL) HError(6015,"StartAudioInput: null audio device");
   a->sig = sig;
   ResetReplayBuf(a);
   sigAudio = a; sigNum = (sig<0?NULLSIG:sig);
   if (sig<0) { /* Means use keys to start/stop */
#ifndef WIN32
      printf("Press return to start sampling\n");
      read(0, &c, 1);
#endif
      StartAudioSignal();
   }
   else if (sig != NULLSIG) {
      ChangeState(a,AI_WAITSIG);
      signal(sig,(void (*) (int))AudioSigHandler);
      waitForSigH = FALSE; alreadyFilling = FALSE;
      stopSignalled = FALSE;
   }
   else
      StartAudioSignal();
}

/* EXPORT->StopAudioInput: stop sampling on given stream */
void StopAudioInput(AudioIn a)
{
   if (a==NULL) HError(6015,"StopAudioInput: null audio device");
   if (a->status != AI_STOPPED  && a->status != AI_CLEARED){
      SignalFillBufferAndStopAudio(a);
   }
}

/* EXPORT-> CloseAudioInput: terminate and free memory */
void CloseAudioInput(AudioIn a)
{
   if (trace&T_TOP) {
      printf("HAudio: closing audio input [%d zeros read]\n",nz);
      fflush(stdout);
   }
   if (a==NULL) HError(6015,"CloseAudioInput: null audio device");
   CloseAudi(a); /* Just in case it was never started */
   if (sigNum!=NULLSIG) signal(sigNum,SIG_DFL);
   Dispose(a->mem,a);
}

/* EXPORT->SamplesInAudio: return num samples available from a */
int SamplesInAudio(AudioIn a)
{
   FillBufferFromAudio(a,0);
   switch(a->status){
   case AI_STOPPED:
   case AI_SAMPLING:
      return a->nInBuffer;
   }
   return 0;
}

/* EXPORT->FramesInAudio: return num frames available from a */
int FramesInAudio(AudioIn a)
{
   int nSamples, firstFrame;

   if (a==NULL) HError(6015,"FramesInAudio: null audio device");
   nSamples = SamplesInAudio(a);
   firstFrame = a->frSize - a->inOLap;
   if (firstFrame > nSamples){
      if (a->status == AI_STOPPED) {
         for (;nSamples>=0;nSamples--)
            GetSampleFromBuffer(a);
      }
      return 0;
   }
   return (nSamples - firstFrame) / a->frRate + 1;
}

/* EXPORT->SampsInAudioFrame: return number of samples per frame */
int SampsInAudioFrame(AudioIn a)
{
   if (a==NULL) HError(6015,"SampsInAudioFrame: null audio device");
   return a->frSize;
}

/* EXPORT->GetAIStatus: return current status of audio stream a */
AudioInStatus GetAIStatus(AudioIn a)
{
   /* Fill buffer at every opportunity */
   FillBufferFromAudio(a,0);
   return a->status;
}

/* EXPORT->GetRawAudio: Get nSamples from a and store in buf */
void GetRawAudio(AudioIn a, int nSamples, short *buf)
{
   int n;
   short x,*p;

   if (a==NULL) HError(6015,"GetRawAudio: null audio device");

   /* Fill buffer at every opportunity */
   n = nSamples;
   if (n>0) FillBufferFromAudio(a,nSamples);
   if (nSamples == 0 ) return;

   n=0;p=buf;
   while (n<nSamples) {
      x=GetSampleFromBuffer(a);
      *p++=x,n++,SaveInReplay(a,x);
   }
}

/* EXPORT->GetAudio: get nFrames from a and store as floats in buf */
void GetAudio(AudioIn a, int nFrames, float *buf)
{
   int i,k,olap;
   float *thisBuf,*p;

   if (a==NULL) HError(6015,"GetAudio: null audio device");
   olap = a->frSize - a->frRate;
   for (thisBuf = buf,i=1; i<=nFrames; i++, thisBuf += a->frSize){
      if (a->inOLap == 0){
         GetRawAudio(a,a->frSize,a->frBuf);
         for (k=0; k<a->frSize; k++) thisBuf[k] = a->frBuf[k];
         if (olap>0){
            for (k=1,p=thisBuf+a->frRate; k<=olap; k++) 
               a->frOLap[k] = *p++;
            a->inOLap = olap;
         }
      } else {
         for (k=1,p=thisBuf; k<=olap; k++) 
            *p++ = a->frOLap[k];
         GetRawAudio(a,a->frRate,a->frBuf);
         for (k=0; k<a->frRate; k++) 
            *p++ = a->frBuf[k];
         for (k=1,p=thisBuf+a->frRate; k<=olap; k++) 
            a->frOLap[k] = *p++;
      }
   }
}

/* ---------------------- Audio Output Routines -------------------- */

/* EXPORT->OpenAudioOutput: return an audio output stream for given rate */
AudioOut OpenAudioOutput(MemHeap *x, HTime *sampPeriod)
{
   AudioOut a;

#ifdef NO_AUDIO
   return NULL;
#endif
   if (trace&T_TOP)
      printf("HAudio: opening audio output - sampP=%.0f\n",*sampPeriod);
   if (x->type != MSTAK) 
      HError(6016,"OpenAudioOutput: memory must be an MSTAK");
   a = (AudioOut)New(x,sizeof(AudioOutRec));
   a->mem = x;
   a->vol = -1;
   InitAudo(a,sampPeriod);
   return a;
}

/* EXPORT->StartAudioOutput: output nSamples to a using data stored in buf */
void StartAudioOutput(AudioOut a, long nSamples, short *buf)
{
   if (trace&T_TOP)
      printf("HAudio: starting audio output\n");
   if (a==NULL) HError(6015,"StartAudioOutput: null audio device");
   if (a->vol >= 0) SetVol(a,a->vol);
   PlayAudio(a,buf,nSamples);
}

/* EXPORT->CloseAudioOutput: Terminate audio stream a */
void CloseAudioOutput(AudioOut a)
{
   if (trace&T_TOP)
      printf("HAudio: closing audio output\n");
   if (a==NULL) HError(6015,"CloseAudioOutput: null audio device");
   CloseAudo(a);
   Dispose(a->mem,a);
}

/* EXPORT->SetVolume: Terminate audio stream a */
void SetVolume(AudioOut a, int volume)
{
   if (a==NULL) HError(6015,"SetVolume: null audio device");
   if (volume<1) volume = 0;
   if (volume>100) volume = 100;
   a->vol = (float) volume / 100.0;
   /* Most machines already log scaled
      a->vol = (volume < 10) ? (float)volume/1000.0 : 
      exp (log(10.0)*((float)volume/50.0 - 2.0));
   */
}

/* EXPORT->SamplesToPlay: return num samples left to play */
int SamplesToPlay(AudioOut a)
{
   if (a==NULL) HError(6015,"SamplesToPlay: null audio device");
   return OutSamples(a);
}

/* ------------------------ End of HAudio.c ------------------------- */
