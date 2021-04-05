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
/*          2001-2004 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*      File: HVite.c: recognise or align file or audio        */
/* ----------------------------------------------------------- */

char *hvite_version = "!HVER!HVite:   3.4.1 [CUED 12/03/09]";
char *hvite_vc_id = "$Id: HVite.c,v 1.1.1.1 2006/10/11 09:55:02 jal58 Exp $";

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
#include "HUtil.h"
#include "HTrain.h"
#include "HAdapt.h"
#include "HMap.h"
#include "HFB.h"
#include "HDict.h"
#include "HNet.h"
#include "HRec.h"

/* -------------------------- Trace Flags & Vars ------------------------ */

#define T_TOP 00001      /* Basic progress reporting */
#define T_OBS 00002      /* list observations */
#define T_FRS 00004      /* Frame by frame best token */
#define T_MEM 00010      /* Memory usage, start and finish */
#define T_MMU 00020      /* Memory usage after each utterance */

static int trace = 0;

/* -------------------------- Global Variables etc ---------------------- */

/* Doing what */
static int nToks = 0;             /* Number of tokens for N best */
static int nTrans = 1;            /* Number of transcriptions for N best */
static Boolean states = FALSE;    /* Keep track of state alignment */
static Boolean models = FALSE;    /* Keep track of model alignment */

/* With what */
static char *datFN;               /* Speech file */
static char *dictFn;              /* Dictionary */
static char *wdNetFn = NULL;      /* Word level lattice */
static char *hmmListFn;           /* HMMs */
static char * hmmDir = NULL;      /* directory to look for hmm def files */
static char * hmmExt = NULL;      /* hmm def file extension */
static Boolean loadLabels = FALSE; /* Load network for each file */
static Boolean loadNetworks = FALSE; /* Load network for each file */
static LabId bndId = NULL;        /* Boundary word for alignment */

/* Results and formats */
static char * labDir = NULL;      /* output label file directory */
static char * labExt = "rec";     /* output label file extension */
static char * labForm = NULL;     /* output label reformat */
static char * latForm = NULL;     /* output lattice format */
static char * labInDir = NULL;    /* input network/label file directory */
static char * labInExt = "lab";   /* input network/label file extension */
static char * latExt = NULL;      /* output lattice file extension */
static char * labFileMask = NULL; /* mask for reading lablels (lattices) */
static FileFormat dfmt=UNDEFF;    /* Data input file format */
static FileFormat ifmt=UNDEFF;    /* Label input file format */
static FileFormat ofmt=UNDEFF;    /* Label output file format */
static Boolean saveAudioOut=FALSE;/* Save rec output from direct audio */
static char * roPrefix=NULL;      /* Prefix for direct audio output name */
static char * roSuffix=NULL;      /* Suffix for direct audio output name */
static int roCounter = 0;         /* Counter for audio output name */
static Boolean replay = FALSE;    /* enable audio replay */

/* Language model */
static double lmScale = 1.0;      /* bigram and log(1/NSucc) scale factor */
static LogDouble wordPen = 0.0;   /* inter model propagation log prob */
static double prScale = 1.0;      /* pronunciation scale factor */

/* Pruning */
static LogDouble genBeam = -LZERO;/* genBeam threshold */
static LogDouble genBeamInc  = 0.0;       /* increment         */
static LogDouble genBeamLim = -LZERO;     /* max value       */
static LogDouble nBeam = 0.0;     /* nBeam threshold */
static LogDouble wordBeam = -LZERO;/* word-end pruning threshold */
static LogFloat tmBeam = 10.0;    /* tied mix prune threshold */
static int maxActive = 0;         /* max active phone instances */

/* Global variables */
static Observation obs;           /* current observation */
static HMMSet hset;               /* the HMM set */
static Vocab vocab;               /* the dictionary */
static Lattice *wdNet;            /* the word level recognition network */
static PSetInfo *psi;             /* Private data used by HRec */
static VRecInfo *vri;             /* Visible HRec Info */
static int maxM = 0;              /* max mixtures in any model */
static int maxMixInS[SMAX];       /* array[1..swidth[0]] of max mixes */

/* Global adaptation variables */
static int update = 0;            /* Perfom MLLR & update every n utts */
static UttInfo *utt;              /* utterance info for state/frame align */
static FBInfo *fbInfo;            /* forward-backward info for alignment */
static PSetInfo *alignpsi;        /* Private data used by HRec */
static VRecInfo *alignvri;        /* Visible HRec Info */
static Boolean saveBinary=FALSE;  /* Save tmf in binary format */

/* Heaps */
static MemHeap ansHeap;
static MemHeap modelHeap;
static MemHeap netHeap;
static MemHeap bufHeap;
static MemHeap repHeap;
static MemHeap regHeap;

/* information about transforms */
static XFInfo xfInfo;

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;
   Boolean b;
   char buf[MAXSTRLEN];

   nParm = GetConfig("HVITE", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfStr(cParm,nParm,"RECOUTPREFIX",buf))
         roPrefix=CopyString(&gstack,buf);
      if (GetConfStr(cParm,nParm,"RECOUTSUFFIX",buf))
         roSuffix=CopyString(&gstack,buf);
      if (GetConfBool(cParm,nParm,"SAVEBINARY",&b)) 
         saveBinary = b;
      if (GetConfStr(cParm,nParm,"LABFILEMASK",buf)) {
         labFileMask = CopyString(&gstack, buf);
      }
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HVite [options] VocabFile HMMList DataFiles...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a      align from label files               off\n");
   printf(" -b s    def s as utterance boundary word     none\n");
   printf(" -c f    tied mixture pruning threshold       10.0\n");
   printf(" -d s    dir to find hmm definitions          current\n");
   printf(" -e      save direct audio rec output         off\n");
   printf(" -f      output full state alignment          off\n");
   printf(" -g      enable audio replay                  off\n");
   printf(" -h s    set speaker name pattern             *.mfc\n");
   printf(" -i s    Output transcriptions to MLF s       off\n"); 
   printf(" -j i    Online MLLR adaptation               off\n");
   printf("         Perform update every i utterances      \n");
   printf(" -k      use an input transform               off\n");
   printf(" -l s    dir to store label/lattice files     current\n");
   printf(" -m      output model alignment               off\n");
   printf(" -n i [N] N-best recognition (using i tokens) off\n");
   printf(" -o s    output label formating NCSTWMX       none\n");
   printf(" -p f    inter model trans penalty (log)      0.0\n");
   printf(" -q s    output lattice formating ABtvaldmn   tvaldmn\n");
   printf(" -r f    pronunciation prob scale factor      1.0\n");
   printf(" -s f    grammar scale factor                 1.0\n");
   printf(" -t f [f f] set pruning threshold             0.0\n");
   printf(" -u i    set pruning max active               0\n");
   printf(" -v f    set word end pruning threshold       0.0\n"); 
   printf(" -w [s]  recognise from network               off\n");
   printf(" -x s    extension for hmm files              none\n");
   printf(" -y s    output label file extension          rec\n");
   printf(" -z s    generate lattices with extension s   off\n");
   PrintStdOpts("BEFGHIJKLPSX");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *s;

   void Initialise(void);
   void DoRecognition(void);
   void DoAlignment(void);

   if(InitShell(argc,argv,hvite_version,hvite_vc_id)<SUCCESS)
      HError(3200,"HVite: InitShell failed");

   InitMem();   InitLabel();
   InitMath();  InitSigP();
   InitWave();  InitAudio();
   InitVQ();    InitModel();

   if(InitParm()<SUCCESS)  
      HError(3200,"HVite: InitParm failed");

   InitDict();
   InitNet();   InitRec();
   InitUtil(); 
   InitAdapt(&xfInfo); InitMap();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);

   SetConfParms();
   CreateHeap(&modelHeap, "Model heap",  MSTAK, 1, 0.0, 100000, 800000 );
   CreateHMMSet(&hset,&modelHeap,TRUE); 

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(3219,"HVite: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'a':
         loadLabels=TRUE; break;
      case 'b':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Utterance boundary word expected");
         bndId = GetLabId(GetStrArg(),TRUE); break;
      case 'c':
         tmBeam = GetChkedFlt(0.0,1000.0,s); break;          
      case 'd':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: HMM definition directory expected");
         hmmDir = GetStrArg(); break;
      case 'e':
         saveAudioOut=TRUE; break;
      case 'f':
         states=TRUE; break;
      case 'g':
         replay=TRUE; break;
      case 'i':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Output MLF file name expected");
         /* if(SaveToMasterfile(GetStrArg())<SUCCESS)
            HError(3214,"HCopy: Cannot write to MLF"); */
         SaveToMasterfile(GetStrArg());
         break;
      case 'k':
	 xfInfo.useInXForm = TRUE;
	 break;
      case 'j':
         if (NextArg()!=INTARG)
            HError(3219,"HVite: No. of files per online adaptation step expected");
         update = GetChkedInt(1,256,s);
         break;
      case 'l':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Label file directory expected");
         labDir = GetStrArg(); break;
      case 'm':
         models=TRUE; break;
      case 'n':
         nToks = GetChkedInt(2,MAX_TOKS,s);
         if (NextArg()==FLOATARG || NextArg()==INTARG)
            nTrans = GetChkedInt(1,10000,s);
         else
            nTrans = 1;
         break;      
      case 'o':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Output label format expected");
         labForm = GetStrArg(); break;
      case 'p':
         wordPen = GetChkedFlt(-1000.0,1000.0,s);  break;
      case 'q':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Output lattice format expected");
         latForm = GetStrArg(); break;
      case 'r':
         prScale = GetChkedFlt(0.0,1000.0,s);  break;
      case 's':
         lmScale = GetChkedFlt(0.0,1000.0,s);  break;
      case 't':
         genBeam = GetChkedFlt(0,1.0E20,s); 
	 if (genBeam == 0.0)
	    genBeam = -LZERO;
         if (NextArg()==FLOATARG || NextArg()==INTARG) {
             genBeamInc = GetChkedFlt(0.0,1.0E20,s);
             genBeamLim = GetChkedFlt(0.0,1.0E20,s);
             if (genBeamLim < (genBeam + genBeamInc)) {
                genBeamLim = genBeam; genBeamInc = 0.0;
             }
          }
          else {
             genBeamInc = 0.0;
             genBeamLim = genBeam;
          }  
          break;
      case 'w':
         if (NextArg()!=STRINGARG)
            loadNetworks=TRUE;
         else {
            wdNetFn = GetStrArg();
            if (strlen(wdNetFn)==0) {
               wdNetFn=NULL;
               loadNetworks=TRUE;
            }
         }
         break;
      case 'u':
         maxActive = GetChkedInt(0,100000,s); break;      
      case 'v':
         wordBeam = GetChkedFlt(0,1.0E20,s); 
         if (wordBeam == 0.0)
            wordBeam = -LZERO;
         break;
      case 'x':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: HMM file extension expected");
         hmmExt = GetStrArg(); break;
      case 'y':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Output label file extension expected");
         labExt = GetStrArg(); break;
      case 'z':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Lattice output file extension expected");
         latExt = GetStrArg(); break;
      case 'F':
         if (NextArg() != STRINGARG)
            HError(3219,"HVite: Data File format expected");
         if((dfmt = Str2Format(GetStrArg())) == ALIEN)
            HError(-3289,"HVite: Warning ALIEN Input file format set");
         break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(3219,"HVite: Source Label File format expected");
         if((ifmt = Str2Format(GetStrArg())) == ALIEN)
            HError(-3289,"HVite: Warning ALIEN Input file format set");
         break;
      case 'H':
         if (NextArg() != STRINGARG)
            HError(3219,"HVite: MMF File name expected");
         AddMMF(&hset,GetStrArg()); 
         break;
      case 'I':
         if (NextArg() != STRINGARG)
            HError(3219,"HVite: MLF file name expected");
         LoadMasterFile(GetStrArg()); break;
      case 'L':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Label/network file directory expected");
         labInDir = GetStrArg(); break;
      case 'P':
         if (NextArg() != STRINGARG)
            HError(3219,"HVite: Target Label File format expected");
         if((ofmt = Str2Format(GetStrArg())) == ALIEN)
            HError(-3289,"HVite: Warning ALIEN Label output file format set");
         break;
      case 'B':
         saveBinary = TRUE;
         break;
      case 'T':
         trace = GetChkedInt(0,511,s); break;
      case 'X':
         if (NextArg()!=STRINGARG)
            HError(3219,"HVite: Input label/network file extension expected");
         labInExt = GetStrArg(); break;
      case 'h':
	if (NextArg()!=STRINGARG)
	  HError(1,"Speaker name pattern expected");
	xfInfo.outSpkrPat = GetStrArg();
	if (NextArg()==STRINGARG) {
	  xfInfo.inSpkrPat = GetStrArg();
	  if (NextArg()==STRINGARG)
	    xfInfo.paSpkrPat = GetStrArg(); 
	}
	if (NextArg() != SWITCHARG)
	  HError(2319,"HERest: cannot have -h as the last option");	  
	break;
      case 'E':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: parent transform directory expected");
	 xfInfo.usePaXForm = TRUE;
         xfInfo.paXFormDir = GetStrArg(); 
         if (NextArg()==STRINGARG)
	   xfInfo.paXFormExt = GetStrArg(); 
	 if (NextArg() != SWITCHARG)
	   HError(2319,"HVite: cannot have -E as the last option");	  
         break;              
      case 'J':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: input transform directory expected");
         AddInXFormDir(&hset,GetStrArg());
         if (NextArg()==STRINGARG)
	   xfInfo.inXFormExt = GetStrArg(); 
	 if (NextArg() != SWITCHARG)
	   HError(2319,"HVite: cannot have -J as the last option");	  
         break;              
      case 'K':
         if (NextArg()!=STRINGARG)
            HError(2319,"HVite: output transform directory expected");
         xfInfo.outXFormDir = GetStrArg(); 
	 xfInfo.useOutXForm = TRUE;
         if (NextArg()==STRINGARG)
	   xfInfo.outXFormExt = GetStrArg(); 
	 if (NextArg() != SWITCHARG)
	   HError(2319,"HVite: cannot have -K as the last option");	  
         break;              
      default:
         HError(3219,"HVite: Unknown switch %s",s);
      }
   }
   
   if (NextArg()!=STRINGARG)
      HError(3219,"HVite: Dictionary file name expected");
   dictFn = GetStrArg();
   if (NextArg()!=STRINGARG)
      HError(3219,"HVite: HMM list  file name expected");
   hmmListFn = GetStrArg();

#ifndef PHNALG
   if ((states || models) && nToks>1)
      HError(3230,"HVite: Alignment using multiple tokens is not supported");
#endif
   if (NumArgs()==0 && wdNetFn==NULL)
      HError(3230,"HVite: Network must be specified for recognition from audio");
   if (loadNetworks && loadLabels)
      HError(3230,"HVite: Must choose either alignment from network or labels");
   if (nToks>1 && latExt==NULL && nTrans==1)
      HError(-3230,"HVite: Performing nbest recognition with no nbest output");
   if (nToks > 1 && latExt != NULL && nTrans > 1) 
      HError(-3230,"HVite: Performing nbest recognition with 1-best and latttices output");
   if ((update>0) && (!xfInfo.useOutXForm))
      HError(3230,"HVite: Must use -K option with incremental adaptation");


   Initialise();


   /* Process the data */
   if (wdNetFn==NULL)
      DoAlignment();
   else
      DoRecognition();

   /* Free up and we are done */

   if (trace & T_MEM) {
      printf("Memory State on Completion\n");
      PrintAllHeapStats();
   }


   DeleteVRecInfo(vri);
   ResetHeap(&netHeap);
   FreePSetInfo(psi);
   UpdateSpkrStats(&hset,&xfInfo, NULL); 
   ResetHeap(&regHeap);
   ResetHeap(&modelHeap);
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* --------------------------- Initialisation ----------------------- */

/* Initialise: set up global data structures */
void Initialise(void)
{
   Boolean eSep;
   int s;

   /* Load hmms, convert to inverse DiagC */
   if(MakeHMMSet(&hset,hmmListFn)<SUCCESS) 
      HError(3228,"Initialise: MakeHMMSet failed");
   if(LoadHMMSet(&hset,hmmDir,hmmExt)<SUCCESS) 
      HError(3228,"Initialise: LoadHMMSet failed");
   ConvDiagC(&hset,TRUE);
   
   /* Create observation and storage for input buffer */
   SetStreamWidths(hset.pkind,hset.vecSize,hset.swidth,&eSep);
   obs=MakeObservation(&gstack,hset.swidth,hset.pkind,
                       hset.hsKind==DISCRETEHS,eSep);

   /* sort out masks just in case using adaptation */
   if (xfInfo.inSpkrPat == NULL) xfInfo.inSpkrPat = xfInfo.outSpkrPat; 
   if (xfInfo.paSpkrPat == NULL) xfInfo.paSpkrPat = xfInfo.outSpkrPat; 

   if (xfInfo.useOutXForm || (update>0)) {
      CreateHeap(&regHeap,   "regClassStore",  MSTAK, 1, 0.5, 1000, 8000 );
      /* This initialises things - temporary hack - THINK!! */
      CreateAdaptXForm(&hset, "tmp");
      /* initialise structures for the f-b frame-state alignment pass */
      utt = (UttInfo *) New(&regHeap, sizeof(UttInfo));
      fbInfo = (FBInfo *) New(&regHeap, sizeof(FBInfo));
      /* initialise a recogniser for frame/state alignment purposes */
      alignpsi=InitPSetInfo(&hset);
      alignvri=InitVRecInfo(alignpsi,1,TRUE,FALSE);
      SetPruningLevels(alignvri,0,genBeam,-LZERO,0.0,tmBeam);
      InitUttInfo(utt, FALSE);
      InitialiseForBack(fbInfo, &regHeap, &hset,
                        (UPDSet) (UPXFORM), genBeam*2.0, genBeam*2.0, 
                        genBeam*4.0+1.0, 10.0);
      utt->twoDataFiles = FALSE;
      utt->S = hset.swidth[0]; 
      AttachPreComps(&hset,hset.hmem);
   }
    
   CreateHeap(&bufHeap,"Input Buffer heap",MSTAK,1,0.0,50000,50000);
   CreateHeap(&repHeap,"Replay Buffer heap",MSTAK,1,0.0,50000,50000);
   
   maxM = MaxMixInSet(&hset);
   for (s=1; s<=hset.swidth[0]; s++)
      maxMixInS[s] = MaxMixInSetS(&hset, s);
   if (trace&T_TOP) {
      printf("Read %d physical / %d logical HMMs\n",
             hset.numPhyHMM,hset.numLogHMM);  fflush(stdout);
   }
   
   /* Initialise recogniser */
   if (nToks>1) nBeam=genBeam;
   psi=InitPSetInfo(&hset);
   vri=InitVRecInfo(psi,nToks,models,states);

   /* Read dictionary and create storage for lattice */
   InitVocab(&vocab);   
   if(ReadDict(dictFn,&vocab)<SUCCESS) 
      HError(3213, "Main: ReadDict failed");
   CreateHeap(&ansHeap,"Lattice heap",MSTAK,1,0.0,4000,4000);
   if (trace & T_MEM){
      printf("Memory State After Initialisation\n");
      PrintAllHeapStats();
   }
}

/* ------------------ Utterance Level Recognition  ----------------------- */

/* ReplayAudio:  replay the last audio input */
void ReplayAudio(BufferInfo info)
{
   AudioOut ao;

   if (info.a != NULL) {
      ao = OpenAudioOutput(&repHeap,&(info.srcSampRate));
      PlayReplayBuffer(ao, info.a);
      while (SamplesToPlay(ao) > 0 );
      CloseAudioOutput(ao);
   }
}

/* DoOnlineAdaptation: Perform unsupervised online adaptation
   using the recognition hypothesis as the transcription */
int DoOnlineAdaptation(Lattice *lat, ParmBuf pbuf, int nFrames)
{
   Transcription *modelTrans, *trans;
   BufferInfo pbinfo;
   Lattice *alignLat, *wordNet;
   Network *alignNet;
   int i;

   GetBufferInfo(pbuf,&pbinfo);
   trans=TranscriptionFromLattice(&netHeap,lat,1);
   wordNet=LatticeFromLabels(GetLabelList(trans,1),bndId,
                             &vocab,&netHeap);
   alignNet=ExpandWordNet(&netHeap,wordNet,&vocab,&hset);

   StartRecognition(alignvri,alignNet,0.0,0.0,0.0);     

   /* do forced alignment */
   for (i = 0; i < nFrames; i++) {
      ReadAsTable(pbuf, i, &obs);
      ProcessObservation(alignvri,&obs,-1,xfInfo.inXForm);
   }
    
   alignLat=CompleteRecognition(alignvri,
                                pbinfo.tgtSampRate/10000000.0,
                                &netHeap);
        
   if (alignvri->noTokenSurvived) {
      Dispose(&netHeap, trans);
      /* Return value 0 to indicate zero frames process failed */
      return 0;
   }
   modelTrans=TranscriptionFromLattice(&netHeap,alignLat,1);
      
   /* format the transcription so that it contains just the models */
   FormatTranscription(modelTrans,pbinfo.tgtSampRate,FALSE,TRUE,
                       FALSE,FALSE,TRUE,FALSE,TRUE,TRUE, FALSE);

   /* Now do the frame/state alignment accumulating MLLR statistics */
   /* set the various values in the utterance storage */
   utt->tr = modelTrans;
   utt->pbuf = pbuf;
   utt->Q = CountLabs(utt->tr->head);
   utt->T = nFrames;
   utt->ot = obs;
  
   /* do frame state alignment and accumulate statistics */
   fbInfo->inXForm = xfInfo.inXForm;
   fbInfo->al_inXForm = xfInfo.inXForm;
   fbInfo->paXForm = xfInfo.paXForm;
   if (!FBFile(fbInfo, utt, NULL))
     nFrames = 0;

   Dispose(&netHeap, trans);

   if (trace&T_TOP) {
      printf("Accumulated statistics...\n"); 
      fflush(stdout);
   }
   return nFrames;
} 

/* ProcessFile: process given file. If fn=NULL then direct audio */
Boolean ProcessFile(char *fn, Network *net, int utterNum, LogDouble currGenBeam, Boolean restartable)
{
   FILE *file;
   ParmBuf pbuf;
   BufferInfo pbinfo;
   NetNode *d;
   Lattice *lat;
   LArc *arc,*cur;
   LNode *node;
   Transcription *trans;
   MLink m;
   LogFloat lmlk,aclk;
   int s,j,tact,nFrames;
   LatFormat form;
   char *p,lfn[255],buf1[80],buf2[80],thisFN[MAXSTRLEN];
   Boolean enableOutput = TRUE, isPipe;

   if (fn!=NULL)
      strcpy(thisFN,fn);
   else if (fn==NULL && saveAudioOut)
      CounterFN(roPrefix,roSuffix,++roCounter,4,thisFN);
   else 
      enableOutput = FALSE;
      
   if((pbuf = OpenBuffer(&bufHeap,fn,50,dfmt,TRI_UNDEF,TRI_UNDEF))==NULL)
      HError(3250,"ProcessFile: Config parameters invalid");   

   /* Check pbuf same as hset */
   GetBufferInfo(pbuf,&pbinfo);
   if (pbinfo.tgtPK!=hset.pkind)
      HError(3231,"ProcessFile: Incompatible sample kind %s vs %s",
             ParmKind2Str(pbinfo.tgtPK,buf1),
             ParmKind2Str(hset.pkind,buf2));
   if (pbinfo.a != NULL && replay)  AttachReplayBuf(pbinfo.a, (int) (3*(1.0E+07/pbinfo.srcSampRate)));

   StartRecognition(vri,net,lmScale,wordPen,prScale);
   SetPruningLevels(vri,maxActive,currGenBeam,wordBeam,nBeam,tmBeam);
 
   tact=0;nFrames=0;
   StartBuffer(pbuf);
   while(BufferStatus(pbuf)!=PB_CLEARED) {
      ReadAsBuffer(pbuf,&obs);
      if (trace&T_OBS) PrintObservation(nFrames,&obs,13);      

      if (hset.hsKind==DISCRETEHS){
         for (s=1; s<=hset.swidth[0]; s++){
            if( (obs.vq[s] < 1) || (obs.vq[s] > maxMixInS[s]))
               HError(3250,"ProcessFile: Discrete data value [ %d ] out of range in stream [ %d ] in file %s",obs.vq[s],s,fn);
         }
      }

      ProcessObservation(vri,&obs,-1,xfInfo.inXForm);
      
      if (trace & T_FRS) {
         for (d=vri->genMaxNode,j=0;j<30;d=d->links[0].node,j++)
            if (d->type==n_word) break;
         if (d->type==n_word){
            if (d->info.pron==NULL) p=":bound:";
            else p=d->info.pron->word->wordName->name;
         }
         else p=":external:";
         m=FindMacroStruct(&hset,'h',vri->genMaxNode->info.hmm);
         printf("Optimum @%-4d HMM: %s (%s)  %d %5.3f\n",
                vri->frame,m->id->name,p,
                vri->nact,vri->genMaxTok.like/vri->frame);
         fflush(stdout);
      }
      nFrames++;
      tact+=vri->nact;
   }
   lat=CompleteRecognition(vri,pbinfo.tgtSampRate/10000000.0,&ansHeap);
   
   if (lat==NULL) {
      if ((trace & T_TOP) && fn != NULL){
         if (restartable)
            printf("No tokens survived to final node of network at beam %.1f\n", currGenBeam);
         else
            printf("No tokens survived to final node of network\n");
         fflush(stdout);
      } else if (fn==NULL){
         printf("Sorry [%d frames]?\n",nFrames);fflush(stdout);
      }      
      if (pbinfo.a != NULL && replay)  ReplayAudio(pbinfo);
      CloseBuffer(pbuf);
      return FALSE;
   }
   
   if (vri->noTokenSurvived && restartable)
      return FALSE;

   if (vri->noTokenSurvived && trace & T_TOP) {
      printf("No tokens survived to final node of network\n");
      printf("  Output most likely partial hypothesis within network\n");
      fflush(stdout);
   }

   lat->utterance=thisFN;
   lat->net=wdNetFn;
   lat->vocab=dictFn;
   
   if (trace & T_TOP || fn==NULL) {
      node=NULL;
      for (j=0;j<lat->nn;j++) {
         node=lat->lnodes+j;
         if (node->pred==NULL) break;
         node=NULL;
      }
      aclk=lmlk=0.0;
      while(node!=NULL) {
         for (arc=NULL,cur=node->foll;cur!=NULL;cur=cur->farc) arc=cur;
         if (arc==NULL) break;
         if (arc->end->word!=NULL)
            printf("%s ",arc->end->word->wordName->name);
         aclk+=arc->aclike+arc->prlike*lat->prscale;
         lmlk+=arc->lmlike*lat->lmscale+lat->wdpenalty;
         node=arc->end;
      }
      printf(" ==  [%d frames] %.4f [Ac=%.1f LM=%.1f] (Act=%.1f)\n",nFrames,
             (aclk+lmlk)/nFrames, aclk,lmlk,(float)tact/nFrames);
      fflush(stdout);
   }
   if (pbinfo.a != NULL && replay)  ReplayAudio(pbinfo);
   
   /* accumulate stats for online unsupervised adaptation 
      only if a token survived */
   if ((lat != NULL) &&  (!vri->noTokenSurvived) && ((update > 0) || (xfInfo.useOutXForm)))
      DoOnlineAdaptation(lat, pbuf, nFrames);

   if (enableOutput){
      if (nToks>1 && latExt!=NULL) {
         MakeFN(thisFN,labDir,latExt,lfn);
         if ((file=FOpen(lfn,NetOFilter,&isPipe))==NULL) 
            HError(3211,"ProcessFile: Could not open file %s for lattice output",lfn);
         if (latForm==NULL)
            form=HLAT_DEFAULT;
         else {
            for (p=latForm,form=0;*p!=0;p++) {
               switch (*p) {
               case 'A': form|=HLAT_ALABS; break;
               case 'B': form|=HLAT_LBIN; break;
               case 't': form|=HLAT_TIMES; break;
               case 'v': form|=HLAT_PRON; break;
               case 'a': form|=HLAT_ACLIKE; break;
               case 'l': form|=HLAT_LMLIKE; break;
               case 'd': form|=HLAT_ALIGN; break;
               case 'm': form|=HLAT_ALDUR; break;
               case 'n': form|=HLAT_ALLIKE; break;
               case 'r': form|=HLAT_PRLIKE; break;
               }
            }
         }
         if(WriteLattice(lat,file,form)<SUCCESS)
            HError(3214,"ProcessFile: WriteLattice failed");

         FClose(file,isPipe);
      }

      /* only output 1-best transcription if generating lattices */
      if (nTrans > 1 && latExt != NULL) 
         trans=TranscriptionFromLattice(&ansHeap,lat,1);
      /* output N-best transcriptions as usual */
      else
      trans=TranscriptionFromLattice(&ansHeap,lat,nTrans);
      
      if (labForm!=NULL)
         FormatTranscription(trans,pbinfo.tgtSampRate,states,models,
                             strchr(labForm,'X')!=NULL,
                             strchr(labForm,'N')!=NULL,strchr(labForm,'S')!=NULL,
                             strchr(labForm,'C')!=NULL,strchr(labForm,'T')!=NULL,
                             strchr(labForm,'W')!=NULL,strchr(labForm,'M')!=NULL);

      MakeFN(thisFN,labDir,labExt,lfn);
      /* if(LSave(lfn,trans,ofmt)<SUCCESS)
         HError(3214,"ProcessFile: Cannot save file %s", lfn); */
      LSave(lfn,trans,ofmt);
      Dispose(&ansHeap,trans);
   }
   Dispose(&ansHeap,lat);
   CloseBuffer(pbuf);
   if (trace & T_MMU){
      printf("Memory State after utter %d\n",utterNum);
      PrintAllHeapStats();
   }

   return !vri->noTokenSurvived;
}

/* --------------------- Top Level Processing --------------------- */

/* DoAlignment: by creating network from transcriptions or lattices */
void DoAlignment(void)
{
   FILE *nf;
   char lfn[MAXSTRLEN], buf[MAXSTRLEN];
   Transcription *trans;
   Network *net;
   Boolean isPipe;
   int n=0;
   LogDouble currGenBeam;
   AdaptXForm *incXForm;

   if (trace&T_TOP) {
      if (loadNetworks) 
         printf("New network will be used for each file\n");
      else
         printf("Label file will be used to align each file\n");
      fflush(stdout);
   }
   CreateHeap(&netHeap,"Net heap",MSTAK,1,0,8000,80000);
   while (NumArgs()>0) {
      if (NextArg() != STRINGARG)
         HError(3219,"DoAlignment: Data file name expected");
      datFN = GetStrArg();
      if (trace&T_TOP) {
         printf("Aligning File: %s\n",datFN);  fflush(stdout);
      }
      if (labFileMask != NULL ) { /* support for rescoring lattice masks */
         if (!MaskMatch(labFileMask,buf,datFN))
            HError(2319,"DoAlignment: mask %s has no match with segemnt %s",labFileMask,datFN);
         MakeFN(buf,labInDir,labInExt,lfn);
      } else {
         MakeFN(datFN,labInDir,labInExt,lfn);
      }
      if (loadNetworks) {
         if ( (nf = FOpen(lfn,NetFilter,&isPipe)) == NULL)
            HError(3210,"DoAlignment: Cannot open Word Net file %s",lfn);
         if((wdNet = ReadLattice(nf,&netHeap,&vocab,TRUE,FALSE))==NULL)
            HError(3210,"DoAlignment: ReadLattice failed");
         FClose(nf,isPipe);
         if (trace&T_TOP) {
            printf("Read lattice with %d nodes / %d arcs\n",
                   wdNet->nn,wdNet->na);
            fflush(stdout);
         }
      }
      else {
         LabList *ll = NULL;

         trans=LOpen(&netHeap,lfn,ifmt);
         if (trans->numLists >= 1)
            ll = GetLabelList(trans,1);
         if (!ll && !bndId)
            HError(3233, "DoAlignment: cannot align empty transcription");

         wdNet=LatticeFromLabels(ll, bndId, &vocab,&netHeap);
         if (trace&T_TOP) {
            printf("Created lattice with %d nodes / %d arcs from label file\n",
                   wdNet->nn,wdNet->na);
            fflush(stdout);
         }
      }
      net=ExpandWordNet(&netHeap,wdNet,&vocab,&hset);

      ++n;
      currGenBeam = genBeam;
      /* This handles the initial input transform, parent transform setting
	 and output transform creation */
      if (UpdateSpkrStats(&hset, &xfInfo, datFN) && (!(xfInfo.useInXForm)) && (hset.semiTied == NULL)) {
         xfInfo.inXForm = NULL;
      }
      if (genBeamInc == 0.0)
         ProcessFile (datFN, net, n, currGenBeam, FALSE);
      else {
         Boolean completed;

         completed = ProcessFile (datFN, net, n, currGenBeam, TRUE);
         currGenBeam += genBeamInc;
         while (!completed && (currGenBeam <= genBeamLim - genBeamInc)) {
            completed = ProcessFile (datFN, net, n, currGenBeam, TRUE);
            currGenBeam += genBeamInc;
         }
         if (!completed)
            ProcessFile (datFN, net, n, currGenBeam, FALSE);
      }

      if (update > 0 && n%update == 0) {
         if (trace&T_TOP) {
            printf("Transforming model set\n");
            fflush(stdout);
         }
	 /* 
	    at every stage a new transform is created - fix?? 
	    Estimate transform and then set it up as the 
	    input XForm
	 */
	 incXForm = CreateAdaptXForm(&hset,"inc");
         TidyBaseAccs();
	 GenAdaptXForm(&hset,incXForm);
         xfInfo.inXForm = GetMLLRDiagCov(incXForm);;
	 SetXForm(&hset,xfInfo.inXForm);
	 ApplyHMMSetXForm(&hset,xfInfo.inXForm);
      }
      ResetHeap(&netHeap);
   }
}

/* DoRecognition:  use single network to recognise each input utterance */
void DoRecognition(void)
{
   FILE *nf;
   Network *net;
   Boolean isPipe;
   int n=0;
   AdaptXForm *incXForm;

   if ( (nf = FOpen(wdNetFn,NetFilter,&isPipe)) == NULL)
      HError(3210,"DoRecognition: Cannot open Word Net file %s",wdNetFn);
   if((wdNet = ReadLattice(nf,&ansHeap,&vocab,TRUE,FALSE))==NULL)
      HError(3210,"DoAlignment: ReadLattice failed");
   FClose(nf,isPipe);

   if (trace&T_TOP) {
      printf("Read lattice with %d nodes / %d arcs\n",wdNet->nn,wdNet->na);
      fflush(stdout);
   }
   CreateHeap(&netHeap,"Net heap",MSTAK,1,0,
              wdNet->na*sizeof(NetLink),wdNet->na*sizeof(NetLink));

   net = ExpandWordNet(&netHeap,wdNet,&vocab,&hset);
   ResetHeap(&ansHeap);
   if (trace&T_TOP) {
      printf("Created network with %d nodes / %d links\n",
             net->numNode,net->numLink);  fflush(stdout);
   }
   if (trace & T_MEM){
      printf("Memory State Before Recognition\n");
      PrintAllHeapStats();
   }

   if (NumArgs()==0) {      /* Process audio */
      while(TRUE){
         printf("\nREADY[%d]>\n",++n); fflush(stdout);
	 /* no input transform possible for audio input .... */
         ProcessFile(NULL,net,n,genBeam, FALSE);
         if (update > 0 && n%update == 0) {
            if (trace&T_TOP) {
               printf("Transforming model set\n");
               fflush(stdout);
            }
	    /* 
	       at every stage a new transform is created - fix?? 
	       Estimate transform and then set it up as the 
	       input XForm
	    */
	    incXForm = CreateAdaptXForm(&hset,"inc");
            TidyBaseAccs();
	    GenAdaptXForm(&hset,incXForm);
            xfInfo.inXForm = GetMLLRDiagCov(incXForm);;
            SetXForm(&hset,xfInfo.inXForm);
	    ApplyHMMSetXForm(&hset,xfInfo.inXForm);
         }
      }
   }
   else {                   /* Process files */
      while (NumArgs()>0) {
         if (NextArg()!=STRINGARG)
            HError(3219,"DoRecognition: Data file name expected");
         datFN = GetStrArg();
         if (trace&T_TOP) {
            printf("File: %s\n",datFN); fflush(stdout);
         }
	 /* This handles the initial input transform, parent transform setting
	    and output transform creation */
         if (UpdateSpkrStats(&hset, &xfInfo, datFN) && (!(xfInfo.useInXForm)) && (hset.semiTied == NULL)) {
            xfInfo.inXForm = NULL;
         }
         ProcessFile(datFN,net,n++,genBeam,FALSE);
         if (update > 0 && n%update == 0) {
            if (trace&T_TOP) {
               printf("Transforming model set\n");
               fflush(stdout);
            }
	    /* 
	       at every stage a new transform is created - fix?? 
	       Estimate transform and then set it up as the 
	       input XForm
	    */
	    incXForm = CreateAdaptXForm(&hset,"inc");
            TidyBaseAccs();
	    GenAdaptXForm(&hset,incXForm);
            xfInfo.inXForm = GetMLLRDiagCov(incXForm);;
            SetXForm(&hset,xfInfo.inXForm);
	    ApplyHMMSetXForm(&hset,xfInfo.inXForm);
         }
      }
   }
}

/* ----------------------------------------------------------- */
/*                      END:  HVite.c                          */
/* ----------------------------------------------------------- */
