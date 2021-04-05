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
/*          2002-2004 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HERest.c: Embedded B-W ReEstimation           */
/* ----------------------------------------------------------- */

char *herest_version = "!HVER!HERest:   3.4.1 [CUED 12/03/09]";
char *herest_vc_id = "$Id: HERest.c,v 1.2 2006/12/07 11:09:08 mjfg Exp $";

/*
   This program is used to perform a single reestimation of
   the parameters of a set of HMMs using Baum-Welch.  Training
   data consists of one or more utterances each of which has a 
   transcription in the form of a standard label file (segment
   boundaries are ignored).  For each training utterance, a
   composite model is effectively synthesised by concatenating
   the phoneme models given by the transcription.  Each phone
   model has the usual set of accumulators allocated to it,
   these are updated by performing a standard B-W pass over
   each training utterance using the composite model. This program
   supports arbitrary parameter tying and multiple data streams.
   
   Added in V1.4 - support for tee-Models ie HMMs with a non-
   zero transition from entry to exit states.

   In v2.2 most of the core functionality has been moved to the
   library module HFB
*/

#include "HShell.h"     /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HAudio.h"
#include "HWave.h"
#include "HVQ.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HTrain.h"
#include "HUtil.h"
#include "HAdapt.h"
#include "HMap.h"
#include "HFB.h"

/* Trace Flags */
#define T_TOP   0001    /* Top level tracing */
#define T_MAP   0002    /* logical/physical hmm map */
#define T_UPD   0004    /* Model updates */


/* possible values of updateMode */
#define UPMODE_DUMP 1
#define UPMODE_UPDATE 2
#define UPMODE_BOTH 3

/* Global Settings */

static char * labDir = NULL;     /* label (transcription) file directory */
static char * labExt = "lab";    /* label file extension */
static char * hmmDir = NULL;     /* directory to look for hmm def files */
static char * hmmExt = NULL;     /* hmm def file extension */
static char * newDir = NULL;     /* directory to store new hmm def files */
static char * newExt = NULL;     /* extension of new reestimated hmm files */
static char * statFN;            /* stats file, if any */
static float minVar  = 0.0;      /* minimum variance (diagonal only) */
static float mixWeightFloor=0.0; /* Floor for mixture weights */
static int minEgs    = 3;        /* min examples to train a model */
static UPDSet uFlags = (UPDSet) (UPMEANS|UPVARS|UPTRANS|UPMIXES); /* update flags */
static int parMode   = -1;       /* enable one of the // modes */
static Boolean stats = FALSE;    /* enable statistics reports */
static char * mmfFn  = NULL;     /* output MMF file, if any */
static int trace     = 0;        /* Trace level */
static Boolean saveBinary = FALSE;  /* save output in binary  */
static Boolean ldBinary = TRUE;        /* load/dump in binary */
static FileFormat dff=UNDEFF;       /* data file format */
static FileFormat lff=UNDEFF;       /* label file format */
static int updateMode = UPMODE_UPDATE; /* dump summed accs, update models or do both? */


static ConfParam *cParm[MAXGLOBS];   /* configuration parameters */
static int nParm = 0;               /* total num params */

static Boolean al_hmmUsed = FALSE;   /* Set for 2-model ReEstimation */
static char al_hmmDir[MAXFNAMELEN];  /* dir to look for alignment hmm defs */
static char al_hmmExt[MAXSTRLEN];  	 /* alignment hmm def file extension */
static char al_hmmMMF[MAXFNAMELEN];  /* alignment hmm MMF */
static char al_hmmLst[MAXFNAMELEN];  /* alignment hmm list */
static char up_hmmMMF[MAXFNAMELEN];  /* alignment hmm list */
static HMMSet al_hset ;      	 /* Option 2nd set of models for alignment */

/* Global Data Structures - valid for all training utterances */
static LogDouble pruneInit = NOPRUNE;    /* pruning threshold initially */
static LogDouble pruneInc = 0.0;         /* pruning threshold increment */
static LogDouble pruneLim = NOPRUNE;     /* pruning threshold limit */
static float minFrwdP = NOPRUNE;         /* mix prune threshold */


static Boolean firstTime = TRUE;    /* Flag used to enable creation of ot */
static Boolean twoDataFiles = FALSE; /* Enables creation of ot2 for FB
                                        training using two data files */
static int totalT=0;       /* total number of frames in training data */
static LogDouble totalPr=0;   /* total log prob upto current utterance */
static Vector vFloor[SMAX]; /* variance floor - default is all zero */

static MemHeap hmmStack;   /*For Storage of all dynamic structures created...*/
static MemHeap uttStack;
static MemHeap fbInfoStack;
static MemHeap accStack;

/* information about transforms */
static XFInfo xfInfo;
static int maxSpUtt = 0;
static float varFloorPercent = 0;

static char *labFileMask = NULL;

/* ------------------ Process Command Line -------------------------- */
   
/* SetConfParms: set conf parms relevant to HCompV  */
void SetConfParms(void)
{
   int i;
   Boolean b;
   double f;
   char buf[MAXSTRLEN];
   
   nParm = GetConfig("HEREST", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfFlt(cParm,nParm,"VARFLOORPERCENTILE",&f)) varFloorPercent = f;
      if (GetConfBool(cParm,nParm,"SAVEBINARY",&b)) saveBinary = b;
      if (GetConfBool(cParm,nParm,"BINARYACCFORMAT",&b)) ldBinary = b;
      /* 2-model reestimation alignment model set */
      if (GetConfStr(cParm,nParm,"ALIGNMODELMMF",buf)) {
          strcpy(al_hmmMMF,buf); al_hmmUsed = TRUE;
      }
      if (GetConfStr(cParm,nParm,"ALIGNHMMLIST",buf)) {
          strcpy(al_hmmLst,buf); al_hmmUsed = TRUE;
      }
      /* allow multiple individual model files */
      if (GetConfStr(cParm,nParm,"ALIGNMODELDIR",buf)) {
          strcpy(al_hmmDir,buf); al_hmmUsed = TRUE;
      }
      if (GetConfStr(cParm,nParm,"ALIGNMODELEXT",buf)) {
          strcpy(al_hmmExt,buf); al_hmmUsed = TRUE;
      }
      if (GetConfStr(cParm,nParm,"ALIGNXFORMEXT",buf)) {
         xfInfo.alXFormExt = CopyString(&hmmStack,buf);
      }
      if (GetConfStr(cParm,nParm,"ALIGNXFORMDIR",buf)) {
         xfInfo.alXFormDir = CopyString(&hmmStack,buf);
      }
      if (GetConfStr(cParm,nParm,"INXFORMMASK",buf)) {
         xfInfo.inSpkrPat = CopyString(&hmmStack,buf);
      }
      if (GetConfStr(cParm,nParm,"PAXFORMMASK",buf)) {
         xfInfo.paSpkrPat = CopyString(&hmmStack,buf);
      }
      if (GetConfStr(cParm,nParm,"LABFILEMASK",buf)) {
         labFileMask = (char*)malloc(strlen(buf)+1); 
         strcpy(labFileMask, buf);
      }

      if (GetConfStr(cParm,nParm,"UPDATEMODE",buf)) {
         if (!strcmp (buf, "DUMP")) updateMode = UPMODE_DUMP;
         else if (!strcmp (buf, "UPDATE")) updateMode = UPMODE_UPDATE;
         else if (!strcmp (buf, "BOTH")) updateMode = UPMODE_BOTH;
         else HError(2319, "Unknown UPDATEMODE specified (must be DUMP, UPDATE or BOTH)");
      }
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HERest [options] hmmList dataFiles...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a      Use an input linear transform        off\n");
   printf(" -c f    Mixture pruning threshold            10.0\n");
   printf(" -d s    dir to find hmm definitions          current\n");
   printf(" -h s    set output speaker name pattern   *.%%%%%%\n");
   printf("         to s, optionally set input and parent patterns\n");
   printf(" -l N    set max files per speaker            off\n");
   printf(" -m N    set min examples needed per model    3\n");
   printf(" -o s    extension for new hmm files          as src\n");
   printf(" -p N    set parallel mode to N               off\n");
   printf(" -r      Enable Single Pass Training...       \n");
   printf("         ...using two parameterisations       off\n");
   printf(" -s s    print statistics to file s           off\n");
   printf(" -t f [i l] set pruning to f [inc limit]      inf\n");
   printf(" -u tmvwaps  update t)rans m)eans v)ars w)ghts tmvw\n");
   printf("                a)daptation xform p)rior used     \n");
   printf("                s)semi-tied xform                 \n");
   printf(" -v f    set minimum variance to f            0.0\n");
   printf(" -w f    set mix weight floor to f*MINMIX     0.0\n");
   printf(" -x s    extension for hmm files              none\n");
   printf(" -z s    Save all xforms to TMF file s        TMF\n");
   PrintStdOpts("BEFGHIJKLMSTX");
   printf("\n\n");
}

void SetuFlags(void)
{
   char *s;
   
   s=GetStrArg();
   uFlags=(UPDSet) 0;        
   while (*s != '\0')
      switch (*s++) {
      case 't': uFlags = (UPDSet) (uFlags+UPTRANS); break;
      case 'm': uFlags = (UPDSet) (uFlags+UPMEANS); break;
      case 'v': uFlags = (UPDSet) (uFlags+UPVARS); break;
      case 'w': uFlags = (UPDSet) (uFlags+UPMIXES); break;
      case 's': uFlags = (UPDSet) (uFlags+UPSEMIT); break;
      case 'a': uFlags = (UPDSet) (uFlags+UPXFORM); break;
      case 'p': uFlags = (UPDSet) (uFlags+UPMAP); break;
      default: HError(2320,"SetuFlags: Unknown update flag %c",*s);
         break;
      }
}

/* ScriptWord: return next word from script */
char *ScriptWord(FILE *script, char *scriptBuf)
{
   int ch,qch,i;
   
   i=0; ch=' ';
   while (isspace(ch)) ch = fgetc(script);
   if (ch==EOF) {
      scriptBuf=NULL;
      return NULL;
   }
   if (ch=='\'' || ch=='"'){
      qch = ch;
      ch = fgetc(script);
      while (ch != qch && ch != EOF) {
         scriptBuf[i++] = ch; 
         ch = fgetc(script);
      }
      if (ch==EOF)
         HError(5051,"ScriptWord: Closing quote missing in script file");
   } else {
      do {
         scriptBuf[i++] = ch; 
         ch = fgetc(script);
      }while (!isspace(ch) && ch != EOF);
   }
   scriptBuf[i] = '\0';

   return scriptBuf;
}

void CheckUpdateSetUp()
{
  AdaptXForm *xf;

  xf = xfInfo.paXForm;
  if ((xfInfo.paXForm != NULL) && !(uFlags&UPXFORM)) {
    while (xf != NULL) {
       if ((xf->xformSet->xkind != CMLLR) && (xf->xformSet->xkind != SEMIT))
	HError(999,"SAT only supported with SEMIT/CMLLR transforms");
      xf = xf->parentXForm;
    }
  }
}

int main(int argc, char *argv[])
{
   char *datafn=NULL;
   char *datafn2=NULL;
   char *s;
   char *scriptFile;
   char datafn1[MAXSTRLEN];
   char newFn[MAXSTRLEN];
   FILE *f;
   UttInfo *utt;            /* utterance information storage */
   FBInfo *fbInfo;          /* forward-backward information storage */
   HMMSet hset;             /* Set of HMMs to be re-estimated */
   Source src;
   float tmpFlt;
   int tmpInt;
   int numUtt,spUtt=0;

   void Initialise(FBInfo *fbInfo, MemHeap *x, HMMSet *hset, char *hmmListFn);
   void DoForwardBackward(FBInfo *fbInfo, UttInfo *utt, char *datafn, char *datafn2);
   void UpdateModels(HMMSet *hset, ParmBuf pbuf2);
   void StatReport(HMMSet *hset);
   
   if(InitShell(argc,argv,herest_version,herest_vc_id)<SUCCESS)
      HError(2300,"HERest: InitShell failed");
   InitMem();    InitMath();
   InitSigP();   InitAudio();
   InitWave();   InitVQ();
   InitLabel();  InitModel();
   if(InitParm()<SUCCESS)  
      HError(2300,"HERest: InitParm failed");
   InitTrain();
   InitUtil();   InitFB();
   InitAdapt(&xfInfo); InitMap();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   al_hmmDir[0] = '\0'; al_hmmExt[0] = '\0'; 
   al_hmmMMF[0] = '\0'; al_hmmLst[0] = '\0'; 
   up_hmmMMF[0] = '\0';
   CreateHeap(&hmmStack,"HmmStore", MSTAK, 1, 1.0, 50000, 500000);
   SetConfParms(); 
   CreateHMMSet(&hset,&hmmStack,TRUE);
   CreateHeap(&uttStack,   "uttStore",    MSTAK, 1, 0.5, 100,   1000);
   utt = (UttInfo *) New(&uttStack, sizeof(UttInfo));
   CreateHeap(&fbInfoStack,   "FBInfoStore",  MSTAK, 1, 0.5, 100 ,  1000 );
   fbInfo = (FBInfo *) New(&fbInfoStack, sizeof(FBInfo));
   CreateHeap(&accStack,   "accStore",    MSTAK, 1, 1.0, 50000,   500000);

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(2319,"HERest: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'b':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: script file expected");
         scriptFile = GetStrArg(); break;
      case 'c':
         minFrwdP = GetChkedFlt(0.0,1000.0,s);
         break;
      case 'd':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: HMM definition directory expected");
         hmmDir = GetStrArg(); break;   
      case 'm':
         minEgs = GetChkedInt(0,1000,s); break;
      case 'o':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: HMM file extension expected");
         newExt = GetStrArg(); break;
      case 'p':
         parMode = GetChkedInt(0,500,s); break;
      case 'r':
         twoDataFiles = TRUE; break;
      case 's':
         stats = TRUE;
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: Stats file name expected");
         statFN = GetStrArg(); break;
      case 't':
         pruneInit =  GetChkedFlt(0.0,1.0E20,s);
         if (NextArg()==FLOATARG || NextArg()==INTARG)
            {
               pruneInc = GetChkedFlt(0.0,1.0E20,s);
               pruneLim = GetChkedFlt(0.0,1.0E20,s);
            }
         else
            {
               pruneInc = 0.0;
               pruneLim = pruneInit  ;
            }
         break;
      case 'u':
         SetuFlags(); break;
      case 'v':
         minVar = GetChkedFlt(0.0,10.0,s); break;
      case 'w':
         mixWeightFloor = MINMIX * GetChkedFlt(0.0,10000.0,s); 
         break;
      case 'x':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: HMM file extension expected");
         hmmExt = GetStrArg(); break;
      case 'B':
         saveBinary=TRUE;
         break;
      case 'F':
         if (NextArg() != STRINGARG)
            HError(2319,"HERest: Data File format expected");
         if((dff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2389,"HERest: Warning ALIEN Data file format set");
         break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(2319,"HERest: Label File format expected");
         if((lff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2389,"HERest: Warning ALIEN Label file format set");
         break;
      case 'H':
         if (NextArg() != STRINGARG)
            HError(2319,"HERest: HMM macro file name expected");
         strcpy(up_hmmMMF,GetStrArg());
         AddMMF(&hset,up_hmmMMF);
         break;     
      case 'I':
         if (NextArg() != STRINGARG)
            HError(2319,"HERest: MLF file name expected");
         LoadMasterFile(GetStrArg());
         break;
      case 'L':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: Label file directory expected");
         labDir = GetStrArg(); break;
      case 'M':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: Output macro file directory expected");
         newDir = GetStrArg();
         break;     
      case 'T':
         trace = GetChkedInt(0,0100000,s);
         break;
      case 'X':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: Label file extension expected");
         labExt = GetStrArg(); break;
	 /* additional options for transform support */
      case 'a':
 	xfInfo.useInXForm = TRUE; break;
      case 'h':
	if (NextArg()!=STRINGARG)
	  HError(1,"Speaker name pattern expected");
	xfInfo.outSpkrPat = GetStrArg();
	break;
      case 'l':
         maxSpUtt = GetChkedInt(0,0100000,s);
         break;
      case 'E':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: parent transform directory expected");
	 xfInfo.usePaXForm = TRUE;
         xfInfo.paXFormDir = GetStrArg(); 
         if (NextArg()==STRINGARG)
	   xfInfo.paXFormExt = GetStrArg(); 
	 if (NextArg() != SWITCHARG)
	   HError(2319,"HERest: cannot have -E as the last option");	  
         break;              
      case 'J':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: input transform directory expected");
         AddInXFormDir(&hset,GetStrArg());
         if (NextArg()==STRINGARG) {
            if (xfInfo.inXFormExt == NULL)
               xfInfo.inXFormExt = GetStrArg(); 
            else
               HError(2319,"HERest: only one input transform extension may be specified");
         }
	 if (NextArg() != SWITCHARG)
	   HError(2319,"HERest: cannot have -J as the last option");	  
         break;              
      case 'K':
         if (NextArg()!=STRINGARG)
            HError(2319,"HERest: output transform directory expected");
         xfInfo.outXFormDir = GetStrArg(); 
         if (NextArg()==STRINGARG)
	   xfInfo.outXFormExt = GetStrArg(); 
	 if (NextArg() != SWITCHARG)
	   HError(2319,"HERest: cannot have -K as the last option");	  
         break;              
      case 'z':
         if (NextArg() != STRINGARG)
            HError(2319,"HERest: output TMF file expected");
         xfInfo.xformTMF = GetStrArg(); break;
      default:
         HError(2319,"HERest: Unknown switch %s",s);
      }
   } 
   if (NextArg() != STRINGARG)
      HError(2319,"HERest: file name of vocabulary list expected");

   Initialise(fbInfo, &fbInfoStack, &hset, GetStrArg());
   InitUttInfo(utt, twoDataFiles);
   numUtt = 1;

   if (trace&T_TOP) 
      SetTraceFB(); /* allows HFB to do top-level tracing */

   do {
      if (NextArg()!=STRINGARG)
         HError(2319,"HERest: data file name expected");
      if (twoDataFiles && (parMode!=0)){
         if ((NumArgs() % 2) != 0)
            HError(2319,"HERest: Must be even num of training files for single pass training");
         strcpy(datafn1,GetStrArg());
         datafn = datafn1;
         
         datafn2 = GetStrArg();
      }else
         datafn = GetStrArg();
      if (parMode==0){
         src=LoadAccs(&hset, datafn,uFlags);
         ReadFloat(&src,&tmpFlt,1,ldBinary);
         totalPr += (LogDouble)tmpFlt;
         ReadInt(&src,&tmpInt,1,ldBinary);
         totalT += tmpInt;
         CloseSource( &src );
      }
      else {
         /* track speakers */	 
         if (UpdateSpkrStats(&hset,&xfInfo, datafn)) spUtt=0;
	 /* Check to see whether set-up is valid */
	 CheckUpdateSetUp();
         fbInfo->inXForm = xfInfo.inXForm;
         fbInfo->al_inXForm = xfInfo.al_inXForm;
         fbInfo->paXForm = xfInfo.paXForm;
         if ((maxSpUtt==0) || (spUtt<maxSpUtt))
            DoForwardBackward(fbInfo, utt, datafn, datafn2) ;
         numUtt += 1; spUtt++;
      }
   } while (NumArgs()>0);

   if (uFlags&UPXFORM) {/* ensure final speaker correctly handled */ 
      UpdateSpkrStats(&hset,&xfInfo, NULL); 
      if (trace&T_TOP) {
         printf("Reestimation complete - average log prob per frame = %e (%d frames)\n",
                totalPr/totalT, totalT);
      }
   } else {
      if (parMode>0  || (parMode==0 && (updateMode&UPMODE_DUMP))){
         MakeFN("HER$.acc",newDir,NULL,newFn);
         f=DumpAccs(&hset,newFn,uFlags,parMode);
         tmpFlt = (float)totalPr;
         WriteFloat(f,&tmpFlt,1,ldBinary);
         WriteInt(f,(int*)&totalT,1,ldBinary);
         fclose( f );
      }
      if (parMode <= 0) {
         if (stats) {
            StatReport(&hset);
         }
         if (updateMode&UPMODE_UPDATE)
            UpdateModels(&hset,utt->pbuf2);
      }
   }
   ResetHeap(&uttStack);
   ResetHeap(&fbInfoStack);
   ResetHeap(&hmmStack);
   ResetHeap(&accStack);
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* -------------------------- Initialisation ----------------------- */

void Initialise(FBInfo *fbInfo, MemHeap *x, HMMSet *hset, char *hmmListFn)
{   
   HSetKind hsKind;
   int L,P,S,vSize,maxM; 

   /* Load HMMs and init HMMSet related global variables */
   if(MakeHMMSet( hset, hmmListFn )<SUCCESS)
      HError(2321,"Initialise: MakeHMMSet failed");
   if(LoadHMMSet( hset,hmmDir,hmmExt)<SUCCESS)
      HError(2321,"Initialise: LoadHMMSet failed");
   if (uFlags&UPSEMIT) uFlags = uFlags|UPMEANS|UPVARS;
   AttachAccs(hset, &accStack, uFlags);
   ZeroAccs(hset, uFlags);
   P = hset->numPhyHMM;
   L = hset->numLogHMM;
   vSize = hset->vecSize;
   S = hset->swidth[0];
   maxM = MaxMixInSet(hset);

   hsKind = hset->hsKind;
   if (hsKind==DISCRETEHS)
     uFlags = (UPDSet) (uFlags & (~(UPMEANS|UPVARS|UPXFORM|UPSEMIT)));

   if (parMode != 0) {
      ConvDiagC(hset,TRUE);
   }
   if (trace&T_TOP) {
      if (uFlags&UPMAP)  printf("HERest  MAP Updating: ");
      else printf("HERest  ML Updating: ");
      if (uFlags&UPTRANS) printf("Transitions "); 
      if (uFlags&UPMEANS) printf("Means "); 
      if (uFlags&UPVARS)  printf("Variances "); 
      if (uFlags&UPSEMIT)  printf("SemiTied "); 
      if (uFlags&UPXFORM)  printf("XForms "); 
      if (uFlags&UPMIXES && maxM>1)  printf("MixWeights "); 
      printf("\n\n ");
    
      if (parMode>=0) printf("Parallel-Mode[%d] ",parMode);

      printf("System is ");
      switch (hsKind){
      case PLAINHS:  printf("PLAIN\n");  break;
      case SHAREDHS: printf("SHARED\n"); break;
      case TIEDHS:   printf("TIED\n"); break;
      case DISCRETEHS: printf("DISCRETE\n"); break;
      }

      printf("%d Logical/%d Physical Models Loaded, VecSize=%d\n",L,P,vSize);
      if (hset->numFiles>0)
         printf("%d MMF input files\n",hset->numFiles);
      if (mmfFn != NULL)
         printf("Output to MMF file:  %s\n",mmfFn); 
      fflush(stdout);
   }
   SetVFloor( hset, vFloor, minVar);
   totalPr = 0.0;

   if (xfInfo.inSpkrPat == NULL) xfInfo.inSpkrPat = xfInfo.outSpkrPat; 
   if (xfInfo.paSpkrPat == NULL) xfInfo.paSpkrPat = xfInfo.outSpkrPat; 
   if (uFlags&UPXFORM) {
      if ((hsKind != PLAINHS) && (hsKind != SHAREDHS))
         HError(999,"Can only estimated transforms with PLAINHS and SHAREDHS!");
      if (uFlags != UPXFORM)
         HError(999,"Can only update linear transforms OR model parameters!");
      xfInfo.useOutXForm = TRUE;
      /* This initialises things - temporary hack - THINK!! */
      CreateAdaptXForm(hset, "tmp");
   } 

   
   /* initialise and  pass information to the forward backward library */
   InitialiseForBack(fbInfo, x, hset, uFlags, pruneInit, pruneInc,
                     pruneLim, minFrwdP);

   if (parMode != 0) {
      ConvLogWt(hset);
   }
   /* 2-model reestimation */
   if (al_hmmUsed){
       if (trace&T_TOP)
           printf("2-model re-estimation enabled\n");
       /* load alignment HMM set */
       CreateHMMSet(&al_hset,&hmmStack,TRUE);
       xfInfo.al_hset = &al_hset;
       if (xfInfo.alXFormExt == NULL) xfInfo.alXFormExt = xfInfo.inXFormExt;
       /* load multiple MMFs */
       if (strlen(al_hmmMMF) > 0 ) {
           char *p,*q;
           Boolean eos;
           p=q=al_hmmMMF;
           for(;;) {
               eos = (*p=='\0');
               if ( ( isspace((int) *p) || *p == '\0' ) && (q!=p) ) {
                   *p='\0';
                   if (trace&T_TOP) { 
                       printf("Loading alignment HMM set %s\n",q);
                   }
                   AddMMF(&al_hset,q);
                   if (eos)
                       break;
                   q=p+1;
               }
               p++;
           }
       }
       if (strlen(al_hmmLst) > 0 ) 
           MakeHMMSet(&al_hset, al_hmmLst );
       else /* use same hmmList */
           MakeHMMSet(&al_hset, hmmListFn );
       if (strlen(al_hmmDir) > 0 )
           LoadHMMSet(&al_hset,al_hmmDir,al_hmmExt);
       else
           LoadHMMSet(&al_hset,NULL,NULL);

       /* switch model set */
       UseAlignHMMSet(fbInfo,x,&al_hset);
       if (parMode != 0) {
	  ConvDiagC(&al_hset,TRUE);
	  ConvLogWt(&al_hset);
       }

       /* and echo status */
       if (trace&T_TOP) { 
           if (strlen(al_hmmDir) > 0 )
               printf(" HMM Dir %s",al_hmmDir);
           if (strlen(al_hmmExt) > 0 )
               printf(" Ext %s",al_hmmExt);
           printf("\n");
           if (strlen(al_hmmLst) > 0 )
               printf("HMM List %s\n",al_hmmLst);
           printf(" %d Logical/%d Physical Models Loaded, VecSize=%d\n",
                  al_hset.numLogHMM,al_hset.numPhyHMM,al_hset.vecSize);
       }
   }
}

/* ------------------- Statistics Reporting  -------------------- */

/* PrintStats: for given hmm */
void PrintStats(HMMSet *hset,FILE *f, int n, HLink hmm, int numEgs)
{
   WtAcc *wa;
   char buf[MAXSTRLEN];
   StateInfo *si;
   int i,N;
    
   N = hmm->numStates;
   ReWriteString(HMMPhysName(hset,hmm),buf,DBL_QUOTE);
   fprintf(f,"%4d %14s %4d ",n,buf,numEgs);
   for (i=2;i<N;i++) {
      si = hmm->svec[i].info;
      wa = (WtAcc *)((si->pdf+1)->hook);
      fprintf(f," %10f",wa->occ);
   }
   fprintf(f,"\n");
}

/* StatReport: print statistics report */
void StatReport(HMMSet *hset)
{
   HMMScanState hss;
   HLink hmm;
   FILE *f;
   int px;

   if ((f = fopen(statFN,"w")) == NULL){
      HError(2311,"StatReport: Unable to open stats file %s",statFN);
      return;
   }
   NewHMMScan(hset,&hss);
   px=1;
   do {
      hmm = hss.hmm;
      PrintStats(hset,f,px,hmm,(int)hmm->hook);
      px++;
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   fclose(f);
}

/* -------------------- Top Level of F-B Updating ---------------- */


/* Load data and call FBFile: apply forward-backward to given utterance */
void DoForwardBackward(FBInfo *fbInfo, UttInfo *utt, char * datafn, char * datafn2)
{
   char datafn_lab[MAXFNAMELEN];

   utt->twoDataFiles = twoDataFiles ;
   utt->S = fbInfo->al_hset->swidth[0];

   /* Load the labels - support for label masks */
   if (labFileMask) {
      if (!MaskMatch (labFileMask, datafn_lab, datafn))
         HError(2319,"HERest: LABFILEMASK %s has no match with segemnt %s", labFileMask, datafn);
   }
   else
      strcpy (datafn_lab, datafn);
   LoadLabs(utt, lff, datafn_lab, labDir, labExt);
   /* Load the data */
   LoadData(fbInfo->al_hset, utt, dff, datafn, datafn2);

   if (firstTime) {
      InitUttObservations(utt, fbInfo->al_hset, datafn, fbInfo->maxMixInS);
      firstTime = FALSE;
   }
  
   /* fill the alpha beta and otprobs (held in fbInfo) */
   if (FBFile(fbInfo, utt, datafn)) {
      /* update totals */
      totalT += utt->T ;
      totalPr += utt->pr ;
      /* Handle the input xform Jacobian if necssary */
      if (fbInfo->al_hset->xf != NULL) {
         totalPr += utt->T*0.5*fbInfo->al_hset->xf->xform->det;
      }

   }
}

/* --------------------------- Model Update --------------------- */

static int nFloorVar = 0;     /* # of floored variance comps */
static int nFloorVarMix = 0;  /* # of mix comps with floored vars */

/* UpdateTrans: use acc values to calc new estimate for transP */
void UpdateTrans(HMMSet *hset, int px, HLink hmm)
{
   int i,j,N;
   float x,occi;
   TrAcc *ta;
   
   ta = (TrAcc *) GetHook(hmm->transP);
   if (ta==NULL) return;   /* already done */
   N = hmm->numStates;
   for (i=1;i<N;i++) {
      occi = ta->occ[i];
      if (occi > 0.0) 
         for (j=2;j<=N;j++) {
            x = ta->tran[i][j]/occi;
            hmm->transP[i][j] = (x>MINLARG)?log(x):LZERO;
         }
      else
         HError(-2326,"UpdateTrans: Model %d[%s]: no transitions out of state %d",
                px,HMMPhysName(hset,hmm),i);
   }
   SetHook(hmm->transP,NULL);
}

/* FloorMixes: apply floor to given mix set */
void FloorMixes(HMMSet *hset, MixtureElem *mixes, int M, float floor)
{
   float sum,fsum,scale;
   MixtureElem *me;
   int m;
   
   if (hset->logWt == TRUE) HError(999,"FloorMixes requires linear weights");
   sum = fsum = 0.0;
   for (m=1,me=mixes; m<=M; m++,me++) {
      if (MixWeight(hset,me->weight)>floor)
         sum += me->weight;
      else {
         fsum += floor; me->weight = floor;
      }
   }
   if (fsum>1.0) HError(2327,"FloorMixes: Floor sum too large");
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2328,"FloorMixes: No mixture weights above floor");
   scale = (1.0-fsum)/sum;
   for (m=1,me=mixes; m<=M; m++,me++)
      if (me->weight>floor) me->weight *= scale;
}

/* FloorTMMixes: apply floor to given tied mix set */
void FloorTMMixes(Vector mixes, int M, float floor)
{
   float sum,fsum,scale,fltWt;
   int m;
   
   sum = fsum = 0.0;
   for (m=1; m<=M; m++) {
      fltWt = mixes[m];
      if (fltWt>floor)
         sum += fltWt;
      else {
         fsum += floor;
         mixes[m] = floor;
      }
   }
   if (fsum>1.0) HError(2327,"FloorTMMixes: Floor sum too large");
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2328,"FloorTMMixes: No mixture weights above floor");
   scale = (1.0-fsum)/sum;
   for (m=1; m<=M; m++){
      fltWt = mixes[m];
      if (fltWt>floor)
         mixes[m] = fltWt*scale;
   }
}

/* FloorDProbs: apply floor to given discrete prob set */
void FloorDProbs(ShortVec mixes, int M, float floor)
{
   float sum,fsum,scale,fltWt;
   int m;
   
   sum = fsum = 0.0;
   for (m=1; m<=M; m++) {
      fltWt = Short2DProb(mixes[m]);
      if (fltWt>floor)
         sum += fltWt;
      else {
         fsum += floor;
         mixes[m] = DProb2Short(floor);
      }
   }
   if (fsum>1.0) HError(2327,"FloorDProbs: Floor sum too large");
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2328,"FloorDProbs: No probabilities above floor");
   scale = (1.0-fsum)/sum;
   for (m=1; m<=M; m++){
      fltWt = Short2DProb(mixes[m]);
      if (fltWt>floor)
         mixes[m] = DProb2Short(fltWt*scale);
   }
}

/* UpdateWeights: use acc values to calc new estimate of mix weights */
void UpdateWeights(HMMSet *hset, int px, HLink hmm)
{
   int i,s,m,M=0,N,S;
   float x,occi;
   WtAcc *wa;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   HSetKind hsKind;

   N = hmm->numStates;
   se = hmm->svec+2;
   hsKind = hset->hsKind;
   S = hset->swidth[0];
   for (i=2; i<N; i++,se++){
      ste = se->info->pdf+1;
      for (s=1;s<=S; s++,ste++){
         wa = (WtAcc *)ste->hook;
         switch (hsKind){
         case TIEDHS:
            M=hset->tmRecs[s].nMix;
            break;
         case DISCRETEHS:
         case PLAINHS:
         case SHAREDHS:
            M=ste->nMix;
            break;
         }
         if (wa != NULL) {
            occi = wa->occ;
            if (occi>0) {
               for (m=1; m<=M; m++){
                  x = wa->c[m]/occi;
                  if (x>1.0){
                     if (x>1.001)
                        HError(2393,"UpdateWeights: Model %d[%s]: mix too big in %d.%d.%d %5.5f",
                               px,HMMPhysName(hset,hmm),i,s,m,x);
                     x = 1.0;
                  }
                  switch (hsKind){
                  case TIEDHS:
                     ste->spdf.tpdf[m] = (x>MINMIX) ? x : 0.0;
                     break;
                  case DISCRETEHS:
                     ste->spdf.dpdf[m]=(x>MINMIX) ? DProb2Short(x) : DLOGZERO;
                     break;
                  case PLAINHS:
                  case SHAREDHS:
                     me=ste->spdf.cpdf+m;
                     me->weight = (x>MINMIX) ? x : 0.0;
                     break;
                  }
               }
               if (mixWeightFloor>0.0){
                  switch (hsKind){
                  case DISCRETEHS:
                     FloorDProbs(ste->spdf.dpdf,M,mixWeightFloor);
                     break;
                  case TIEDHS:
                     FloorTMMixes(ste->spdf.tpdf,M,mixWeightFloor);
                     break;
                  case PLAINHS:
                  case SHAREDHS:
                     FloorMixes(hset,ste->spdf.cpdf+1,M,mixWeightFloor);
                     break;
                  }
               }
            }else
               HError(-2330,"UpdateWeights: Model %d[%s]: no use of mixtures in %d.%d",
                      px,HMMPhysName(hset,hmm),i,s);
            ste->hook = NULL;
         }
      }
   }
}
      
/* UpdateMeans: use acc values to calc new estimate of means */
void UpdateMeans(HMMSet *hset, int px, HLink hmm)
{
   int i,s,m,k,M,N,S,vSize;
   float occim;
   MuAcc *ma;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   Vector mean;
   
   N = hmm->numStates;
   se = hmm->svec+2;
   S = hset->swidth[0];
   for (i=2; i<N; i++,se++){
      ste = se->info->pdf+1;
      for (s=1;s<=S;s++,ste++){
         /* nuisance dimensions not updated */
         vSize = hset->swidth[s]-hset->projSize;
         me = ste->spdf.cpdf + 1; M = ste->nMix;
         for (m=1;m<=M;m++,me++)
            if (me->weight > MINMIX){
               mean = me->mpdf->mean;
               ma = (MuAcc *) GetHook(mean);
               if (ma != NULL){
                  occim = ma->occ;
                  if (occim > 0.0)
                     for (k=1; k<=vSize; k++) 
                       mean[k] += ma->mu[k]/occim;
                  else{
                     M = ste->nMix;
                     HError(-2330,"UpdateMeans: Model %d[%s]: no use of mean %d.%d.%d",
                            px,HMMPhysName(hset,hmm),i,s,m);
                  }
                  SetHook(mean,NULL);
               }
            }
      }
   }
}

/* UpdateTMMeans: use acc values to calc new estimate of means for TIEDHS */
void UpdateTMMeans(HMMSet *hset)
{
   int s,m,k,M,S,vSize;
   float occim;
   MuAcc *ma;
   MixPDF *mpdf;
   Vector mean;
   
   S = hset->swidth[0];
   for (s=1;s<=S;s++){
      vSize = hset->swidth[s];
      M = hset->tmRecs[s].nMix;
      for (m=1;m<=M;m++){
         mpdf = hset->tmRecs[s].mixes[m];
         mean = mpdf->mean;
         ma = (MuAcc *) GetHook(mean);
         if (ma != NULL){
            occim = ma->occ;
            if (occim > 0.0)
               for (k=1; k<=vSize; k++)
                  mean[k] += ma->mu[k]/occim;
            else
               HError(-2330,"UpdateMeans: No use of mean %d in stream %d",m,s);
            SetHook(mean,NULL);
         }
      }
   }
}

/* UpdateVars: use acc values to calc new estimate of variances */
void UpdateVars(HMMSet *hset, int px, HLink hmm)
{
   int i,s,m,k,l,M,N,S,vSize;
   float occim,x,muDiffk,muDiffl;
   Vector minV,mpV;
   VaAcc *va;
   MuAcc *ma;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   Vector mean;
   Covariance cov;
   Boolean mixFloored,shared;
   
   N = hmm->numStates;
   se = hmm->svec+2;
   S = hset->swidth[0];
   for (i=2; i<N; i++,se++){
      ste = se->info->pdf+1;
      for (s=1;s<=S;s++,ste++){
         vSize = hset->swidth[s]-hset->projSize;
         minV = vFloor[s];
         me = ste->spdf.cpdf + 1; M = ste->nMix;
         for (m=1;m<=M;m++,me++)
            if (me->weight > MINMIX){
               if (me->mpdf->vFloor == NULL) mpV=minV;
               else mpV=me->mpdf->vFloor;
               cov = me->mpdf->cov;
               va = (VaAcc *) GetHook(cov.var);
               mean = me->mpdf->mean;
               ma = (MuAcc *) GetHook(mean);
               if (va != NULL){
                  occim = va->occ;
                  mixFloored = FALSE;
                  if (occim > 0.0){
                     shared=(GetUse(cov.var)>1 || ma==NULL || ma->occ<=0.0);
                     if (me->mpdf->ckind==DIAGC) {
                        for (k=1; k<=vSize; k++){
                           muDiffk=(shared)?0.0:ma->mu[k]/ma->occ;
                           x = va->cov.var[k]/occim - muDiffk*muDiffk;
                           if (x<mpV[k]) {
                              x = mpV[k];
                              nFloorVar++;
                              mixFloored = TRUE;
                           }
                           cov.var[k] = x;
                        }
                     }
                     else { /* FULLC */
                        for (k=1; k<=vSize; k++){
                           muDiffk=(shared)?0.0:ma->mu[k]/ma->occ;
                           for (l=1; l<=k; l++){
                              muDiffl=(shared)?0.0:ma->mu[l]/ma->occ;
                              x = va->cov.inv[k][l]/occim - muDiffk*muDiffl; 
                              if (k==l && x<mpV[k]) {
                                 x = mpV[k];
                                 nFloorVar++;
                                 mixFloored = TRUE;
                              }
                              cov.inv[k][l] = x;
                           }
                        }
                        CovInvert(cov.inv,cov.inv);
                     }
                  }
                  else{
                    MixtureElem *me2;
                    me2 = ste->spdf.cpdf + 1; M = ste->nMix;
                    HError(-2330,"UpdateVars: Model %d[%s]: no use of variance %d.%d.%d",
                           px,HMMPhysName(hset,hmm),i,s,m);
                  }
                  if (mixFloored == TRUE) nFloorVarMix++;
                  SetHook(cov.var,NULL);
               }
            }
      }
   }
}

/* UpdateTMVars: use acc values to calc new estimate of vars for TIEDHS */
void UpdateTMVars(HMMSet *hset)
{
   int s,m,k,l,M,S,vSize;
   float occim,x,muDiffk,muDiffl;
   Vector minV;
   VaAcc *va;
   MuAcc *ma;
   MixPDF *mpdf;
   Vector mean;
   Covariance cov;
   Boolean mixFloored,shared;
   
   S = hset->swidth[0];
   for (s=1;s<=S;s++){
      vSize = hset->swidth[s];
      minV = vFloor[s];
      M = hset->tmRecs[s].nMix;
      for (m=1;m<=M;m++){
         mpdf = hset->tmRecs[s].mixes[m];
         cov = mpdf->cov;
         va = (VaAcc *) GetHook(cov.var);
         mean = mpdf->mean;
         ma = (MuAcc *) GetHook(mean);
         if (va != NULL){
            occim = va->occ;
            mixFloored = FALSE;
            if (occim > 0.0){
               shared=(GetUse(cov.var)>1 || ma==NULL || ma->occ<=0.0);
               if (mpdf->ckind==DIAGC) {
                  for (k=1; k<=vSize; k++){
                     muDiffk=(shared)?0.0:ma->mu[k]/ma->occ;
                     x = va->cov.var[k]/occim - muDiffk*muDiffk;
                     if (x<minV[k]) {
                        x = minV[k];
                        nFloorVar++;
                        mixFloored = TRUE;
                     }
                     cov.var[k] = x;
                  }
               }
               else { /* FULLC */
                  for (k=1; k<=vSize; k++){
                     muDiffk=(shared)?0.0:ma->mu[k]/ma->occ;
                     for (l=1; l<=k; l++){
                        muDiffl=(shared)?0.0:ma->mu[l]/ma->occ;
                        x = va->cov.inv[k][l]/occim - muDiffk*muDiffl;
                        if (k==l && x<minV[k]) {
                           x = minV[k];
                           nFloorVar++;
                           mixFloored = TRUE;
                        }
                        cov.inv[k][l] = x;
                     }
                  }
                  CovInvert(cov.inv,cov.inv);
               }
            }
            else
               HError(-2330,"UpdateTMVars: No use of var %d in stream %d",m,s);
            if (mixFloored == TRUE) nFloorVarMix++;
            SetHook(cov.var,NULL);
         }
      }
   }
}

static  int fltcompare(const void *_i, const void *_j)
{
  const float *i = (const float*)_i;
  const float *j = (const float*)_j;
  if (*i > *j)
    return (1);
  if (*i < *j)
    return (-1);
  return (0);
}


void FloorVars(HMMSet *hset1, int s){
  HMMScanState hss1;
  int vsize;
  int i;
  if(!(hset1->hsKind==PLAINHS || hset1->hsKind==SHAREDHS)){
     HError(1, "Percentile var flooring not supported for this kind of hmm set. (e.g. tied.) should be easy.");
  } else { 
     float **varray;
     int M=0,m=0,floored=0;
     vsize = hset1->swidth[s];
     
     NewHMMScan(hset1,&hss1); 
     while(GoNextMix(&hss1,FALSE)){
        if (hss1.s == s) M++;
     }
     EndHMMScan(&hss1); 

     varray = New(&gstack, sizeof(float*) * (vsize+1));
     for(i=1;i<=vsize;i++) varray[i] = New(&gstack, sizeof(float) * M);

     NewHMMScan(hset1,&hss1); 
     while(GoNextMix(&hss1,FALSE)){
        if (hss1.s == s) {
           int k;
           if(hss1.mp->ckind != DIAGC ) HError(1, "FloorVars expects DIAGC covariances. ");
           
           for(k=1;k<=vsize;k++){
              varray[k][m] = hss1.mp->cov.var[k];
           }
           m++;
        }
     }
     EndHMMScan(&hss1); 
     
     for(i=1;i<=vsize;i++){
        qsort((char *) varray[i], M, sizeof(float), fltcompare);
     }

     if(varFloorPercent <=0 || varFloorPercent >= 100) HError(1, "varFloorPercent should be <100 and >0..");
     

     NewHMMScan(hset1,&hss1); 
     while(GoNextMix(&hss1,FALSE)){
        if (hss1.s == s) {
           int k, Pos = (int)(varFloorPercent*0.01*M);
           for(k=1;k<=vsize;k++){
              if(hss1.mp->cov.var[k] < varray[k][Pos]){
                 hss1.mp->cov.var[k] =  varray[k][Pos];
                 floored++;
              }
           }
        }
     }
     EndHMMScan(&hss1); 
     printf("Floored %d (expected to floor %d)\n", floored, (int)( varFloorPercent * 0.01 * M * vsize));
  }
  FixAllGConsts(hset1);
}

void MLUpdateModels(HMMSet *hset, UPDSet uFlags)
{
   HSetKind hsKind;
   HMMScanState hss;
   HLink hmm;
   int px,n,maxM;

   hsKind = hset->hsKind;
   maxM = MaxMixInSet(hset);

   if (hsKind == TIEDHS){ /* TIEDHS - update mu & var once per HMMSet */
      if (uFlags & UPVARS)
         UpdateTMVars(hset);
      if (uFlags & UPMEANS)
         UpdateTMMeans(hset);
      if (uFlags & (UPMEANS|UPVARS))
         FixAllGConsts(hset);
   }

   NewHMMScan(hset,&hss);
   px=1;
   do {   
      hmm = hss.hmm;
      n = (int)hmm->hook;
      if (n<minEgs && !(trace&T_UPD))
         HError(-2331,"UpdateModels: %s[%d] copied: only %d egs\n",
                HMMPhysName(hset,hmm),px,n);
      if (trace&T_UPD) {
         if (n<minEgs)
            printf("Model %s[%d] copied: only %d examples\n",
                   HMMPhysName(hset,hmm),px,n);
         else
            printf("Model %s[%d] to be updated with %d examples\n",
                   HMMPhysName(hset,hmm),px,n);
         fflush(stdout);
      }
      if (n>=minEgs && n>0) {
         if (uFlags & UPTRANS)
            UpdateTrans(hset,px,hmm);
         if (maxM>1 && uFlags & UPMIXES)
            UpdateWeights(hset,px,hmm);
         if (hsKind != TIEDHS){
            if (uFlags & UPVARS)
               UpdateVars(hset,px,hmm);
            if (uFlags & UPMEANS)
               UpdateMeans(hset,px,hmm);
            if (uFlags & (UPMEANS|UPVARS)) 
               FixGConsts(hmm);
         }  
      }
      px++;
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (trace&T_TOP) {
      if (nFloorVar > 0)
         printf("Total %d floored variance elements in %d different mixes\n",
                nFloorVar,nFloorVarMix);
      fflush(stdout);
   }
}


/* UpdateModels: update all models and save them in newDir if set,
   new files have newExt if set */
void UpdateModels(HMMSet *hset, ParmBuf pbuf2)
{
   int maxM;
   static char str[100];
   BufferInfo info2;
   char macroname[MAXSTRLEN];

   if (trace&T_UPD){
      printf("Starting Model Update\n"); fflush(stdout);
   }
   if (parMode == -1) {
      ForceDiagC(hset); /* invert the variances again */
      ConvExpWt(hset);
   }
   maxM = MaxMixInSet(hset);

   /* 
      This routine tidies up the semi-tied transform and 
      tidies the model up. The transition and priors are 
      not updated 
   */
   if (uFlags & UPSEMIT) {
      UpdateSemiTiedModels(hset, &xfInfo);
      uFlags = uFlags & ~(UPMEANS|UPVARS);
   }

   if (uFlags & UPMAP)
     MAPUpdateModels(hset, uFlags);
   else {
     MLUpdateModels(hset, uFlags);     
   }
   
   if(varFloorPercent){
      int s;
      printf("Flooring all vars to the %f'th percentile of distribution... ", varFloorPercent);
      for(s=1;s<=hset->swidth[0];s++)
         FloorVars(hset,s);
   }

   if (trace&T_TOP){
      if (mmfFn == NULL)
         printf("Saving hmm's to dir %s\n",(newDir==NULL)?"Current":newDir); 
      else
         printf("Saving hmm's to MMF %s\n",mmfFn);
      fflush(stdout);
   }
   ClearSeenFlags(hset,CLR_ALL);
   if (twoDataFiles){
      if (parMode == 0){
         SetChannel("HPARM2");
         nParm = GetConfig("HPARM2", TRUE, cParm, MAXGLOBS);
         if (GetConfStr(cParm,nParm,"TARGETKIND",str))
            hset->pkind = Str2ParmKind(str);
	 if (GetConfStr(cParm,nParm,"MATTRANFN",str)) {
            /* The transform needs to be set-up */
            hset->xf = LoadInputXForm(hset,NameOf(str,macroname),str);
         }
      } else {
         GetBufferInfo(pbuf2,&info2);
         hset->pkind = info2.tgtPK;
	 hset->xf = info2.xform;
      }
   }
   SaveHMMSet(hset,newDir,newExt,NULL,saveBinary);
   if (trace&T_TOP) {
      printf("Reestimation complete - average log prob per frame = %e\n",
             totalPr/totalT);
      printf("     - total frames seen          = %e\n", (double)totalT);
   }
}

/* ----------------------------------------------------------- */
/*                      END:  HERest.c                         */
/* ----------------------------------------------------------- */





