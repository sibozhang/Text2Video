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
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*              2003  M.J.F. Gales and                         */
/*                    Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HAdapt.c      Adaptation Library module       */
/* ----------------------------------------------------------- */

char *hadapt_version = "!HVER!HAdapt:   3.4.1  [CUED 12/03/09]";
char *hadapt_vc_id =  "$Id: HAdapt.c,v 1.2 2006/12/07 11:09:07 mjfg Exp $";


#include <stdio.h>      /* Standard C Libraries */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "HShell.h"     /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HWave.h"
#include "HAudio.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HTrain.h"
#include "HUtil.h"
#include "HAdapt.h"
#include "HFB.h"

/* trace flags */
#define T_TOP   00001    /* Top level tracing */
#define T_ADT   00002    /* Trace number of adapted components  */
#define T_ACC   00004    /* Trace number of accumulates generated  */
#define T_TRE   00010    /* Trace use of regression class tree  */
#define T_XFM   00020    /* Trace generation of xforms  */
#define T_SXF   00040    /* Speaker xform updates */
#define T_OBC   00100    /* Trace observation cache */
#define T_SWP   00200    /* Trace transform manipulation */
#define T_FRS   00400    /* Trace fisher ratio selection */

/* -------------- Structures to store adaptation info -------------------- */

typedef struct {
   XFormKind xkind;
   int dim;
   double occ;  
   IntVec blockSize;
   DVector *K, D;
   DMatrix *G;
} AccStruct;

typedef struct _AInfo {
   int baseClass;
   int level;
   struct _AInfo *next;            /* next external file name in list */
} AInfo;

typedef struct {
   Vector mean;
   Covariance cov;
   float gConst;
} MInfo;

typedef struct _ObsCache{
   int time;
   Vector obs;
   float det;
   struct _ObsCache *next;
} ObsCache;                        /* observation cache to save rotated observations */

typedef struct _AccCache{
   int     baseclass;
   DVector bVector;
   TriMat  *bTriMat;
   struct _AccCache *next;
} AccCache;                       /* acc cache to save accumulators related to parent XForm */  

typedef struct {
   float occ;
   Vector spSum;
   Vector spSumSq;
   TriMat *bTriMat;
   TriMat *bDiagMat;
   DVector bVector;
   Vector  obsVec;
} RegAcc;

typedef struct {
   AInfo *aInfo;         /* current transform information */
   MInfo *mInfo;         /* any original model information */
   AInfo *paInfo;        /* parent transform information */
   RegAcc *regAcc;       /* accumulate information for generating transform */
   ObsCache *oc;         /* observation cache for input transform */
   ObsCache *paoc;       /* observation cache for parent transform */
   AccCache *paac;       /* accummulator cache for parent transform */
} XFormInfo;

/* General variables */
static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;
static int trace = 0;                   /* trace info */

/* Global information about current set of transforms */
/* 
static AdaptXForm* curXForm = NULL;
static AdaptXForm* parentXForm = NULL;
*/
static AdaptXForm* outXForm = NULL;
static AdaptXForm* diagCovXForm = NULL;

/* Local stack to allow storage of internal structrures */
static MemHeap infoStack;
static MemHeap acccaStack;
static MemHeap bobcaStack; /* base obscache */
static MemHeap pobcaStack; /* parent obscache */

/* Global variables */
static XFormKind xKind     = MLLRMEAN;  /* Transform Kind to be created */

/* also have the option of storing a model set for each of the speakers */
static Boolean saveSpkrModels = FALSE;

/* The xform config variable information */
static float minOccThresh = 0.0;       /* minimum occupancy to accumulate stats to estimate xform */
static Boolean useBias = TRUE;         /* whether a bias is to be estimated for the xform */
static Boolean storeMInfo = TRUE;      /* whether original model information  is to be stored */
static Boolean keepXFormDistinct = TRUE;
static Boolean swapXForms = FALSE;     /* swap the transforms around after generating transform */
static Boolean mllrCov2CMLLR= FALSE;   /* apply mllrcov transforms as cmllr transform */ 
static Boolean mllrDiagCov = FALSE;    /* perform diagonal covariance adaptation */

static IntVec enableBlockAdapt = NULL;

/* split threshold definitions for each xform kind */
static float xformSplitThresh = -1000.0;
static float mllrMeanSplitThresh = 1000.0;
static float mllrCovSplitThresh = 1000.0;
static float cmllrSplitThresh = 1000.0;

/* adaptation kind  definitions for each xform kind */
static AdaptKind xformAdaptKind = BASE;
static AdaptKind mllrMeanAdaptKind = BASE;
static AdaptKind mllrCovAdaptKind = BASE;
static AdaptKind cmllrAdaptKind = BASE;

/* regression tree definitions for each xform kind */
static char *xformRegTree = NULL;
static char *mllrMeanRegTree = NULL;
static char *mllrCovRegTree = NULL;
static char *cmllrRegTree = NULL;

/* baseclass definitions for each xform kind */
static char *xformBaseClass = NULL;
static char *mllrMeanBaseClass = NULL;
static char *mllrCovBaseClass = NULL;
static char *cmllrBaseClass = NULL;

/* block size definitions for each xform kind */
static IntVec xformBlockSize = NULL;
static IntVec mllrMeanBlockSize = NULL;
static IntVec mllrCovBlockSize = NULL;
static IntVec cmllrBlockSize = NULL;

/* current time when this changes accumulate complete stats */
/* -1 indicates that this is the first frame of a new file */
static int baseTriMatTime=-1;  
static double maxXFormIter = 10; /* something big, for CMLLR */ 
static ObsCache *headpoc = NULL; 
static ObsCache *headboc = NULL; 
static AccCache *headac = NULL;

/* new variables to support semi-tied transforms */
static float semiTiedFloorScale = 0.1;
static int maxSemiTiedIter = 10;
static char *semiTiedMacro = NULL;
static Boolean semiTied2InputXForm = FALSE;
static Boolean semiTiedVFloor = TRUE;
static int numNuisanceDim = 0;
static TriMat *avCov = NULL;
static Boolean staticSemiTied = FALSE;
static Boolean initNuisanceFR = TRUE;
static Boolean initNuisanceFRIdent = FALSE;
static Boolean saveSemiTiedBinary = FALSE;

/* specifies whether the transforms change the model variances */
static Boolean covarChanged = FALSE;
static Boolean covarPChanged = FALSE;

/*------------------------------------------------------------------------*/
/*    Support Routines for determining internal structures required       */
/*    Note: these only act on the transform NOT any parents.              */
/*------------------------------------------------------------------------*/

static Boolean AccAdaptMean(AdaptXForm *xform)
{
   /* Currently always true */
   if (xform->xformSet->xkind == SEMIT) 
      return (FALSE);
   else
      return (TRUE);
}

static Boolean AccAdaptVar(AdaptXForm *xform)
{
  XFormKind xkind = xform->xformSet->xkind;
  if ( (xkind == CMLLR) || (xkind == MLLRCOV) || (mllrDiagCov)) 
    return (TRUE);
  else
    return (FALSE);
}

static Boolean AccAdaptBaseTriMat(AdaptXForm *xform)
{
  XFormKind xkind = xform->xformSet->xkind;
  if ( (xkind == CMLLR) || (xkind == MLLRCOV)) 
    return (TRUE);
  else
    return (FALSE);
}

Boolean HardAssign(AdaptXForm *xform)
{
   AdaptKind akind = xform->akind;
   return ((akind == TREE) || (akind == BASE));
}

static Boolean StoreObsCache(AdaptXForm *xform)
{
   XFormKind xkind = xform->xformSet->xkind;
   return ((xkind == CMLLR)  || (xkind == MLLRCOV) || (xkind == SEMIT));
}

static Boolean StoreAdaptMean(AdaptXForm *xform)
{
   XFormKind xkind = xform->xformSet->xkind;
   return ((xkind == MLLRMEAN) || (xkind == MLLRCOV));
}

static Boolean StoreAdaptCov(AdaptXForm *xform)
{
   XFormKind xkind = xform->xformSet->xkind;
   return (xkind == MLLRVAR);
}

/*------------------------------------------------------------------------*/
/*            Initialisations and general structures allocation           */
/*------------------------------------------------------------------------*/

static void CheckAdaptOptions()
{
   if ((!keepXFormDistinct) && (swapXForms))
      HError(999,"Cannot save swapped XForms in a TMF");
   if (((xKind == MLLRCOV)||(xKind == SEMIT)) && (useBias))
      HError(999,"Cannot have a Bias with a Full variance transform");
   if ((mllrDiagCov) && (xKind != MLLRMEAN))
      HError(999,"Cannot have mllrDiagCov and not have MLLRMEAN");
   if ((!swapXForms) && (mllrCov2CMLLR))
      HError(999,"Cannot save mllrCov as CMLLR");
   if (strmProj && ((xKind != SEMIT) || (numNuisanceDim>0)))
      HError(999,"Can only have strmProj with SEMIT");
   if (numNuisanceDim > 0) {
      if (xKind != SEMIT) 
         HError(999,"Can only use HLDA with global SEMIT transforms");
   }
   if (staticSemiTied) {
      if (xKind != SEMIT) 
         HError(999,"Can only use ststic semiTied  with SEMIT transforms");
   }
}

/* ParseConfIntVec: interpret config string as integer array */
static IntVec ParseConfIntVec(MemHeap *x, char *inbuf)
{
   IntVec ivec = NULL;
   int size,cnt;
   char buf[MAXSTRLEN],tbuf[MAXSTRLEN];

   if (sscanf(inbuf,"%s",buf)>0) {
      if (strcmp(buf,"IntVec") != 0)
         HError(999,"ParseConfIntVec: format is IntVec d d d ....");
      inbuf=strstr(inbuf,"IntVec")+strlen("IntVec");
      sscanf(inbuf,"%d",&size);
      sprintf(tbuf,"%d",size);
      inbuf=strstr(inbuf,tbuf)+(int)strlen(tbuf);
      ivec = CreateIntVec(x,size);
      cnt = 1;
      while ((strlen(inbuf)>0) && (cnt<=size) &&
             (sscanf(inbuf,"%d",&(ivec[cnt])))) {
         sprintf(tbuf,"%d",ivec[cnt]);
         inbuf=strstr(inbuf,tbuf)+(int)strlen(tbuf);
         cnt++;
      }
      if (strlen(inbuf)>0)
         HError(999,"ParseConfIntVec: residual elements - format is  n b1 ... bn");
   } else 
      HError(999,"ParseConfIntVec: format is  n b1 ... bn");
   return ivec;
}

/* EXPORT->InitAdapt: initialise configuration parameters */
void InitAdapt (XFInfo *xfinfo) 
{
   int i;
   Boolean b;
   double d;
   char buf[MAXSTRLEN];
  
   Register(hadapt_version,hadapt_vc_id);
   nParm = GetConfig("HADAPT", TRUE, cParm, MAXGLOBS);

   /* setup the local memory management - defaults sensible? */
   CreateHeap(&infoStack,"InfoStore", MSTAK, 1, 1.0, 50000, 500000);
   CreateHeap(&acccaStack,"AccStore", MSTAK, 1, 1.0, 50000, 500000);
   CreateHeap(&bobcaStack,"baseObsStore", MSTAK, 1, 1.0, 50000, 500000);
   CreateHeap(&pobcaStack,"parentObsStore", MSTAK, 1, 1.0, 50000, 500000);

   if (nParm>0){
      /* general adaptation config variables */
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfFlt(cParm,nParm,"MINOCCTHRESH",&d)) minOccThresh = (float) d;
      if (GetConfBool(cParm,nParm,"STOREMINFO",&b)) storeMInfo = b;
      if (GetConfBool(cParm,nParm,"KEEPXFORMDISTINCT",&b)) keepXFormDistinct = b;
      if (GetConfBool(cParm,nParm,"SAVESPKRMODELS",&b)) saveSpkrModels = b;
      /* Adaptation transformation set-up */
      if (GetConfBool(cParm,nParm,"USEBIAS",&b)) useBias = b;
      if (GetConfFlt(cParm,nParm,"SPLITTHRESH",&d)) xformSplitThresh = (float) d;
      if (GetConfStr (cParm,nParm,"TRANSKIND",buf)) xKind = Str2XFormKind(buf);
      if (GetConfStr (cParm,nParm,"BLOCKSIZE",buf)) 
	 xformBlockSize = ParseConfIntVec(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"BASECLASS",buf)) 
         xformBaseClass = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"REGTREE",buf)) 
         xformRegTree = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"ADAPTKIND",buf)) 
         xformAdaptKind = Str2AdaptKind(buf);
      if (GetConfInt(cParm,nParm,"MAXXFORMITER",&i)) maxXFormIter = i;
      if (GetConfBool(cParm,nParm,"MLLRDIAGCOV",&b)) mllrDiagCov = b;      
      if (GetConfBool(cParm,nParm,"SWAPXFORMS",&b)) swapXForms = b;      
      if (GetConfBool(cParm,nParm,"MLLRCOV2CMLLR",&b)) mllrCov2CMLLR = b; 

      if (GetConfFlt(cParm,nParm,"SEMITIEDFLOOR",&d)) semiTiedFloorScale = (float) d;
      if (GetConfInt(cParm,nParm,"MAXSEMITIEDITER",&i)) maxSemiTiedIter = i;
      if (GetConfStr (cParm,nParm,"SEMITIEDMACRO",buf)) 
         semiTiedMacro = CopyString(&infoStack,buf);
      if (GetConfBool(cParm,nParm,"SEMITIED2INPUTXFORM",&b)) semiTied2InputXForm = b; 
      if (GetConfBool(cParm,nParm,"SEMITIEDVFLOOR",&b)) semiTiedVFloor = b; 
      if (GetConfInt(cParm,nParm,"NUMNUISANCEDIM",&i)) numNuisanceDim = i;
      if (GetConfBool(cParm,nParm,"STATICSEMITIED",&b)) staticSemiTied = b; 
      if (GetConfBool(cParm,nParm,"INITNUISANCEFR",&b)) initNuisanceFR = b; 
      if (GetConfBool(cParm,nParm,"INITNUISANCEFRIDENT",&b)) initNuisanceFRIdent = b; 
      if (GetConfBool(cParm,nParm,"SAVESEMITIEDBINARY",&b)) saveSemiTiedBinary = b; 

      /* Backward compatibility with old configuration options */
      /* MLLRMEAN specification */
      if (GetConfFlt(cParm,nParm,"MLLRMEANSPLITTHRESH",&d)) mllrMeanSplitThresh = (float) d;
      if (GetConfStr (cParm,nParm,"MLLRMEANBLOCKSIZE",buf)) 
         mllrMeanBlockSize = ParseConfIntVec(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"MLLRMEANBASECLASS",buf)) 
         mllrMeanBaseClass = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"MLLRMEANREGTREE",buf)) 
         mllrMeanRegTree = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"MLLRMEANADAPTKIND",buf)) 
         mllrMeanAdaptKind = Str2AdaptKind(buf);
      /* MLLRCOV specification */      
      if (GetConfFlt(cParm,nParm,"MLLRCOVSPLITTHRESH",&d)) mllrCovSplitThresh = (float) d;
      if (GetConfStr (cParm,nParm,"MLLRCOVBLOCKSIZE",buf)) 
         mllrCovBlockSize = ParseConfIntVec(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"MLLRCOVBASECLASS",buf)) 
         mllrCovBaseClass = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"MLLRCOVREGTREE",buf)) 
         mllrCovRegTree = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"MLLRCOVADAPTKIND",buf)) 
         mllrCovAdaptKind = Str2AdaptKind(buf);
      /* CMLLR specification */
      if (GetConfFlt(cParm,nParm,"CMLLRSPLITTHRESH",&d)) cmllrSplitThresh = (float) d;
      if (GetConfStr (cParm,nParm,"CMLLRBLOCKSIZE",buf))
         cmllrBlockSize = ParseConfIntVec(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"CMLLRBASECLASS",buf))
         cmllrBaseClass = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"CMLLRREGTREE",buf))
         cmllrRegTree = CopyString(&infoStack,buf);
      if (GetConfStr (cParm,nParm,"CMLLRADAPTKIND",buf))
         cmllrAdaptKind = Str2AdaptKind(buf);
   }

   /* Initialise the XFInfo values */
   xfinfo->outSpkrPat = "*.%%%";
   xfinfo->inSpkrPat = NULL;
   xfinfo->paSpkrPat = NULL;
   xfinfo->outXFormExt = NULL;
   xfinfo->inXFormExt = NULL;
   xfinfo->paXFormExt = NULL;
   xfinfo->outXFormDir = NULL;
   xfinfo->paXFormDir = NULL;
   xfinfo->useOutXForm = FALSE;
   xfinfo->useInXForm = FALSE;
   xfinfo->usePaXForm = FALSE;
   xfinfo->xformTMF = NULL;
   xfinfo->inXForm = NULL;
   xfinfo->outXForm = NULL;
   xfinfo->paXForm = NULL;
   xfinfo->al_hset = NULL;
   xfinfo->alXFormExt = NULL;
   xfinfo->alXFormDir = NULL;
   CheckAdaptOptions();
}

/* Additional code to parse configs to for appropriate thresholds */

static float GetSplitThresh(AdaptXForm *xform)
{
   float thresh=0.0;

   if (xformSplitThresh > 0) {
     thresh = xformSplitThresh;
   } else {
     switch(xform->xformSet->xkind) {
     case MLLRMEAN:
       thresh = mllrMeanSplitThresh;
       break;
     case MLLRCOV:
       thresh = mllrCovSplitThresh;
       break;
     case CMLLR:
       thresh = cmllrSplitThresh;
       break;
     }
   }
   return thresh;
}

static AdaptKind GetAdaptKind(AdaptXForm *xform)
{
   AdaptKind akind = BASE;

   if (xformAdaptKind != BASE) {
     akind = xformAdaptKind;
   } else {
     switch(xform->xformSet->xkind) {
     case MLLRMEAN:
       akind = mllrMeanAdaptKind;
       break;
     case MLLRCOV:
       akind = mllrCovAdaptKind;
       break;
     case CMLLR:
       akind = cmllrAdaptKind;
       break;
     }
   }
   return akind;
}

static RegTree* GetRegTree(HMMSet *hset, AdaptXForm *xform)
{
   char* basename = NULL;
   char macroname[MAXSTRLEN];

   if (xformRegTree != NULL) {
     basename = xformRegTree;
   } else {
     switch(xform->xformSet->xkind) {
     case MLLRMEAN:
       basename = mllrMeanRegTree;
       break;
     case MLLRCOV:
       basename = mllrCovRegTree;
       break;
     case CMLLR:
       basename = cmllrRegTree;
       break;
     }
   }
   if (basename == NULL) {
      /* must have specified a regression class tree with TREE adapt kind */
      HError(999, "GetRegTree: No regression tree macro name specified\n");
   } 
   return LoadRegTree(hset,NameOf(basename,macroname),basename);  
}

static BaseClass* GetBaseClass(HMMSet *hset,AdaptXForm *xform)
{
   char* basename = NULL;
   char macroname[MAXSTRLEN];

   if (xformBaseClass != NULL) {
     basename = xformBaseClass;
   } else {
     switch(xform->xformSet->xkind) {
     case MLLRMEAN:
       basename = mllrMeanBaseClass;
       break;
     case MLLRCOV:
       basename = mllrCovBaseClass;
       break;
     case CMLLR:
       basename = cmllrBaseClass;
       break;
     }
   }
   if (basename == NULL) {
      HError(-1,"No baseclass macro name specified - global transform assumed");
      basename = "global";
   }
   /* name may be a complete path, or just the macroname */
   return LoadBaseClass(hset,NameOf(basename,macroname),basename);
}

static int GetVecSizeClass(BaseClass *bclass, int class)
{
   ILink i;
   MixtureElem *me;

   /* currently does not check consistency of vector sizes */
   i=bclass->ilist[class];
   me = (MixtureElem *)i->item;
   return VectorSize(me->mpdf->mean);
}

static IntVec GetBlockSize(AdaptXForm *xform, int class)
{
   IntVec blockSize = NULL;

   if (xformBlockSize != NULL) {
     blockSize = xformBlockSize;
   } else {
     switch(xform->xformSet->xkind) {
     case MLLRMEAN:
       blockSize = mllrMeanBlockSize;
       break;
     case MLLRCOV:
       blockSize = mllrCovBlockSize;
       break;
     case CMLLR:
       blockSize = cmllrBlockSize;
       break;
     }
   }
   if (strmProj) {
      if (blockSize != NULL)
         HError(-1,"For stream projection only full transform used");
      blockSize = CreateIntVec(xform->mem,1);
      blockSize[1] = xform->hset->vecSize;
   } else if (blockSize == NULL) {    
      blockSize = CreateIntVec(xform->mem,1);
      blockSize[1] = GetVecSizeClass(xform->bclass,class);
   }
   return blockSize;  
}

/*------------------------------------------------------------------------*/
/*                      Internal Structure Support                        */
/*------------------------------------------------------------------------*/

/* ----------------- Access Routines for the structures  ---------------- */

static RegAcc *GetRegAcc(MixPDF *mp)
{
   return ((XFormInfo *)mp->info)->regAcc;
}

static AInfo *GetAInfo(MixPDF *mp)
{
   return ((XFormInfo *)mp->info)->aInfo;
}

static AInfo *GetPAInfo(MixPDF *mp)
{
   return ((XFormInfo *)mp->info)->paInfo;
}

static MInfo *GetMInfo(MixPDF *mp)
{
   return ((XFormInfo *)mp->info)->mInfo;
}

static ObsCache *GetObsCache(MixPDF *mp)
{
  return ((XFormInfo *)mp->info)->oc;
}

static ObsCache *GetPAObsCache(MixPDF *mp)
{
  return ((XFormInfo *)mp->info)->paoc;
}

static AccCache *GetPAAccCache(MixPDF *mp)
{
  return ((XFormInfo *)mp->info)->paac;
}


/* --------------- handling the MInfo structure ------------------ */

static MInfo *CreateMInfo(MemHeap *x, MixPDF *mp, AdaptXForm *xform)
{
   MInfo *mi;
   Boolean adaptMean, adaptCov;
   AdaptXForm *xf;
   int size;

   size = VectorSize(mp->mean);
   adaptMean = FALSE; adaptCov = FALSE;
   mi = (MInfo *)New(&infoStack,sizeof(MInfo));
   /* depending on the nature of the transform determines the 
      parameters to be stored */
   xf = xform;
   while (xf != NULL) {
      if (StoreAdaptMean(xf)) adaptMean = TRUE;
      if (StoreAdaptCov(xf)) adaptCov = TRUE;
      xf = xf->parentXForm;
   }
   if (adaptMean) {
      mi->mean = CreateVector(x,size);
      CopyVector(mp->mean,mi->mean);
   } else 
      mi->mean = NULL;
   if (adaptCov) {
      switch(mp->ckind){
      case DIAGC:
      case INVDIAGC:
         mi->cov.var = CreateVector(x,size);
         CopyVector(mp->cov.var,mi->cov.var);
         mi->gConst = mp->gConst;
         break;
      default:
         HError(999,"AccMixPDFStats: bad ckind %d",mp->ckind);
      }
   } else {
      switch(mp->ckind){
      case DIAGC:
      case INVDIAGC:
         mi->cov.var = NULL;
         break;
      default:
         HError(999,"AccMixPDFStats: bad ckind %d",mp->ckind);
      }
   }
   return mi;
}

/* Function used to store the original model parameters. May
   be overriden using 
    HMODEL:STOREMINFO = FALSE
*/
static void SetMInfo(HMMSet *hset, AdaptXForm *xform)
{
   HMMScanState hss;
   MixPDF *mp;
   int nMInfo=0;
  
   NewHMMScan(hset,&hss);
   do {
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {            
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  mp = hss.mp;
                  ((XFormInfo *)mp->info)->mInfo = CreateMInfo(hset->hmem,mp,xform);
                  nMInfo++;
               }
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (trace&T_ACC) printf("Attached %d MInfo structures\n",nMInfo);
   hset->attMInfo = TRUE;
}

static Boolean CompareMInfo(HMMSet *hset, AdaptXForm *xform)
{
   Boolean adaptMean, adaptCov;
   AdaptXForm *xf;
   HMMScanState hss;
   MixPDF *mp;
   MInfo *mi;

   adaptMean = FALSE; adaptCov = FALSE;
   /* depending on the nature of the transform determines the 
      parameters to be stored */
   xf = xform;
   while (xf != NULL) {
      if (StoreAdaptMean(xf)) adaptMean = TRUE;
      if (StoreAdaptCov(xf)) adaptCov = TRUE;
      xf = xf->parentXForm;
   }
   /* now check to see what is currently stored */
   NewHMMScan(hset,&hss);
   mp = hss.mp;
   mi = GetMInfo(mp);
   if ((adaptMean) && (mi->mean == NULL)) {
      EndHMMScan(&hss);
      return FALSE;
   }
   if ((adaptCov) && (mi->cov.var == NULL)) {
      EndHMMScan(&hss);
      return FALSE;
   }
   EndHMMScan(&hss);
   return TRUE;
}

/* 
   If the model info has already been set need to check that there are 
   no new parameters that need to be stored - add additional 
   storage as required
*/
static void UpdateMInfo(HMMSet *hset, AdaptXForm *xform)
{
   Boolean adaptMean, adaptCov;
   AdaptXForm *xf;
   int size;
   HMMScanState hss;
   MixPDF *mp;
   MInfo *mi;
   int nMInfo=0;

   adaptMean = FALSE; adaptCov = FALSE;
   /* depending on the nature of the transform determines the 
      parameters to be stored */
   xf = xform;
   while (xf != NULL) {
      if (StoreAdaptMean(xf)) adaptMean = TRUE;
      if (StoreAdaptCov(xf)) adaptCov = TRUE;
      xf = xf->parentXForm;
   }
   /* now check to see what is currently stored */
   NewHMMScan(hset,&hss);
   do {
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {            
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  mp = hss.mp;
                  mi = GetMInfo(mp);
                  if ((adaptMean) && (mi->mean == NULL)) {
		     size = VectorSize(mp->mean);
                     mi->mean = CreateVector(hset->hmem,size);
                     CopyVector(mp->mean,mi->mean);
                     nMInfo++;
                  } 
                  if (adaptCov) {
                     switch(mp->ckind){
                     case DIAGC:
                     case INVDIAGC:
                        if (mi->cov.var == NULL) {
			   size = VectorSize(mp->mean);
                           mi->cov.var = CreateVector(hset->hmem,size);
                           CopyVector(mp->cov.var,mi->cov.var);
                           mi->gConst = mp->gConst;
                        }
                        break;
                     default:
                        HError(999,"AccMixPDFStats: bad ckind %d",mp->ckind);
                     }
                     nMInfo++;
                  }
               }
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (trace&T_ACC) printf("Attached %d additional MInfo structures\n",nMInfo);
}

/* --------------- handling the XFormInfo structure ------------------ */

static XFormInfo *CreateXFormInfo(MemHeap *x)
{
   XFormInfo *info;

   info = (XFormInfo *)New(x,sizeof(XFormInfo));
   info->aInfo  = NULL;
   info->mInfo = NULL;
   info->paInfo = NULL;
   info->regAcc = NULL;
   info->paoc = NULL;
   info->oc = NULL;
   info->paac = NULL;
   return info;
}

static void AttachXFormInfo(HMMSet *hset)
{
   HMMScanState hss;
   MixPDF *mp;
   int nXFormInfo=0;
  
   NewHMMScan(hset,&hss);
   do {
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {            
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  mp = hss.mp;
                  mp->info = (XFormInfo *)CreateXFormInfo(hset->hmem);
                  nXFormInfo++;
               }
            else
               HError(7450, "AttachXFormInfo: Adaptation only available for PLAIN or SHARED systems!");
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss); 
   if (trace&T_ACC) printf("Attached %d XFormInfo structures\n",nXFormInfo);
   hset->attXFormInfo = TRUE;
}

static Boolean CompareXFormInfo(AdaptXForm *xform1, AdaptXForm *xform2)
{
   AdaptXForm *xf1, *xf2;
   Boolean valid=FALSE;

   if (xform1 == xform2) return TRUE;
   xf1 = xform1; xf2 = xform2;
   while ((xf1 != NULL) && (xf2 != NULL)) {
      if (xf1->bclass != xf2->bclass)
         return valid;
      xf1 = xf1->parentXForm;
      xf2 = xf2->parentXForm;
   }
   /* check that they are both the same length */
   if ((xf1 == NULL) && (xf2 == NULL))
      valid = TRUE;
   return valid;
}

/* --------------- handling the AInfo structure ------------------ */

static void SetAInfo(HMMSet *hset, AdaptXForm *xform, Boolean parent)
{
   BaseClass *bclass;
   MixPDF *mp;
   ILink i;
   int b, nlevel;
   AdaptXForm *xf;
   AInfo *ai;
   int nAInfo=0;

   if (!hset->attXFormInfo) AttachXFormInfo(hset);
   /* transform baseclasses differ reset internal information */
   /* ResetHeap(&infoStack); */
   if (xform != NULL) {
      /* setup the adptation information for each component 
         according to the baseclass information.
         Expect the itemlist to always specify components .... */
      nlevel = 0; xf = xform;
      while (xf != NULL) {
         nlevel++;
         bclass = xf->bclass;
         for (b = 1; b <= bclass->numClasses; b++) {
            for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
               mp = ((MixtureElem *)i->item)->mpdf;
               if (parent) ai = ((XFormInfo *)mp->info)->paInfo;
               else ai = ((XFormInfo *)mp->info)->aInfo;
               if (ai == NULL) {
                  if (parent) 
                     ((XFormInfo *)mp->info)->paInfo = ai = (AInfo *)New(&infoStack,sizeof(AInfo));
                  else ((XFormInfo *)mp->info)->aInfo = ai = (AInfo *)New(&infoStack,sizeof(AInfo));
                  nAInfo++;
               } else if (nlevel>1) { 
                  /* go the end of the chain and add adaptation info, but not at the first level */
                  /* a fix to tidy memory at this stage is required */
                  while (ai->next != NULL) ai=ai->next;
                  ai = ai->next = (AInfo *)New(&infoStack,sizeof(AInfo));
                  nAInfo++;
               }
               ai->baseClass = b; ai->level = nlevel; ai->next = NULL;
            }
         }
         xf = xf->parentXForm;
      }
   } else { /* This may be called during a reset so set all the ai's to NULL */
      HMMScanState hss;
      NewHMMScan(hset,&hss);
      do {
         while (GoNextState(&hss,TRUE)) {
            while (GoNextStream(&hss,TRUE)) {            
                  while (GoNextMix(&hss,TRUE)) {
                     ((XFormInfo *)hss.mp->info)->aInfo = NULL;
                  }
            }
         }
      } while (GoNextHMM(&hss));
      EndHMMScan(&hss);
   }
   if (trace&T_ACC) printf("Attached %d AInfo structures\n",nAInfo);
}

/* --------------- handling the RegAcc structure ------------------ */

/* Using stored information reset all the model parameters */
static void ResetComp(MixPDF *mp)
{
   MInfo *mi;

   mi = GetMInfo(mp); 
   if ( mi != NULL) { /* Initial model parameters have been stored */
      if (mi->mean != NULL) CopyVector(mi->mean,mp->mean);
      switch (mp->ckind) {
      case DIAGC:
      case INVDIAGC:
         if (mi->cov.var != NULL) {
            CopyVector(mi->cov.var,mp->cov.var);
            mp->gConst = mi->gConst;
         }
         break;
      default:
         HError(999,"ResetComp: bad ckind %d",mp->ckind);
      }
   } 
}

static TriMat *CreateBlockTriMat(MemHeap *x, IntVec blockSize)
{
  TriMat *tm;
  int nblock, bsize, b, *i;
  
  nblock = IntVecSize(blockSize);
  tm = (TriMat *)New(x,sizeof(TriMat)*(nblock+1));  
  i = (int *)tm; *i = nblock;
  for (b=1;b<=nblock;b++) {
    bsize = blockSize[b];
    tm[b] = CreateTriMat(x, bsize); 
    ZeroTriMat(tm[b]);
  }
  return(tm);
}

static void ZeroBlockTriMat(TriMat *bTriMat)
{
  int *nblock,b;

  nblock = (int *)bTriMat;
  for (b=1; b<=*nblock; b++)
      ZeroTriMat(bTriMat[b]);  
}

static void ZeroBaseTriMat(TriMat *bTriMat)
{
  int i;
  int *vsize;
  TriMat tm;
 
  vsize = (int *)bTriMat;
  for (i=1;i<=*vsize;i++) {
    tm = bTriMat[i];
    ZeroTriMat(tm);
  }  
}

static void CreateBaseTriMat(MemHeap *x, MixPDF *mp, AdaptXForm *xform, int class)
{
  TriMat *tm;
  int vsize = VectorSize(mp->mean);
  IntVec blockSize = GetBlockSize(xform,class);
  RegAcc *regAcc, *ra;
  MixPDF *me;
  BaseClass *bclass;
  ILink i;
  int j, cntj, *vsp, b, bsize;

  regAcc = GetRegAcc(mp);
  if (xform->info->accBTriMat) { 
    regAcc->bVector =  CreateDVector(x,vsize);
    ZeroDVector(regAcc->bVector);
    regAcc->obsVec =  CreateVector(x,vsize);
    ZeroVector(regAcc->obsVec);
    regAcc->bDiagMat = CreateBlockTriMat(x,blockSize); 
    ZeroBlockTriMat(regAcc->bDiagMat);
    tm = (TriMat *)New(x,sizeof(TriMat)*(vsize+1));
    vsp = (int *)tm; *vsp = vsize;
    for (b=1,cntj=1;b<=IntVecSize(blockSize);b++) {
      bsize = blockSize[b];
      for (j=1;j<=bsize;j++,cntj++) {
	tm[cntj] =  CreateTriMat(x, bsize);  
	ZeroTriMat(tm[cntj]);
      }
    }
    regAcc->bTriMat = tm;    
  } else regAcc->bTriMat = NULL; 

  bclass = xform->bclass;
  for (i=bclass->ilist[class]; i!=NULL; i=i->next) { 
    if (xform->info->accBTriMat) {
      me = ((MixtureElem *)i->item)->mpdf;
      if( me != mp ) {
        ra = GetRegAcc(me);
        ra->bVector = regAcc->bVector;
        ra->bDiagMat = regAcc->bDiagMat;
        ra->bTriMat = regAcc->bTriMat;
        ra->obsVec = regAcc->obsVec;
      }
    } else regAcc->bTriMat = NULL;
  }      
}

void SetBaseAccsTime(int t)
{
   baseTriMatTime = t;
}


void UpdateAccCache(double Lr, Vector svec, MixPDF *mp)
{
   AccCache *paac;
   TriMat m;
   int vsize = VectorSize(svec);
   Vector covar;
   int i, j, bl, bstart, nblock, bsize; 

   paac = GetPAAccCache(mp);    
   if ( paac != NULL ) {
      /* This needs to be altered so that it does not rely on a non-zero first element */
      if ( paac->bTriMat[1][1][1] == 0 ) {
	nblock = (int)(paac->bTriMat[0]);
	for (bl=1,bstart=0;bl<=nblock;bl++) {
	  m = paac->bTriMat[bl];
	  bsize = TriMatSize(m);
	  for (i=1;i<=bsize;i++) { /* Fill the accumulate stores */
	    for (j=1; j<=i; j++)
		m[i][j] = svec[i+bstart] * svec[j+bstart];
	  }
	  bstart += bsize;
	}
      }
      covar = mp->cov.var;
      for (i=1;i<=vsize;i++) {
         if (mp->ckind==INVDIAGC)
            paac->bVector[i] += covar[i]*Lr;
         else
            paac->bVector[i] += Lr/covar[i];
      }
   }
}

void UpdateBaseAccs(Vector svec)
{
   int i,j,b,k, bsize, nblock, bl;
   int cnt, cnti, cntj;
   TriMat tm, m;
   RegAcc *ra;
   DVector acc;
   Vector ovec;
   BaseClass *bclass;
   MixPDF *mp;
   
   bclass = outXForm->bclass;
   for (b=1;b<=bclass->numClasses;b++) {
      mp = ((MixtureElem *)(bclass->ilist[b])->item)->mpdf;
      ra = GetRegAcc(mp);
      if ((ra->bTriMat != NULL) && (ra->bVector[1]>0)) {    
         acc = ra->bVector;
         nblock = (int)(ra->bDiagMat[0]);

         /* generate the outer-product */
         /* moved to here for speed with large numbers of baseclasses */
         ovec = ra->obsVec;
         for (bl=1, cnt=1; bl<=nblock;bl++){
            bsize = TriMatSize(ra->bDiagMat[bl]);
            m = ra->bDiagMat[bl];
            for (i=1, cnti=cnt; i<=bsize; i++,cnti++) { /* Fill the outer product */
               for (j=1,cntj=cnt; j<=i; j++,cntj++)
                  m[i][j] = ovec[cnti]*ovec[cntj];
            }
            cnt +=bsize;
         }

    for (bl=1,cnti=1;bl<=nblock;bl++) {
      m = ra->bDiagMat[bl];
      bsize = TriMatSize(m);
      for (i=1;i<=bsize;i++,cnti++) { /* Fill the accumulate stores */
        tm = ra->bTriMat[cnti];
        for (j=1; j<=bsize; j++)
          for (k=1; k<=j; k++)
            tm[j][k] += m[j][k] * acc[cnti];
      }
         }
         ZeroDVector(ra->bVector);
      }
      /* now update the observation cache */
      if (svec != NULL) {
         for (i=1;i<=VectorSize(svec);i++)
            ra->obsVec[i] = svec[i];
      }
   }
}


void UpdateBaseAccsWithPaac(void)
{
   int i,j,k, b, bsize, nblock, bl;
   int cnti;
   TriMat tm, m;
   RegAcc *ra;
   DVector acc;
   BaseClass *bclass;
   MixPDF *mp;
   AccCache *paac;

   bclass = outXForm->bclass;

   for (b=1;b<=bclass->numClasses;b++) {
      mp = ((MixtureElem *)(bclass->ilist[b])->item)->mpdf;
      ra = GetRegAcc(mp);

      if ( ra->bTriMat != NULL) {
        for (paac = headac; paac!= NULL; paac=paac->next) { 
          if ( (paac->baseclass == b)&& (paac->bVector[1]>0) ){
            acc = paac->bVector;
            nblock = (int)(ra->bDiagMat[0]);
            for (bl=1,cnti=1;bl<=nblock;bl++) {
              m = paac->bTriMat[bl];
              bsize = TriMatSize(m);
              for (i=1;i<=bsize;i++,cnti++) { /* Fill the accumulate stores */
                tm = ra->bTriMat[cnti];
                for (j=1; j<=bsize; j++)
                  for (k=1; k<=j; k++)
                    tm[j][k] += m[j][k] * acc[cnti];
              }
            }
          }
        }
      }
   }  
}

void ResetAccCache(void)
{
   AccCache *ac;
 
   if ( headac != NULL) {
      for (ac = headac; ac!= NULL; ac=ac->next) {
        ZeroDVector(ac->bVector);
        ZeroBlockTriMat(ac->bTriMat);
      }
   }
}

static Boolean XFormModCovar(AdaptXForm *xform)
{
  Boolean isModified = FALSE;

  while (xform != NULL){
    if (StoreAdaptCov(xform)){
      isModified = TRUE;
      break;
    }
    xform = xform->parentXForm;
  }
  return isModified;
}

static void AccBaseTriMat(HMMSet *hset, double Lr, Vector svec, MixPDF *mp, int t)
{
   int vsize, i;
   /* TriMat m; */
   Vector covar;
   RegAcc *regAcc;
   Boolean resetMixPDF = FALSE;
  
   if (((hset->parentXForm == NULL) && (hset->curXForm == NULL) ) || (hset->parentXForm == hset->curXForm))  {
      /* There's nothing to be done as model set the same */
      resetMixPDF = FALSE;
   } else if (hset->parentXForm == NULL) {
      /* xform to be built on original parameters */
      /* reset the models if the inputXForm alters the variances */
      if(covarChanged){
	      ResetComp(mp);
         resetMixPDF = TRUE;
      }
   } else {
      /* xform to be built on a parent xform */
      if (covarPChanged || covarChanged){
         ApplyCompXForm(mp,hset->parentXForm);
         resetMixPDF = TRUE;
      }
   }
   
   regAcc = GetRegAcc(mp);
   covar = mp->cov.var;
   vsize = VectorSize(svec);

   if (t != baseTriMatTime) {
     /* Check to see whether this is the very first frame */
     if (headac == NULL ) 
       UpdateBaseAccs(svec);
     else  {
       UpdateBaseAccsWithPaac(); 
       ResetAccCache();   
     }
     SetBaseAccsTime(t);
   }

   if  (headac == NULL) {    
     for (i=1;i<=vsize;i++) {
       if (mp->ckind==INVDIAGC)
	 regAcc->bVector[i] += covar[i]*Lr;
       else
	 regAcc->bVector[i] += Lr/covar[i];
     }
   } else
     UpdateAccCache( Lr, svec, mp);

   if (resetMixPDF) {
     ApplyCompXForm(mp,hset->curXForm);
   }
}

static RegAcc *CreateRegAcc(MemHeap *x, MixPDF *mp, AdaptXForm *xform)
{
  RegAcc *regAcc;
  int vsize = VectorSize(mp->mean);
  
  regAcc = (RegAcc *)New(x,sizeof(RegAcc));
  regAcc->occ = 0;
  if (xform->info->accSum) {
    regAcc->spSum = CreateVector(x,vsize);
    ZeroVector(regAcc->spSum);
  } else regAcc->spSum = NULL;
  if (xform->info->accSumSq) {
    regAcc->spSumSq = CreateVector(x,vsize);
    ZeroVector(regAcc->spSumSq);
  } else regAcc->spSumSq = NULL;
  regAcc->bTriMat = NULL;   
  return regAcc;
}

static void AttachRegAccs(HMMSet *hset, AdaptXForm *xform)
{  
  MixPDF *mp = NULL;
  int nRegAcc=0, b;
  BaseClass *bclass;
  ILink i;

  /* RegAccs stored on the Info structure */
  if (!hset->attXFormInfo) AttachXFormInfo(hset);
  bclass = xform->bclass;
  for (b=1;b<=bclass->numClasses;b++) {
    for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
      mp = ((MixtureElem *)i->item)->mpdf;
      ((XFormInfo *)(mp->info))->regAcc = (RegAcc *)CreateRegAcc(hset->hmem,mp,xform);
      nRegAcc++;   
    }
    /* Use last component of the baseclass to access baseclass stats */
    CreateBaseTriMat(hset->hmem,mp,xform,b); 
  }

  if (trace&T_ACC) printf("Attached %d RegAcc structures\n",nRegAcc);
  hset->attRegAccs = TRUE;
}

/* --------------- handling the AccStruct structure ------------------ */

static AccStruct *CreateAccStruct(MemHeap *x, AdaptXForm *xform, 
				  int vsize, IntVec blockSize)
{
  AccStruct *accs;
  int dim,i,cnti;
  int b,bsize; 

  accs = (AccStruct *)New(x,sizeof(AccStruct));
  accs->occ = 0;
  accs->dim = vsize;
  accs->blockSize = blockSize;
  accs->K = NULL;
  accs->G = NULL;
  accs->D = NULL;
  /* Depending on the form of transform initialise appropriate
     structure elements */
  accs->xkind = xform->xformSet->xkind;
  switch (accs->xkind) {
  case MLLRMEAN:
    accs->K = (DVector *)New(x,(vsize+1)*sizeof(DVector));
    accs->G = (DMatrix *)New(x,(vsize+1)*sizeof(DMatrix));
    for (b=1,cnti=1;b<=IntVecSize(blockSize);b++) {
      bsize = blockSize[b];
      if (useBias) dim = bsize+1;
      else dim = bsize;
      for (i=1;i<=bsize;i++,cnti++) {
	accs->K[cnti] = CreateDVector(x,dim);
	ZeroDVector(accs->K[cnti]);
	accs->G[cnti] = CreateDMatrix(x,dim,dim);
	ZeroDMatrix(accs->G[cnti]);
      }
    }
    if (mllrDiagCov) {
      accs->D = CreateDVector(x,vsize);
      ZeroDVector(accs->D);
    }
    break;
  case MLLRCOV:
    accs->G = (DMatrix *)New(x,(vsize+1)*sizeof(DMatrix));
    for (b=1,cnti=1;b<=IntVecSize(blockSize);b++) {
      bsize = blockSize[b];
      for (i=1;i<=bsize;i++,cnti++) {
        accs->G[cnti] = CreateDMatrix(x,bsize,bsize);
        ZeroDMatrix(accs->G[cnti]);
      }
    }
    break;
  case SEMIT:
    accs->G = (DMatrix *)New(x,(vsize+1)*sizeof(DMatrix));
    /* For stream projections CANNOT have multiple blocks */
    if (strmProj) {
       for (i=1;i<=vsize;i++) {
          accs->G[i] = CreateDMatrix(x,vsize,vsize);
          ZeroDMatrix(accs->G[i]);
       }
    } else {
       for (b=1,cnti=1;b<=IntVecSize(blockSize);b++) {
          bsize = blockSize[b];
          for (i=1;i<=bsize;i++,cnti++) {
             if ((staticSemiTied) && (b>1)){ 
                /* link all accumulates into one */
                accs->G[cnti] = accs->G[i];
             } else {
                accs->G[cnti] = CreateDMatrix(x,bsize,bsize);
                ZeroDMatrix(accs->G[cnti]);
             }
          }
       }
    }
    break;
  case CMLLR:
    accs->K = (DVector *)New(x,(vsize+1)*sizeof(DVector));
    accs->G = (DMatrix *)New(x,(vsize+1)*sizeof(DMatrix));
    for (b=1,cnti=1;b<=IntVecSize(blockSize);b++) {
      bsize = blockSize[b];
      if (useBias) dim = bsize+1;
      else dim = bsize;
      for (i=1;i<=bsize;i++,cnti++) {
        accs->K[cnti] = CreateDVector(x,dim);
        ZeroDVector(accs->K[cnti]);
        accs->G[cnti] = CreateDMatrix(x,dim,dim);
        ZeroDMatrix(accs->G[cnti]);
      }
    }
    break;
  default :
    HError(999,"Transform kind not currently supported");
    break;
  }
  return accs;
}


/* 
   A separate global floor is set for each of the streams.
   For systems with mixed streams for a baseclass no 
   variance floors is applied.
*/
static void SetSemiTiedAvCov(HMMSet *hset)
{
   HMMScanState hss;
   StateInfo *si   ;
   StreamElem *ste;
   MixPDF *mp;
   STriMat inv;
   DVector mean;
   DMatrix *mat, *smat, cov;
   DVector *svec;
   double occAcc, occ, tmp; 
   int k,kk,s,S;
   MuAcc *ma;
   VaAcc *va;
   int vsize,m;

   /* allocate accumulators */
   S= hset->swidth[0];
   mat = (DMatrix *)New(&gstack,(S+1)*sizeof(DMatrix));
   smat = (DMatrix *)New(&gstack,(S+1)*sizeof(DMatrix));
   svec = (DVector *)New(&gstack,(S+1)*sizeof(DVector));
   avCov = (TriMat *)New(hset->hmem,(S+1)*sizeof(TriMat));
   for (s=1;s<=S;s++) {
      if (strmProj) vsize = hset->vecSize;
      else vsize = hset->swidth[s];
      mat[s] = CreateDMatrix(&gstack,vsize,vsize);
      smat[s] = CreateDMatrix(&gstack,vsize,vsize);
      svec[s] = CreateDVector(&gstack,vsize);
      /* create on HMMSet memory heap */
      avCov[s] = CreateTriMat(hset->hmem,vsize);
      ZeroDMatrix(mat[s]);
   }
   NewHMMScan(hset,&hss);
   occAcc = 0.0;
   while(GoNextState(&hss,FALSE)) {
      si = hss.si;
      while (GoNextStream(&hss,TRUE)) {
         s = hss.s;
         if (strmProj) vsize = hset->vecSize;
         else vsize = hset->swidth[s];
         cov = smat[s]; ZeroDMatrix(cov);
         mean = svec[s]; ZeroDVector(mean);
         ste = hss.ste; 
         occ = 0;
         for (m=1; m<= hss.M; m++) {
            mp = ste->spdf.cpdf[m].mpdf;
            ma = (MuAcc *)GetHook(mp->mean);
            va = (VaAcc *)GetHook(mp->cov.var);
            if (fabs(ma->occ-va->occ) > 0.1)
               HError(999,"State-clustered systems only for semitied updates");
            if (va->occ > 0) {
               inv = va->cov.inv;
               occ += va->occ;
               for (k=1;k<=vsize;k++) { 
                  mean[k] += ma->mu[k];
                  for (kk=1;kk<=k;kk++) 
                     cov[k][kk] += inv[k][kk];
               }
            }
         }
         /* accumulate in */
         if (occ > 0) {
            for (k=1;k<=vsize;k++) { 
               tmp = mean[k]/occ;
               for (kk=1;kk<=k;kk++) 
                  mat[s][k][kk] += cov[k][kk] - tmp*mean[kk];
            }
         }
         occAcc += occ;
      }
   }
   EndHMMScan(&hss);
   
   /* normalisation and put in avCov */
   for (s=1;s<=S;s++) {
      if (strmProj) vsize = hset->vecSize;
      else vsize = hset->swidth[s];
      for (k=1;k<=vsize;k++)
         for (kk=1;kk<=k;kk++)
            avCov[s][k][kk] = mat[s][k][kk]/occAcc;
   }
   Dispose(&gstack,mat);
}

static void FixSemiTiedStats(HMMSet *hset)
{
   /* 
      This uses the standard HMM stats and sets the stream number for each 
      component (future support for multiple stream systems).
   */
   HMMScanState hss;
   MixPDF *mp;
   int k,kk;
   float tmp;
   VaAcc *va;
   MuAcc *ma;
   Vector mu;
   STriMat inv;

   NewHMMScan(hset,&hss);
   do {
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {            
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  mp = hss.mp; 
                  mp->stream = hss.s;
                  /* 
                     fix the statistics to be zero-mean out-product 
                  */
                  va = (VaAcc *) GetHook(mp->cov.var);
                  if (va->occ>0) {
                     ma = (MuAcc *) GetHook(mp->mean);
                     inv = va->cov.inv;
                     mu = ma->mu;
                     for (k=1;k<=VectorSize(mu);k++) {
                        tmp = mu[k]/va->occ;
                        for (kk=1;kk<=k;kk++)
                           inv[k][kk] -= tmp*mu[kk];
                     }
                  }
               }
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
}

/*------------------------------------------------------------------------*/
/*                       Regression Tree Parsing                          */
/*------------------------------------------------------------------------*/

static float SetNodeOcc(RegNode *node, BaseClass *bclass)
{
   int c;
   ILink i;
   MixPDF *mp=NULL;
   int dim,b;

   node->nodeOcc = 0;
   if (node->numChild>0) {
      for (c=1;c<=node->numChild;c++) {
         /* Check dimensionality of children is consistent */
         node->nodeOcc += SetNodeOcc(node->child[c],bclass);
         dim = (node->child[c])->vsize;
         if (node->vsize>0) {
            if (dim != node->vsize)
               HError(999,"Inconsistent dimensions in baseclasses (%d %d)",dim,node->vsize);
         } else 
            node->vsize = dim;
      }
   } else {
      for (b=1;b<=IntVecSize(node->baseClasses);b++) {
         for (i=bclass->ilist[node->baseClasses[b]]; i!=NULL; i=i->next) {
            mp = ((MixtureElem *)i->item)->mpdf;
            node->nodeOcc += GetRegAcc(mp)->occ;
         }
         /* Baseclass definition ensures that dimensions are correct within a baseclass */
         dim = VectorSize(mp->mean);
         if (node->vsize>0) {
            if (dim != node->vsize)
               HError(999,"Inconsistent dimensions in baseclasses (%d %d)",dim,node->vsize);
         } else 
            node->vsize = dim;
      }
   }
   return node->nodeOcc;
}

static Boolean ParseNode(RegNode *node, AdaptXForm *xform, 
			 RegTree *rtree, IntVec classes)
{
   int b,c,size;
   Boolean genXForm;
   IntVec lclasses;

   void GenXForm(RegNode *node, AdaptXForm *xform, IntVec classes);  

   genXForm = FALSE;
   if (trace&T_TRE) printf("Node %d (%f)\n",node->nodeIndex,node->nodeOcc);
   if (node->nodeOcc > rtree->thresh) {
      size = IntVecSize(classes);
      lclasses = CreateIntVec(&gstack,IntVecSize(classes));
      ZeroIntVec(lclasses);
      if (node->numChild>0) { /* Not a terminal node */
         for (c=1;c<=node->numChild;c++)
            if (ParseNode(node->child[c], xform, rtree, lclasses)) genXForm = TRUE;
         /* any of the children need a xform generate it */
         if (genXForm) GenXForm(node,xform,lclasses);
      } else { /* Generate xform for this node */
         for (b=1;b<=IntVecSize(node->baseClasses);b++) lclasses[node->baseClasses[b]] = 1;
         GenXForm(node,xform,lclasses);
      }
      FreeIntVec(&gstack,lclasses);
      genXForm = FALSE;
   } else {
      if (node->numChild>0) { /* Not a terminal node */
         for (c=1;c<=node->numChild;c++)
            ParseNode(node->child[c], xform, rtree, classes);
      } else { /* Mark baseclasses for adaptation */
         for (b=1;b<=IntVecSize(node->baseClasses);b++) classes[node->baseClasses[b]] = 1;
      }
      genXForm = TRUE;
   }
   return genXForm;
}

static Boolean ParseTree(RegTree *rtree, AdaptXForm *xform)
{
   int c;
   IntVec classes;
   float occ;

   /* First set the correct threshold for this tree */
   rtree->thresh = GetSplitThresh(xform);
   occ = SetNodeOcc(rtree->root, rtree->bclass);
   if (!(rtree->valid)) { /* multiple streams being used */
      /* the total number of observations will be given by the roots child */
      /* should be approximately the same for all children */
      occ = rtree->root->child[1]->nodeOcc;
   }
   if (occ<rtree->thresh) /* not enough data to generate transforms */
      return FALSE;
   /* reset the number of transforms */
   xform->xformSet->numXForms = 0;
   if (xform->xformSet->xkind == SEMIT) 
      HError(999,"No support for semi-tied transforms with regression class trees");
   classes = CreateIntVec(&gstack,rtree->bclass->numClasses);
   if (rtree->valid) { /* dimensionality of all transforms the same */
      ZeroIntVec(classes);
      ParseNode(rtree->root, xform, rtree, classes);
   } else {
      for (c=1;c<=rtree->root->numChild;c++) {
         ZeroIntVec(classes);
         ParseNode(rtree->root->child[c], xform, rtree, classes);
      }
   }
   FreeIntVec(&gstack,classes);

   if (xform->xformSet->numXForms == 0) return FALSE;
   else return TRUE;
}

/*------------------------------------------------------------------------*/
/*                 Accumulation of statistics from Tree                   */
/*------------------------------------------------------------------------*/

static void Tri2DMat (DMatrix m1, DMatrix m2)
{
  int i,j,nrows,ncols;

  nrows = NumDRows(m2); ncols = NumDCols(m2);
  if (nrows != ncols)
    HError(5270,"Tri2Mat: target matrix not square %d vs %d",
	   nrows,ncols);   
  /* if (ncols != TriMatSize(m1)) 
    HError(5270,"Tri2Mat: sizes differ %d vs %d",
	   TriMatSize(m1),ncols);
  */
  if (ncols != NumDRows(m1))
    HError(5270,"Tri2Mat: sizes differ %d vs %d",
	   NumDRows(m1),ncols);
  for (i=1; i<=nrows; i++)
    for (j=1; j<=i; j++) {
      m2[i][j] = m1[i][j];
      if (i!=j) m2[j][i] = m1[i][j];
    }
}

static void AccCMLLRBaseStats(MixPDF *mp, AccStruct *accs)
{
  /* update for the accumulates at the base level */
  RegAcc *ra;
  int i,j,k;
  int cnti,b,bsize;
  TriMat tm;
 
  ra = GetRegAcc(mp);
  for (b=1,cnti=1;b<=IntVecSize(accs->blockSize);b++) {
    bsize = accs->blockSize[b];
    for (i=1;i<=bsize;i++,cnti++) {
      tm = ra->bTriMat[cnti];
      for (j=1;j<=bsize;j++) {
         for (k=1;k<=j;k++) {
            accs->G[cnti][j][k] += tm[j][k];
         }      
      }
    }
  }
}
 
static void AccCMLLRPDFStats(MixPDF *mp,  AccStruct *accs)
{
  RegAcc *ra;
  int i,j;
  float icov=0.0,scale;
  int cnt,cnti,cntj,b,bsize;
  Vector mean;
  Covariance cov;
 
  ra = GetRegAcc(mp);
  mean = mp->mean;
  cov = mp->cov;
  for (b=1,cnti=1,cnt=1;b<=IntVecSize(accs->blockSize);b++) {
    bsize = accs->blockSize[b];
    for (i=1;i<=bsize;i++,cnti++) {
      switch(mp->ckind){
      case INVDIAGC:
        icov = cov.var[cnti];
        break;
      case DIAGC:
        icov = 1/cov.var[cnti];
        break;
      default:
        HError(999,"AccMixPDFStats: bad ckind %d",mp->ckind);
      }
      scale = ra->occ * icov;
      for (j=1,cntj=cnt;j<=bsize;j++,cntj++) {
        accs->K[cnti][j] += ra->spSum[cntj] * mean[cnti] * icov;
        if (useBias)
          accs->G[cnti][bsize+1][j] += icov * ra->spSum[cntj];
      }
      if (useBias) {
        accs->K[cnti][bsize+1] += scale * mean[cnti];
        accs->G[cnti][bsize+1][bsize+1] += scale;
      }
    }
    cnt += bsize;
  }
}

static void AccMLLRPDFStats(MixPDF *mp,  AccStruct *accs)
{
   RegAcc *ra;
   int i,j,k;
   float icov=0.0,scale;
   int cnt,cnti,cntj,cntk,b,bsize;
   Vector mean;
   Covariance cov;

   ra = GetRegAcc(mp);
   mean = mp->mean;
   cov = mp->cov;
   for (b=1,cnti=1,cnt=1;b<=IntVecSize(accs->blockSize);b++) {
      bsize = accs->blockSize[b];
      for (i=1;i<=bsize;i++,cnti++) {
         switch(mp->ckind){
         case INVDIAGC:
            icov = cov.var[cnti]; 
            break;
         case DIAGC:
            icov = 1/cov.var[cnti]; 
            break;
         default:
            HError(999,"AccMixPDFStats: bad ckind %d",mp->ckind);
         }
         scale = ra->occ * icov;
         for (j=1,cntj=cnt;j<=bsize;j++,cntj++) {
            accs->K[cnti][j] += ra->spSum[cnti] * mean[cntj] * icov;
            for (k=1,cntk=cnt;k<=j;k++,cntk++)
               accs->G[cnti][j][k] += scale * mean[cntj] * mean[cntk];
            if (useBias)
               accs->G[cnti][bsize+1][j] += scale * mean[cntj];
         }
         if (useBias) {
            accs->K[cnti][bsize+1] += ra->spSum[cnti] *icov;
            accs->G[cnti][bsize+1][bsize+1] += scale;
         }
	 if (mllrDiagCov) {
	   accs->D[cnti] += icov * ra->spSumSq[cnti];
	 }
      }
      cnt += bsize;
   }
}

static void AccMLLRCOVPDFStats(MixPDF *mp,  AccStruct *accs)
{
  RegAcc *ra;
  int i,j,k;
  float icov=0.0,scale, c1, c2, c3;
  int cnt,cnti,cntj,cntk,b,bsize;
  Vector mean;
  Covariance cov;

  ra = GetRegAcc(mp);
  mean = mp->mean;
  cov = mp->cov;
  for (b=1,cnti=1,cnt=1;b<=IntVecSize(accs->blockSize);b++) {
    bsize = accs->blockSize[b];
    for (i=1;i<=bsize;i++,cnti++) {
      switch(mp->ckind){
      case INVDIAGC:
	icov = cov.var[cnti]; 
	break;
      case DIAGC:
	icov = 1/cov.var[cnti]; 
	break;
      default:
	HError(999,"AccMixPDFStats: bad ckind %d",mp->ckind);
      }
      scale = ra->occ * icov;
      for (j=1,cntj=cnt;j<=bsize;j++,cntj++) {
         for (k=1,cntk=cnt;k<=j;k++,cntk++) {
            c1 = scale * mean[cntj] * mean[cntk] ;
            c2 = ra->spSum[cntj] * mean[cntk] * icov;
            c3 = ra->spSum[cntk] * mean[cntj] * icov;  
            accs->G[cnti][j][k] += (c1 - c2 -c3);
         }	
      }    
    }
    cnt += bsize;
  }
}

static void AccMixPDFStats(HMMSet *hset, MixPDF *mp, AccStruct *accs)
{
  RegAcc *ra;

  ra = GetRegAcc(mp);
  if (ra->occ > minOccThresh) {
    accs->occ += ra->occ;
    if (((hset->parentXForm == NULL) && (hset->curXForm == NULL) ) || (hset->parentXForm == hset->curXForm))  {
      /* There's nothing to be done as model set the same */
    } else if (hset->parentXForm == NULL) { 
      /* xform to be built on original parameters */
      ResetComp(mp);
    } else {
      /* xform to be built on a parent xform */
      ApplyCompXForm(mp,hset->parentXForm);
    }
    switch (accs->xkind) {
    case MLLRMEAN:
      AccMLLRPDFStats(mp,accs);
      break;
    case MLLRCOV:
      AccMLLRCOVPDFStats(mp,accs);           
      break;
    case CMLLR:
      AccCMLLRPDFStats(mp,accs);
      break;
    case SEMIT:
     /* The accstructure is not used for semi-tied estimation */
       break;
    default :
      HError(999,"Transform kind not currently supported");
      break;
    }
  }
}

static void AccBaseClassStats(MixPDF *mp, AccStruct *accs)
{
	/* RegAcc *ra; */

  /* 
     Accumulate the statistics for the base classes. The 
     parent transforms have been already sorted.
  */
  switch (accs->xkind) {
  case MLLRCOV:
  case CMLLR:
    AccCMLLRBaseStats(mp,accs);
    break;
  default :
    HError(999,"Transform kind not currently supported");
    break;
  }
}

static void AccNodeStats(RegNode *node, AccStruct *accs, 
			 AdaptXForm *xform, IntVec classes)
{
  BaseClass *bclass;
  ILink i;
  int b,c;
  MixPDF *mp = NULL;
  

  if (node->numChild>0) {
    for (c=1;c<=node->numChild;c++)
      AccNodeStats(node->child[c],accs,xform,classes);
  } else {
    bclass = xform->bclass;
    for (b=1;b<=IntVecSize(node->baseClasses);b++) {
      for (i=bclass->ilist[node->baseClasses[b]]; i!=NULL; i=i->next) {
	mp = ((MixtureElem *)i->item)->mpdf;
	AccMixPDFStats(xform->hset,mp,accs);
      }
      /* Use last component of the baseclass to access baseclass stats */
      if( AccAdaptBaseTriMat(xform) )  AccBaseClassStats(mp,accs);
    }
  }
}

/* Feature-Space adaptation */
static void FixDet(LinXForm *xf)
{
   int ind,nblock;
   double scale, bdet;
   float det;
 
   nblock = IntVecSize(xf->blockSize);
   if ( nblock == xf->vecSize) {   
      det=0;
      for (ind=1;ind<=xf->vecSize;ind++) {
         scale = xf->xform[ind][1][1];
         det += log(scale*scale);
      }
      xf->det = det;
   } else {   
      det=0; 
      for (ind=1;ind<=nblock;ind++) {
         bdet = MatDet(xf->xform[ind]);
         det += 2*log(fabs(bdet));
      }
      xf->det = det;
   }
}
/*------------------------------------------------------------------------*/
/*     Accummulator Cache for application of parent XForms                */
/*------------------------------------------------------------------------*/
static AccCache  *CreateAccCache(IntVec size,  int b)
{
   AccCache *ac;
   int vsize, bl;

   vsize = 0;
   for (bl=1;bl<=IntVecSize(size);bl++) vsize += size[bl];
   
   ac = (AccCache *)New(&acccaStack,sizeof(AccCache));
   ac->baseclass = b;
   ac->bVector  = CreateDVector(&acccaStack,vsize);
   ZeroDVector(ac->bVector);
   ac->bTriMat = CreateBlockTriMat(&acccaStack,size);
   ZeroBlockTriMat(ac->bTriMat);
   ac->next = headac;
   headac = ac;
   return(ac);
}

static void SetAccCache(AdaptXForm *xform)
{
   MixPDF *mp;
   BaseClass *bclass;
   int b;
   ILink i;
   AccCache **ac = NULL;
   int nxflevel = 0, nxfcomb = 1, numXf = 0, nxf, ind;
   int nCache = 0;
   AInfo *ai;
   AdaptXForm *xf;
   XFormSet  *xformSet;
   HMMSet *hset;
   
   if ((xform != NULL) && (AccAdaptBaseTriMat(xform))) {
      hset = xform->hset;
      if (hset->parentXForm != NULL) {
        xform->parentXForm = hset->parentXForm;
        xform->parentXForm->nUse++;
      } else
        xform->parentXForm = NULL;    

      nxflevel = 1; 
      nxfcomb *= (xform->bclass->numClasses + 1);
      xf = xform->parentXForm;
      /* Count the number of levels and combinations */
      while ( xf != NULL ) {
         if (StoreObsCache(xf)) {
            nxfcomb *= (xf->xformSet->numXForms + 1);
            nxflevel++;
         }
         xf = xf->parentXForm;
      }
 
      if (nxflevel>0) {
	ac = (AccCache **)New(&gstack,sizeof(AccCache *)*(nxfcomb+1));
         for ( ind = 0; ind <= nxfcomb; ind++)
            ac[ind] = NULL;
      }
 
      bclass = xform->bclass;
      for (b = 1; b <= bclass->numClasses; b++) {
         for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
            mp = ((MixtureElem *)i->item)->mpdf;
            if (nxflevel  == 0)    ((XFormInfo *)mp->info)->paac = NULL;
            else {
               ai = GetPAInfo(mp);
               nxf = xform->bclass->numClasses + 1;
               ind = b;
               xf = xform->parentXForm;
               while ( xf != NULL ){
                  xformSet = xf->xformSet;
                  if (StoreObsCache(xf)) {
                     if (HardAssign(xform))
                        numXf = xf->xformWgts.assign[ai->baseClass];
                     else
                        HError(999,"Not currently supported");
                     ind += numXf * nxf;
                     nxf *= (xformSet->numXForms + 1);
                  }
                  xf = xf->parentXForm;
                  if ( xf != NULL )  ai = ai->next;
               }
               if (nxflevel > 0) {
                  if (ind > 0) { /* support no transform has been generated */
                    if ( ac[ind] == NULL )  {
                      ac[ind] = CreateAccCache(GetBlockSize(xform,b), b);
                      nCache++;
                    }
                  }
                  ((XFormInfo *)mp->info)->paac = ac[ind];
               }
            }
         }
      }
      if (ac != NULL) Dispose(&gstack,ac);
      ac = NULL;
   }
   if (trace&T_TOP)
     printf("Created %d AccCaches (of %d possible)\n",nCache,nxfcomb);
}

/*------------------------------------------------------------------------*/
/*      Observation Cache for application of feature-space transform      */
/*------------------------------------------------------------------------*/

static ObsCache *CreateObsCache(MemHeap *heap, ObsCache **headoc, int size)
{
   ObsCache  *oc;

   oc = (ObsCache *)New(heap,sizeof(ObsCache));
   oc->time = -1;
   oc->obs = CreateVector(heap,size);
   ZeroVector(oc->obs);
   oc->det =0;  
 
   oc->next = *headoc;
   *headoc = oc;

   return(oc);
}
  

static void SetObsCache(AdaptXForm *xform, Boolean parent)
{
   MixPDF *mp;
   BaseClass *bclass;
   int b, size, nxflevel = 0, nxfcomb = 1, numXf = 0, nxf, ind, nCache = 0;
   ILink i;
   ObsCache **headoc, **oc = NULL;
   AInfo *ai;
   AdaptXForm *xf;
   XFormSet *xformSet;
   MemHeap *heap;

   if (xform != NULL) {

    if (parent) {
      ResetHeap(&pobcaStack);
      heap = &pobcaStack;
      headpoc = NULL;
      headoc = &headpoc;
    }
    else {
      ResetHeap(&bobcaStack);
      heap = &bobcaStack;
      headboc = NULL;
      headoc = &headboc;
    }

    xf = xform;
    /* Count the number of levels and combinations */
    while ( xf != NULL ) {
      if (StoreObsCache(xf)) {
   nxfcomb *= (xf->xformSet->numXForms + 1);
   nxflevel++;
      }
      xf = xf->parentXForm;
    }
    
    if (nxflevel>0) {
      oc = (ObsCache **)New(&gstack,sizeof(ObsCache *)*(nxfcomb+1));
      for ( ind = 0; ind <= nxfcomb; ind++) {
   oc[ind] = NULL;
      }
    }
       
    bclass = xform->bclass;
    for (b = 1; b <= bclass->numClasses; b++) {
      for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
   mp = ((MixtureElem *)i->item)->mpdf;
   if (nxflevel  == 0) {
     if (parent) {
       ((XFormInfo *)mp->info)->paoc = NULL;
     }
     else ((XFormInfo *)mp->info)->oc = NULL;
   } else {
     if (parent) ai = GetPAInfo(mp);
     else  ai = GetAInfo(mp);

     xf = xform; nxf = 1; 
     ind = 0;  
     while ( xf != NULL ){
       xformSet = xf->xformSet;
       if (StoreObsCache(xf)) {
         if (HardAssign(xform)) 
      numXf = xf->xformWgts.assign[ai->baseClass];
         else
      HError(999,"Not currently supported");
         ind += numXf * nxf;
         nxf *= (xformSet->numXForms + 1);
       } 
       xf = xf->parentXForm;
       ai = ai->next;
     }   
     if (nxflevel > 0) {
       if (ind > 0) { /* support no transform has been generated */
         size = VectorSize(mp->mean);
         if ( oc[ind] == NULL )  {
      oc[ind] = CreateObsCache(heap, headoc, size);
      nCache++;
         }
       }
       if (parent) ((XFormInfo *)mp->info)->paoc = oc[ind];
       else ((XFormInfo *)mp->info)->oc = oc[ind];
                 
     }
   }
      } 
    }
    if (oc != NULL) Dispose(&gstack,oc);
    oc = NULL;
   }
   if (trace&T_TOP)
     printf("Created %d ObsCaches (of %d possible)\n",nCache,nxfcomb);
}

static void UpdateObsCache( ObsCache *oc, Vector svec, LogFloat det, int t)  
{

   if (oc != NULL ) {  
      if (t != oc->time) {
         oc->time = t;
         CopyVector(svec, oc->obs);
         oc->det = det;
      } 
   }
}

void ResetObsCache(void)
{
   ObsCache *oc;
      
   if ( headboc != NULL) {
     for (oc = headboc; oc!= NULL; oc=oc->next) {
       oc->time = -1;
       ZeroVector(oc->obs);
       oc->det = 0;
     }
   }
   if ( headpoc != NULL) {
     for (oc = headpoc; oc!= NULL; oc=oc->next) {
       oc->time = -1;
       ZeroVector(oc->obs);
       oc->det = 0;
     }
   }
}

/*------------------------------------------------------------------------*/
/*                          Adaptation Application                         */
/*------------------------------------------------------------------------*/

static void ApplyXForm2Vector(LinXForm *linXForm, Vector mean)
{  
   Vector vec, bias;
   int size,b,bsize;
   Matrix A;
   float tmp;
   int i,j;
   int cnt,cnti,cntj;

   /* Check dimensions */
   size = linXForm->vecSize;
   if (size != VectorSize(mean))
      HError(999,"Transform dimension (%d) does not match mean dimension (%d)",
             size,VectorSize(mean));
   vec = CreateVector(&gstack,size);
   CopyVector(mean,vec); ZeroVector(mean);
   /* Transform mean */
   for (b=1,cnti=1,cnt=1;b<=IntVecSize(linXForm->blockSize);b++) {
      bsize = linXForm->blockSize[b];
      A = linXForm->xform[b];
      for (i=1;i<=bsize;i++,cnti++) {
         tmp = 0;
         for (j=1,cntj=cnt;j<=bsize;j++,cntj++)
            tmp += A[i][j] * vec[cntj];
         mean[cnti] = tmp;
      }
      cnt += bsize;
   }
   /* Apply bias if required */
   bias = linXForm->bias;
   if (bias != NULL) {
      for (i=1;i<=size;i++)
         mean[i] += bias[i];
   }
   FreeVector(&gstack,vec);
}

static void ApplyXForm2TriMat(LinXForm *linXForm, TriMat t, Matrix m)
{  
   int size,b,bsize;
   Matrix A,mat;
   float tmp;
   int i,j,k;
   int cnt,cnti,cntj,cntk;

   /* Check dimensions */
   size = linXForm->vecSize;
   if (size != TriMatSize(t))
      HError(999,"Transform dimension (%d) does not match matrix dimension (%d)",
             size,TriMatSize(t));
   mat = CreateMatrix(&gstack,size,size);
   Tri2Mat(t,mat); 
   /* Transform mean */
   for (b=1,cnti=1,cnt=1;b<=IntVecSize(linXForm->blockSize);b++) {
      bsize = linXForm->blockSize[b];
      A = linXForm->xform[b];
      for (i=1;i<=bsize;i++,cnti++) {
         for (j=1,cntj=cnt;j<=bsize;j++,cntj++) {
            tmp=0;
            for (k=1,cntk=cnt;k<=bsize;k++,cntk++)
               tmp += A[i][k] * mat[cntk][cntj];
            m[cnti][cntj] = tmp;
         }
      }
      cnt += bsize;
   }
   if (linXForm->bias != NULL) HError(999,"Assumes there is no bias in transform");  
   FreeMatrix(&gstack,mat);
}

static void DiagApplyMat2TXForm(LinXForm *linXForm, Matrix m, Vector v)
{  
   int size,b,bsize;
   Matrix A;
   float tmp;
   int i,j;
   int cnt,cnti,cntj;

   /* Check dimensions */
   size = linXForm->vecSize;
   if (size != NumRows(m))
      HError(999,"Transform dimension (%d) does not match matrix dimension (%d)",
             size,NumRows(m));
   for (b=1,cnti=1,cnt=1;b<=IntVecSize(linXForm->blockSize);b++) {
      bsize = linXForm->blockSize[b];
      A = linXForm->xform[b];
      for (i=1;i<=bsize;i++,cnti++) {
         tmp=0;
         for (j=1,cntj=cnt;j<=bsize;j++,cntj++) {
            tmp +=  m[cnti][cntj] * A[i][j];
         }
         v[cnti] = tmp;
      }
      cnt += bsize;
   }
   if (linXForm->bias != NULL) HError(999,"Assumes there is no bias in transform");  
}

/* Feature-Space adaptation */
static Vector CompFXForm(MixPDF *mp, Vector svec, AdaptXForm *xform, AInfo *ai, LogFloat *det)
{
  Vector vec;
  XFormSet *xformSet;
  int numXf = 0;

  if (ai->next != NULL) { /* There's a parent transform */
    vec = CompFXForm(mp,svec,xform->parentXForm,ai->next,det);
  } else {
     *det = 0;
     vec = svec;
  }
  /* Check the kind of the adaptation */
  if ((xform->akind != BASE) && (xform->akind != TREE))
    HError(999,"Only BASE and TREE adaptation currently supported");
  if (HardAssign(xform))
    numXf = xform->xformWgts.assign[ai->baseClass];
  else 
    HError(999,"Not currently supported");
  /* Apply linear transformations to the parameters */
  if (numXf > 0) { /* Allows support when no transforms have been generated */
    xformSet = xform->xformSet;
    switch (xformSet->xkind) {
    case CMLLR: 
    case MLLRCOV:
    case SEMIT:
       ApplyXForm2Vector(xformSet->xforms[numXf],svec);
       *det += 0.5* (xformSet->xforms[numXf]->det);
       break;
    default:
      /* nothing is done */
      break;
    } /* No other options currently supported */
  } else {
    /* no transforms equates to an identity transform */
    svec = vec;
  }
  return svec;
}

/* Model space adaptation */
static void CompXForm(MixPDF *mp, AdaptXForm *xform, AInfo *ai)
{
  XFormSet *xformSet;
  int numXf, i;
  int size = VectorSize(mp->mean);
  Vector cov = CreateVector(&gstack,size);    

  if (ai->next != NULL) { /* There's a parent transform */
    CompXForm(mp,xform->parentXForm,ai->next);
  } else { /* set up model parameters for adptation */
    ResetComp(mp);
  }
  /* Check the kind of the adaptation */
  if ((xform->akind != BASE) && (xform->akind != TREE))
    HError(999,"Only BASE and TREE adaptation currently supported");
  numXf = xform->xformWgts.assign[ai->baseClass];
  /* Apply linear transformations to the parameters */
  if (numXf > 0) { /* Allows support when no transforms have been generated */
    xformSet = xform->xformSet;
    switch (xformSet->xkind) {
    case MLLRMEAN:
      ApplyXForm2Vector(xformSet->xforms[numXf],mp->mean);
      break;
    case MLLRCOV:
      ApplyXForm2Vector(xformSet->xforms[numXf],mp->mean);
      break;
    case MLLRVAR:    
      switch(mp->ckind){
      case DIAGC:
        ApplyXForm2Vector(xformSet->xforms[numXf], mp->cov.var);
        FixDiagGConst(mp);
        break;
      case INVDIAGC:    
        for (i=1;i<=size;i++)
          cov[i] = 1/mp->cov.var[i];
        ApplyXForm2Vector(xformSet->xforms[numXf], cov);
        for (i=1;i<=size;i++)
           mp->cov.var[i] = 1/cov[i];
        FixInvDiagGConst(mp);
        break;
      default:
        HError(999,"CompXForm: bad ckind %d",mp->ckind);
      }
      break;
    default:
      /* nothing is done */
      break;
    } /* No other options currently supported */
  } 
  FreeVector(&gstack, cov);
}

/*------------------------------------------------------------------------*/
/*                 Transform Initialisation and Estimation                */
/*------------------------------------------------------------------------*/

static LinXForm *CreateLinXForm(MemHeap *x,int vsize,IntVec blockSize)
{
   LinXForm *xf;
   int b,bsize,size;

   xf = (LinXForm *)New(x,sizeof(LinXForm));
   xf->det = 0;
   xf->nUse = 0;
   xf->vecSize = vsize;
   xf->blockSize = blockSize;    
   xf->vFloor = NULL;
   xf->xform = (SMatrix *)New(x,(IntVecSize(blockSize)+1)*sizeof(Matrix));
   size = 0;
   for (b=1;b<=IntVecSize(blockSize);b++) {
      bsize = blockSize[b];
      xf->xform[b] = CreateSMatrix(x,bsize,bsize);
      size += bsize;
   }
   if (size != vsize)
      HError(999,"Incompatible xform sizes %d and %d (block)",vsize,size);
   if (useBias) xf->bias = CreateSVector(x,size);
   else xf->bias = NULL;
   return xf;
}

static void EstMLLRDiagCovXForm(AccStruct *accs, LinXForm *xf, LinXForm *dxf)
{
   int b, cnti,dim, bsize;
   int i,j,k;  
   double tmu, tvar;
   DVector tvec;
   Matrix A;
   DMatrix G;
  
   for (b=1,cnti=1;b<=IntVecSize(accs->blockSize);b++) {
      if ((enableBlockAdapt == NULL) || (enableBlockAdapt[b] == 1)) {
         bsize = accs->blockSize[b];
         if (xf->bias == NULL) dim = bsize;
         else dim = bsize+1;
         A = xf->xform[b]; 
         tvec = CreateDVector(&gstack,dim);
         G = CreateDMatrix(&gstack,dim,dim);
         for (i=1;i<=bsize;i++,cnti++) {
            tmu = 0; tvar = 0;
            ZeroDVector(tvec);
            Tri2DMat(accs->G[cnti],G);
            for (j=1;j<=bsize;j++) {
               tmu += A[i][j] * accs->K[cnti][j];
               for (k=1;k<=bsize;k++)
                  tvec[j] += G[j][k] * A[i][k];
               if (xf->bias != NULL) 
                  tvec[j] += G[j][dim] * xf->bias[cnti];
            }
            if (xf->bias != NULL) {
               for (k=1;k<=bsize;k++)
                  tvec[dim] += G[dim][k] * A[i][k];
               tvec[dim] += xf->bias[cnti] * G[dim][dim];
            }
            for (j=1;j<=bsize;j++)
               tvar += A[i][j] * tvec[j];
            if (xf->bias != NULL) {
               tmu += xf->bias[cnti] * accs->K[cnti][dim];
               tvar += xf->bias[cnti] * tvec[dim];
            }
            dxf->xform[cnti][1][1] =  (accs->D[cnti] - 2*tmu + tvar)/accs->occ;
         }
         FreeDVector(&gstack,tvec);
      } else {
         bsize = accs->blockSize[b];         
         for (i=1;i<=bsize;i++,cnti++) {
            dxf->xform[cnti][1][1] = 1.0;
         }
      }
   }
}

static void EstMLLRMeanXForm(AccStruct *accs, LinXForm *xf)
{
   DMatrix invG,u,v;
   DVector w;
   SMatrix A;
   SVector bias;
   int i,j,k,dim;
   int cnti,b,bsize;
   Boolean uBias;

   bias = xf->bias; 
   if (bias==NULL) uBias = FALSE;
   else uBias = TRUE;
   for (b=1,cnti=1;b<=IntVecSize(accs->blockSize);b++) {
      if ((enableBlockAdapt == NULL) || (enableBlockAdapt[b] == 1)) {
         bsize = accs->blockSize[b];
         if (uBias) dim = bsize+1;
         else dim = bsize;
         /* set up the matrices for the inversion */
         invG = CreateDMatrix(&gstack,dim,dim);
         u = CreateDMatrix(&gstack, dim, dim);
         v = CreateDMatrix(&gstack, dim, dim);
         w = CreateDVector(&gstack, dim);
         /* and the transforms to be estimated */
         A = xf->xform[b]; 
         ZeroMatrix(A); 
         for (i=1;i<=bsize;i++,cnti++) {
            Tri2DMat(accs->G[cnti],invG);
            InvSVD(invG, u, w, v, invG);
            for (j=1;j<=bsize;j++)
               for (k=1;k<=dim;k++)
                  A[i][j] += invG[j][k] * accs->K[cnti][k];
            if (uBias) {
               bias[cnti]=0;
               for (k=1;k<=dim;k++)
                  bias[cnti] += invG[dim][k] * accs->K[cnti][k];
            }
         }
         FreeDMatrix(&gstack,invG);
      } else {
         bsize = accs->blockSize[b];         
         A = xf->xform[b]; 
         ZeroMatrix(A); 
         for (i=1;i<=bsize;i++,cnti++) {
            A[i][i] = 1.0;
            if (uBias) bias[cnti] = 0.0;
         }
      }
   }
}

static double GetAlphaLike(double a, double b, double c, double alpha)
{
  return (-c*log(fabs(alpha*a+b))-(alpha*alpha*a)/2);
}

static double GetAlpha(DMatrix invgmat,DVector kmat,double occ, DVector cofact)
{
  int bsize, dim, i ,j;
  DVector tvec;
  double a, b, c, tmp;
  double alpha1, alpha2, like1, like2;
 
  bsize= DVectorSize(cofact); 
  dim = DVectorSize(kmat);
  tvec = CreateDVector(&gstack,dim);
  ZeroDVector(tvec);
  for (i=1;i<=dim;i++)
    for (j=1;j<=bsize;j++)
      tvec[i] += cofact[j]*invgmat[i][j];
  /* Now set up the quadratic equation */
  a=0;b=0;c=-occ;
  for (i=1;i<=bsize;i++) {
    a += tvec[i]*cofact[i];
    b += tvec[i] * kmat[i];
  }
  if(bsize != dim)  b += tvec[dim] * kmat[dim];
  /* Must by definition be real */
  tmp = (b*b-4*a*c);
  if (tmp<0) {
    HError(-1,"WARNING: accumulates incorrect (%f < 0) - resetting",tmp);
    tmp=0;
  }
  
  tmp = sqrt(tmp);
  /* Now get the possible values of alpha */
  alpha1 = (-b+tmp)/(2*a);
  alpha2 = (-b-tmp)/(2*a);
  like1 = GetAlphaLike(a,b,c,alpha1);
  like2 = GetAlphaLike(a,b,c,alpha2);
 
  if (like2>like1)
    return alpha2;
  else
    return alpha1;
}

static double GetRowLike(DMatrix gmat,DVector kmat, DVector cofact, double occ, DVector w)
{
  double rowLike, det;
  int i, j, size,size2;
  DVector tvec;
  DMatrix tmat;
 
  size = DVectorSize(w);
  size2 = DVectorSize(cofact);
  tvec = CreateDVector(&gstack,size);
  tmat = CreateDMatrix(&gstack,size,size);
  Tri2DMat(gmat,tmat);
  ZeroDVector(tvec);
  for (i=1;i<=size;i++)
    for (j=1;j<=size;j++)
      tvec[i] += w[j]*tmat[i][j];
  rowLike = 0;
  for (i=1;i<=size;i++)
    rowLike += (tvec[i] - 2*kmat[i])*w[i];
  det=0;
  for (i=1;i<=size2;i++)
    det += cofact[i]*w[i];
  rowLike = log(fabs(det))*occ - rowLike/2;
  FreeDVector(&gstack,tvec);
  return rowLike;
}

static double GetSemiTiedLike(AccStruct *accs, LinXForm *xf)
{
   double loglike,tmp;
   int bsize,b,k,kk,cnt,nblocks,i;
   DMatrix G;
   Matrix A;
   Vector w;

   /* this is the complete dimensionality of the xform */
   loglike = 0;
   if (staticSemiTied) nblocks=1;
   else nblocks = IntVecSize(accs->blockSize);
   if (xf == NULL) {
      /* implies an identity XForm */
      for (b=1,cnt=1;b<=nblocks;b++) {
         bsize = accs->blockSize[b];
         for (k=1;k<=bsize;k++,cnt++)
            loglike += accs->G[cnt][k][k];
      }      
      loglike = -loglike/2;
   } else {
      for (b=1,cnt=1;b<=nblocks;b++) {
         bsize = accs->blockSize[b];
         A = xf->xform[b];
         for (i=1;i<=bsize;i++,cnt++) {
            w = A[i]; 
            G = accs->G[cnt];
            for (k=1; k<= bsize; k++) {
               tmp = 0;
               for (kk=1; kk<=bsize; kk++)
                  tmp += w[kk] * G [k][kk];
               loglike += w [k] * tmp;
            }
         }
      }
      if (staticSemiTied) 
         loglike = (xf->det*accs->occ/IntVecSize(accs->blockSize) - loglike)/2;
      else 
         loglike = (xf->det*accs->occ - loglike)/2;
   }
   return loglike;
}

static void InitCMLLRXForm(AccStruct *accs, DVector W, DVector bias)
{
  DMatrix invG,u,v,lG;
  int i,k,dim,ldim;
  int cnt, cnti,b,bsize;
  Boolean uBias;
  double alpha, likeNew, likeOld;
  DVector tvec,tW,w,iW,lK;
  DVector cofact;
  
  if (bias==NULL) uBias = FALSE;
  else uBias = TRUE;
  cofact = CreateDVector(&gstack,1);
  if (uBias) ldim = 2;
  else ldim = 1;
  /* set up the matrices for the inversion */
  lG = CreateDMatrix(&gstack,ldim,ldim);
  invG = CreateDMatrix(&gstack,ldim,ldim);
  u = CreateDMatrix(&gstack, ldim, ldim);
  v = CreateDMatrix(&gstack, ldim, ldim);
  w = CreateDVector(&gstack, ldim);
  tW = CreateDVector(&gstack, ldim);
  tvec = CreateDVector(&gstack, ldim);
  lK = CreateDVector(&gstack, ldim);
  iW = CreateDVector(&gstack, ldim);
  /* identity xform for log-likelihood check */
  iW[1]=1; iW[2]=0;
  for (b=1,cnt=1;b<=IntVecSize(accs->blockSize);b++) {
    bsize = accs->blockSize[b];
    if (uBias) dim = bsize+1;
    else dim = bsize;
    /* and the transforms to be estimated */
    for (i=1,cnti=cnt;i<=bsize;i++,cnti++) {
      /* Copy appropriate elements from the accumlates */
      if (uBias) {
	lG[1][1] = accs->G[cnti][i][i]; lG[1][2] = accs->G[cnti][dim][i];
	lG[2][1] = lG[1][2]; lG[2][2] = accs->G[cnti][dim][dim];
	lK[1] = accs->K[cnti][i]; lK[2] = accs->K[cnti][dim];
      } else {
	lG[1][1] = accs->G[cnti][i][i];
	lK[1] = accs->K[cnti][i];
      }
      /* For diag case the cofactors are independent */
      cofact[1]=1;
      InvSVD(lG, u, w, v, invG);
      alpha = GetAlpha(invG,lK,accs->occ,cofact);
      tvec[1] = alpha * cofact[1] + lK[1];
      if (uBias) tvec[2] = lK[2];
      ZeroDVector(tW);
      for (k=1;k<=ldim;k++)
	tW[1] += invG[1][k] * tvec[k];
      if (uBias) {
	tW[ldim]=0;
	for (k=1;k<=ldim;k++)
	  tW[ldim] += invG[ldim][k] * tvec[k];
      }
      likeNew = GetRowLike(lG,lK,cofact,accs->occ,tW);
      /* compare to identity transform */
      likeOld = GetRowLike(lG,lK,cofact,accs->occ,iW);
      if (likeNew<likeOld) {
	if (likeOld/likeNew>1.00001) /* put a threshold on this! */
	  printf(" Issue in intialising row %d of block %d (%f->%f)\n",
		 i,b,likeNew/accs->occ,likeOld/accs->occ);
	W[cnti] = iW[1];
	if (uBias) bias[cnti] = iW[2];
      } else {
	W[cnti] = tW[1];
	if (uBias) bias[cnti] = tW[2];
      }
    }     
    cnt += bsize;
  }
  FreeDVector(&gstack,cofact);
}

static void EstCMLLRXForm(AccStruct *accs, LinXForm *xf)
{
  DMatrix *InvG,invG,u,v;
  DVector w;
  DMatrix A;
  DVector bias;
  int i,j,k,dim;
  int iter;
  int cnt, cnti,b,bsize;
  Boolean uBias;
  double alpha, likeNew, likeOld;
  double det=0.0,tdet;
  DVector W, iniW, tvec, iniA;
  DVector cofact;
  
  iniA = CreateDVector(&gstack, xf->vecSize);
  if (xf->bias == NULL) {
     uBias = FALSE;
     bias = NULL;
  } else {
     uBias = TRUE;
     bias = CreateDVector(&gstack,xf->vecSize);
  } 

  InitCMLLRXForm(accs, iniA , bias);
  InvG = (DMatrix *)New(&gstack,sizeof(DMatrix)*(accs->dim+1)); 
  tdet = 0;
  
  for (b=1,cnt=1;b<=IntVecSize(accs->blockSize);b++) {
    bsize = accs->blockSize[b];
    cofact = CreateDVector(&gstack,bsize);
    if (uBias) dim = bsize+1;
    else dim = bsize;
    /* set up the matrices for the inversion */
    u = CreateDMatrix(&gstack, dim, dim);
    v = CreateDMatrix(&gstack, dim, dim);
    w = CreateDVector(&gstack, dim);
    /* and the transforms to be estimated */
    A = CreateDMatrix(&gstack, bsize,bsize);
    W = CreateDVector(&gstack,dim);
    iniW = CreateDVector(&gstack,dim);
    tvec = CreateDVector(&gstack,dim);
    ZeroDMatrix(A); 
    for (i=1,cnti=cnt;i<=bsize;i++,cnti++) {
      A[i][i] = iniA[cnti];   
      InvG[cnti] = CreateDMatrix(&gstack,dim,dim);
      Tri2DMat(accs->G[cnti],InvG[cnti]);
      InvSVD(InvG[cnti], u, w, v, InvG[cnti]);
    }
    for (iter=1;iter<=maxXFormIter;iter++) {
      ZeroDVector(iniW);
      for (i=1,cnti=cnt;i<=bsize;i++,cnti++) {
        for (j=1;j<=bsize;j++)      iniW[j] = A[i][j];
        if (uBias)  iniW[dim] = bias[cnti];
        det = DMatCofact(A,i,cofact);        
	invG = InvG[cnti];    
        alpha = GetAlpha(invG,accs->K[cnti],accs->occ,cofact);
        ZeroDVector(W);
        for (j=1;j<=bsize;j++)
          tvec[j] = alpha * cofact[j] + accs->K[cnti][j];
        if (uBias)  tvec[dim] = accs->K[cnti][dim];
        for (j=1;j<=bsize;j++)
          for (k=1;k<=dim;k++)
            W[j] += invG[j][k] * tvec[k];
        if (uBias) {
          W[dim]=0;
          for (k=1;k<=dim;k++)
            W[dim] += invG[dim][k] * tvec[k];
        }      
        likeNew = GetRowLike(accs->G[cnti],accs->K[cnti],cofact,accs->occ,W);
        likeOld = GetRowLike(accs->G[cnti],accs->K[cnti],cofact,accs->occ,iniW);
        if (likeNew>likeOld) {
           det = 0; 
           for (j=1;j<=bsize;j++) {
              A[i][j] = W[j];
              det += cofact[j]*W[j];
           }
           if (uBias) {
              bias[cnti] = 0;
              bias[cnti] += W[dim];
           }
	 } else {
            if (likeOld/likeNew>1.00001) /* put a threshold on this! */
	      printf("  Not updating transform (Block: %d Row: %d Iter: %d (%f %f))\n",
		     b,i,iter,likeNew/accs->occ,likeOld/accs->occ);
	 }
      }
    }
    cnt += bsize;
    tdet += log(fabs(det));
    /* Copy the transform into single precision for storage */
    for (i=1;i<=bsize;i++)
       for (j=1;j<=bsize;j++)
          xf->xform[b][i][j] = A[i][j];
    FreeDVector(&gstack,cofact);
  }
  /* Copy the bias transform and determinant (stored single precision) */
  if (uBias) {
     for (i=1;i<=xf->vecSize;i++) xf->bias[i] = bias[i];
  }
  xf->det = tdet*2;
  FreeDVector(&gstack, iniA);
}

static void AccMixPDFSemiTiedStats(HMMSet *hset,MixPDF *mp, AccStruct *accs)
{
   VaAcc *va;
   int b,k,kk,bsize,s,strm;
   int i,cnti,bstart;
   DMatrix G;
   float ivar;
   STriMat inv;

   va = (VaAcc *) GetHook(mp->cov.var);
   if (va->occ>0) {
      strm = mp->stream;
      inv = va->cov.inv;
      if (strmProj) {
         /* 
            Only accumulate observations for the first stream.
            Otherwise the occupancy will be too large - the 
            sum for each stream of the state must be the same
         */
         if (strm == 1) accs->occ += va->occ;
         /* First extract the correct accumulate to add */
         for (s=1,cnti=1;s<strm;s++) cnti += hset->swidth[s];
         for (i=1;i<=hset->swidth[strm];i++,cnti++) {
            if (mp->ckind == INVDIAGC) ivar=mp->cov.var[i];
            else ivar = 1/mp->cov.var[i];
            G = accs->G[cnti]; 
            for (k=1;k<=hset->vecSize;k++) 
               for (kk=1;kk<=k;kk++) 
                  G[k][kk] += ivar*inv[k][kk];
         }
      } else {
         /* the G accumulates may have been tied ... */
         if (staticSemiTied) accs->occ += va->occ * IntVecSize(accs->blockSize);
         else accs->occ += va->occ;
         for (b=1,cnti=1,bstart=0;b<=IntVecSize(accs->blockSize);b++) {
            bsize = accs->blockSize[b]; 
            /* 
               numNuisanceDim can ONLY be non-zero for single block
               systems.
            */
            for (i=1;i<=bsize-numNuisanceDim;i++,cnti++) {
               if (mp->ckind == INVDIAGC) ivar=mp->cov.var[cnti];
               else ivar = 1/mp->cov.var[cnti];
               G = accs->G[cnti]; 
               for (k=1;k<=bsize;k++) 
                  for (kk=1;kk<=k;kk++) 
                     G[k][kk] += ivar*inv[bstart+k][bstart+kk];
            }
            bstart += bsize;
         }
      }
   }
}

/* 
   returns -1 if the stream associated with the transform is not consistent
   and the list of all streams used. Wasteful ...
*/
static int GetXFormStream(AdaptXForm *xform, IntVec classes, IntVec streams)
{
   BaseClass *bclass;
   int b,stream=0;
   ILink il;
   MixPDF *mp = NULL;

   bclass = xform->bclass;
   ZeroIntVec(streams);
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (il=bclass->ilist[b]; il!=NULL; il=il->next) {
            mp = ((MixtureElem *)il->item)->mpdf;
            streams[mp->stream] = 1;
            if (stream == 0) stream = mp->stream;
            else if (stream != mp->stream) stream = -1;
         }
      }
   }
   return stream;
}

/* extract the dimensions with the greatest between to within ratio (same as greatest total 
   to within given occupancy weighted counts. modify the transform so that this change is reflected
   in ixf
*/
static void InitSemiTiedFR(AdaptXForm *xform, LinXForm *xf, IntVec classes, TriMat totCov)
{
   BaseClass *bclass;
   DVector withinCov, fisherRatio;
   double occ;
   int size,i,j,k,b,max;
   double vmax;
   Matrix A,inv;
   ILink il;
   MixPDF *mp = NULL;
   VaAcc *va;

   size = xf->vecSize;
   withinCov = CreateDVector(&gstack,size);
   fisherRatio = CreateDVector(&gstack,size);
   ZeroDVector(withinCov); occ = 0;
   bclass = xform->bclass;
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (il=bclass->ilist[b]; il!=NULL; il=il->next) {
            mp = ((MixtureElem *)il->item)->mpdf;
            va = (VaAcc *)GetHook(mp->cov.var);
            if (va->occ > 0.0) {
               occ += va->occ;
               inv = va->cov.inv;
               for (i=1;i<=size;i++)
                  withinCov[i] += inv[i][i];
            }
         }
      }
   }
   /* map withinCov to hold the ratio */
   for (i=1;i<=size;i++) fisherRatio[i] = totCov[i][i]*(occ/withinCov[i]) -1;
   /* perform simple select process - only single block for HLDA */
   A = xf->xform[1];
   ZeroMatrix(A);
   /* naive search, swap initialise as bottom dimensions projected out */
   if (trace & T_FRS) ShowDVector("Initial",fisherRatio,size);
   for (i=1;i<=size;i++) {
      max = 0; vmax = -1;
      for (j=1;j<=size;j++) {
         if (fisherRatio[j]>vmax) {
            max=j;
            vmax=fisherRatio[j];
         }
      }
      /* can either order by ranking .. or ident */
      if (!initNuisanceFRIdent) A[i][max] = 1;
      fisherRatio[max] = -i;
   }
   for (i=1;i<=size;i++) fisherRatio[i] = -fisherRatio[i];
   if (initNuisanceFRIdent) {
      for (i=1,j=1,k=size-numNuisanceDim+1;i<=size;i++) {
         if (fisherRatio[i] <= size-numNuisanceDim) {
            A[j][i] = 1;
            j++;
         } else {
            A[k][i] = 1;
            k++;
         }
      }
   }
   if (trace & T_FRS) {
      ShowDVector("Selection Order",fisherRatio,size);
      /* ShowMatrix("Initial",A,size,size); */
   }
   xf->det=0;
   FreeDVector(&gstack,withinCov);
}


static void UpdateSemiTiedAccs(AdaptXForm *xform, AccStruct *accs, IntVec classes, TriMat totCov)
{
   BaseClass *bclass;
   int b,i,k,kk,bsize,vsize;
   int cnti;
   ILink il;
   MixPDF *mp = NULL;
   DMatrix G;
   float ivar;

   /* zero the accumulates */
   accs->occ = 0;
   if (strmProj) vsize = xform->hset->vecSize;
   else vsize = accs->dim;
   for (i=1;i<=vsize;i++) ZeroDMatrix(accs->G[i]);
   bclass = xform->bclass;
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (il=bclass->ilist[b]; il!=NULL; il=il->next) {
            mp = ((MixtureElem *)il->item)->mpdf;
            AccMixPDFSemiTiedStats(xform->hset,mp,accs);
         }
      }
   }
   /* make the accumulates symmetric (saves thinking later) */
   if (strmProj) {
      for (i=1;i<=vsize;i++) {
         G = accs->G[i]; 
         for (k=1;k<=vsize;k++) 
            for (kk=1;kk<k;kk++) 
               G[kk][k] = G[k][kk];
      }
   } else {
      /* handle the nuisance dimension attributes */
      for (i=vsize-numNuisanceDim+1;i<=vsize;i++) {
         /* mp is the last component of this baseclass */
         if (mp->ckind == INVDIAGC) ivar=mp->cov.var[i];
         else ivar = 1/mp->cov.var[i];
         /* scale the inverse to reflect counts */
         ivar *= accs->occ;
         G = accs->G[i];
         for (k=1;k<=vsize;k++) 
            for (kk=1;kk<=k;kk++) 
               G[k][kk] += ivar*totCov[k][kk];
      }
      for (b=1,cnti=1;b<=IntVecSize(accs->blockSize);b++) {
         bsize = accs->blockSize[b]; 
         for (i=1;i<=bsize;i++,cnti++) {
            G = accs->G[cnti]; 
            for (k=1;k<=bsize;k++) 
               for (kk=1;kk<k;kk++) 
                  G[kk][k] = G[k][kk];
         }
      }
   }
}

static void InitSemiTiedVars(AdaptXForm *xform, IntVec classes, TriMat totCov)
{
   int b,j,size;
   ILink i;
   MixPDF *mp = NULL;
   Vector tcov;
   HMMSet *hset;
   BaseClass *bclass;

   hset = xform->hset;
   bclass = xform->bclass;
   size = hset->vecSize; 
   tcov = CreateVector(&gstack,size);
   for (j=1;j<=size;j++)
      tcov[j] = totCov[j][j];
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
            mp = ((MixtureElem *)i->item)->mpdf;
            for (j=size-numNuisanceDim+1;j<=size;j++) {
               /* no flooring necessary ... */
               mp->cov.var[j] = tcov[j];
            }
            if (mp->ckind == INVDIAGC) 
               for (j=size-numNuisanceDim+1;j<=size;j++)
                  mp->cov.var[j] = 1/mp->cov.var[j];
         }
      }
   }
   FreeVector(&gstack,tcov);
}

/*
  Updates the variance and returns the occupancy weighted summed log determinant
  for use in the likelihood calculation.
*/
static double UpdateSemiTiedVars(HMMSet *hset, LinXForm *xf, BaseClass *bclass, IntVec classes, Vector vFloor, TriMat totCov)
{
   int b,j,size,strm,s,cnt;
   int nFloor;
   ILink i;
   MixPDF *mp = NULL;
   Matrix mat;
   VaAcc *va;
   STriMat inv;
   Vector vec,tcov=NULL;
   double logdet;

   size = xf->vecSize; nFloor = 0; logdet=0;
   mat = CreateMatrix(&gstack,size,size);
   if (strmProj) vec = CreateVector(&gstack,size);
   else vec = NULL;
   if (numNuisanceDim > 0) {
      tcov = CreateVector(&gstack,size);
      ApplyXForm2TriMat(xf,totCov,mat);
      DiagApplyMat2TXForm(xf,mat,tcov);
   }
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
            mp = ((MixtureElem *)i->item)->mpdf;
            va = (VaAcc *) GetHook(mp->cov.var);
            if (va->occ>0) {
               inv = va->cov.inv;
               ApplyXForm2TriMat(xf,inv,mat);
               if (strmProj) {
                  strm = mp->stream;
                  /* This is very wasteful - possibly fix later */
                  DiagApplyMat2TXForm(xf,mat,vec);
                  for (s=1,cnt=1;s<strm;s++)  cnt += hset->swidth[s];
                  for (j=1;j<=hset->swidth[strm];j++,cnt++) {
                     mp->cov.var[j] = vec[cnt]/va->occ;
                     if (mp->cov.var[j]<vFloor[cnt])
                        mp->cov.var[j] = vFloor[cnt];
                  }
                  if (mp->ckind == INVDIAGC) 
                     for (j=1;j<=hset->swidth[strm];j++)
                        mp->cov.var[j] = 1/mp->cov.var[j];
                  if (mp->ckind == INVDIAGC) {
                     for (j=1;j<=hset->swidth[strm];j++)
                        logdet += va->occ * log(mp->cov.var[j]);
                  } else {
                     for (j=1;j<=hset->swidth[strm];j++)
                        logdet -= va->occ * log(mp->cov.var[j]);
               }
               } else { 
                  DiagApplyMat2TXForm(xf,mat,mp->cov.var);
                  /* apply variance floor and normalise */
                  for (j=1;j<=size-numNuisanceDim;j++) {
                     mp->cov.var[j] /= va->occ;
                     if (mp->cov.var[j]<vFloor[j]) {
                        mp->cov.var[j] = vFloor[j];
                        nFloor++;
                     }
                  }
                  if (numNuisanceDim > 0) {
                     for (j=size-numNuisanceDim+1;j<=size;j++) {
                        /* no flooring necessary ... */
                        mp->cov.var[j] = tcov[j];
                     }
                  }
                  if (mp->ckind == INVDIAGC) 
                     for (j=1;j<=size;j++)
                        mp->cov.var[j] = 1/mp->cov.var[j];
                  /* get the information for the likelihood calculation */
                  if (mp->ckind == INVDIAGC) {
                     for (j=1;j<=size;j++)
                        logdet += va->occ * log(mp->cov.var[j]);
                  } else {
                     for (j=1;j<=size;j++)
                        logdet -= va->occ * log(mp->cov.var[j]);
                        }

               }
            }
         }
      }
   }
   if (nFloor>0)
      printf("  Floored %d component elements\n",nFloor);
   FreeMatrix(&gstack,mat);
   return logdet/2;
}

static double InitSemiTiedDet(BaseClass *bclass, IntVec classes)
{
   int b,j,size;
   ILink i;
   MixPDF *mp = NULL;
   VaAcc *va;
   double logdet;

   logdet = 0;
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
            mp = ((MixtureElem *)i->item)->mpdf;
            va = (VaAcc *)GetHook(mp->cov.var);
            if (va->occ > 0) {
               size = VectorSize(mp->mean);
               if (mp->ckind == INVDIAGC) {
                  for (j=1;j<=size;j++)
                     logdet += va->occ * log(mp->cov.var[j]);
               } else {
                  for (j=1;j<=size;j++)
                     logdet -= va->occ * log(mp->cov.var[j]);
               }
            }
         }
      }
   }
   return logdet/2;
}


static void UpdateSemiTiedMeans(HMMSet *hset, LinXForm *xf, BaseClass *bclass, IntVec classes, Vector totMean)
{
   int b,j,size,s,strm,cnt;
   ILink i;
   MixPDF *mp = NULL;
   Vector vec,tvec;
   MuAcc *ma;

   size = xf->vecSize; 
   vec = CreateVector(&gstack,size);
   if (numNuisanceDim > 0) {
      tvec = CreateVector(&gstack,size);
      CopyVector(totMean,tvec);
      ApplyXForm2Vector(xf,tvec);
   } else {
      tvec = NULL;
   }
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
            mp = ((MixtureElem *)i->item)->mpdf;
            ma = (MuAcc *) GetHook(mp->mean);
            if (ma->occ>0) {
               CopyVector(ma->mu,vec);
               ApplyXForm2Vector(xf,vec);
               if (strmProj) {
                  strm = mp->stream;
                  for (s=1,cnt=1;s<strm;s++)  cnt += hset->swidth[s];
                  for (j=1;j<=hset->swidth[strm];j++,cnt++) 
                     mp->mean[j] = vec[cnt]/ma->occ;
               } else {
                  for (j=1;j<=size-numNuisanceDim;j++)
                     mp->mean[j] = vec[j]/ma->occ;
               }
               /* update the global "tied" means */
               if (numNuisanceDim > 0) {
                  for (j=size-numNuisanceDim+1;j<=size;j++)
                     mp->mean[j] = tvec[j];
               }
            }
         }
      }
   }
   FreeVector(&gstack,vec);
}

static void GetSemiTiedTotCov(AdaptXForm *xform, IntVec classes, TriMat totCov, Vector totMean)
{
   int b,k,kk;
   int size,cnt;
   ILink i;
   MixPDF *mp=NULL;
   DMatrix mat=NULL, tmat=NULL;
   DVector tvec=NULL;
   VaAcc *va;
   MuAcc *ma;
   double totOcc;
   STriMat inv;
   Vector mu;
   BaseClass *bclass;
   float tmp;

   cnt = 0;
   bclass = xform->bclass;
   size = 0; mat = NULL; totOcc = 0;
   for (b=1;b<=bclass->numClasses;b++) {
      if (classes[b] == 1) {
         for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
            cnt++;
            mp = ((MixtureElem *)i->item)->mpdf;
            if (mat == NULL) { /* need to get size for this transform */
               if (strmProj) size = xform->hset->vecSize; 
               else size = VectorSize(mp->mean);
               mat = CreateDMatrix(&gstack,size,size);
               tmat = CreateDMatrix(&gstack,size,size);
               tvec = CreateDVector(&gstack,size);
               totOcc = 0; ZeroDMatrix(mat);
               ZeroDMatrix(tmat); ZeroDVector(tvec);
            }
            va = (VaAcc *) GetHook(mp->cov.var);
            totOcc += va->occ;
            inv = va->cov.inv;
            if (va->occ>0) {
               ma = (MuAcc *) GetHook(mp->mean);
               mu = ma->mu;
               for (k=1;k<=size;k++) {
                  tmp = mu[k];
                  tvec[k] += tmp;
                  for (kk=1;kk<=k;kk++) 
                     tmat[k][kk] += inv[k][kk] + tmp*mu[kk]/va->occ;
               }
            }
         }
      }
   }
   for (k=1;k<=size;k++) {
      totMean[k] = tvec[k]/totOcc;
      for (kk=1;kk<=k;kk++) 
         totCov[k][kk] = tmat[k][kk]/totOcc - totMean[k]*totMean[kk];   
   }
   FreeDMatrix(&gstack,mat);
}

static void UpdateSemiTiedVFloor(HMMSet *hset, LinXForm *xf, TriMat avCov, SVector vFloor)
{
   Matrix mat;
   int size,i;

   if (semiTiedVFloor) {
      size = xf->vecSize;
      mat = CreateMatrix(&gstack,size,size);
      ApplyXForm2TriMat(xf,avCov,mat);
      DiagApplyMat2TXForm(xf,mat,vFloor);
      for (i=1;i<=size;i++) vFloor[i] *= semiTiedFloorScale;
      FreeMatrix (&gstack, mat);
   } else
      ZeroVector(vFloor);
}

void UpdateHLDAModel(HMMSet *hset)
{
   HMMScanState hss;
   MixPDF *mp;
   SVector mu;
   SVector var;
   int size, hldasize, i;

   size = hset->vecSize;
   /* support HHEd projection work */
   if (hset->projSize > 0) hldasize = size-hset->projSize;
   else hldasize = size-numNuisanceDim;
   NewHMMScan(hset,&hss);
   do {
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {            
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  mp = hss.mp;
                  mu = CreateSVector(hset->hmem,hldasize);
                  var = CreateSVector(hset->hmem,hldasize);
                  for (i=1;i<=hldasize;i++) {
                     mu[i] = mp->mean[i];
                     var[i] = mp->cov.var[i];
                  }
                  /* set the hooks correctly so no by-products */
                  SetHook(mu,GetHook(mp->mean));
                  SetHook(var,GetHook(mp->cov.var));
                  mp->mean = mu;
                  mp->cov.var = var;
               }
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   /* now tidy the global attributes */
   hset->vecSize = hldasize;
   hset->swidth[1] = hldasize;
   hset->projSize = 0;
}


static void UpdateSemiTiedXForm(AccStruct *accs, LinXForm *xf, LinXForm *ixf)
{
   DMatrix invG,u,v;
   DMatrix *InvG;
   DVector w;
   DMatrix A;
   int i,j,k,dim;
   int iter;
   int cnt, cnti,b,bsize;
   double det=0.0,tdet=0.0;
   double beta, likeNew, likeOld;
   DVector W, iniW;
   DVector cofact;
  
   /* Reset xform  to identity matrix */
  
   InvG = (DMatrix *)New(&gstack,sizeof(DMatrix)*(accs->dim+1)); 
   for (b=1,cnt=1;b<=IntVecSize(accs->blockSize);b++) {
     bsize = accs->blockSize[b];
     dim = bsize;
     cofact = CreateDVector(&gstack,bsize);
     ZeroDVector(cofact); 
     /* set up the matrices for the inversion */
     u = CreateDMatrix(&gstack, dim, dim);
     v = CreateDMatrix(&gstack, dim, dim);
     w = CreateDVector(&gstack, dim);
     /* and the transforms to be estimated */
     A = CreateDMatrix(&gstack,bsize,bsize);
     ZeroDMatrix(A); 
     if (ixf == NULL) {
        /* initialise with the diagonal transform */
        for (i=1,cnti=cnt;i<=bsize;i++, cnti++) {
           A[i][i] = sqrt(accs->G[cnti][i][i]/accs->occ); 
           A[i][i] = 1/A[i][i];
           InvG[cnti] = CreateDMatrix(&gstack,dim,dim);
           Tri2DMat(accs->G[cnti],InvG[cnti]);
           InvSVD(InvG[cnti], u, w, v, InvG[cnti]);
        }
     } else {
        for (i=1,cnti=cnt;i<=bsize;i++, cnti++) {
           InvG[cnti] = CreateDMatrix(&gstack,dim,dim);
           Tri2DMat(accs->G[cnti],InvG[cnti]);
           InvSVD(InvG[cnti], u, w, v, InvG[cnti]);
           for (j=1;j<=bsize;j++)
              A[i][j] = ixf->xform[b][i][j];
        }
     }
     W = CreateDVector(&gstack,dim);
     iniW = CreateDVector(&gstack,dim);
     for (iter=1;iter<=maxXFormIter;iter++) {
       ZeroDVector(iniW);
       for (i=1,cnti=cnt;i<=bsize;i++,cnti++) {
         for (j=1;j<=bsize;j++)      iniW[j] = A[i][j];
         invG = InvG[cnti];    
         det = DMatCofact(A,i,cofact);     
         beta = 0;
         for(j=1;j<=bsize;j++){
            for(k=1;k<=bsize;k++)
               beta += cofact[j]*invG[j][k]*cofact[k];
         }
         beta = sqrt(accs->occ/beta);
         ZeroDVector(W);
         for(j=1;j<=bsize;j++){
            for(k=1;k<=bsize;k++)
               W[j] += cofact[k]*invG[j][k];
            W[j] *= beta;          
         }
         ZeroDVector(w);
         likeNew = GetRowLike(accs->G[cnti],w,cofact,accs->occ,W);
         likeOld = GetRowLike(accs->G[cnti],w,cofact,accs->occ,iniW);
         /* printf("Iteration %d (row %d): ",iter,cnt); */
         if (likeNew>likeOld) {
           for(j=1;j<=bsize;j++)  A[i][j] = W[j];
         } else {
            if (likeOld/likeNew>1.00001) /* put a threshold on this! */
               printf("  Not updating transform (Block: %d Row: %d Iter: %d (%f %f))\n",
		      b,i,iter,likeNew/accs->occ,likeOld/accs->occ);
	 }
       }
     }
     cnt += bsize;
     tdet += log(fabs(det));
     /* Copy the transform into single precision for storage */
     for (i=1;i<=bsize;i++)
        for (j=1;j<=bsize;j++)
           xf->xform[b][i][j] = A[i][j];
     FreeDVector(&gstack, cofact);
   }
   xf->det = tdet*2;
   Dispose(&gstack,InvG);
}

static void CheckSemiTiedOptions(AdaptXForm *xform, AccStruct *accs)
{
   int b,bsize;

   if (strmProj) { /* confirm that strmProj valid */
      if (xform->bclass->numClasses != 1)
         HError(999,"Can only have strmProj with global transform");
      if (xform->hset->swidth[0] == 1) 
         HError(999,"strmProj with a single stream is not valid");
   } else if (numNuisanceDim>0) { /* check the nuisance dimension options */
      if (xform->hset->swidth[0] != 1)
         HError(999,"Can only have Nuisance Dimensions with single streams");
   }

   if (semiTied2InputXForm) {
      if (xform->bclass->numClasses != 1)
         HError(999,"Can only store as input XForm with global transform");
      if (xform->hset->swidth[0] != 1) 
         HError(999,"Can only store as input XForm with single stream");
      if (xform->hset->xf != NULL)
         HError(999,"Can not store as input XForm if HMMSet already has an input XForm");
      if (xform->hset->semiTied != NULL)
         HError(999,"Can not store as input XForm if HMMSet already has an semitied XForm");
      if (IntVecSize(accs->blockSize) > 1)
         HError(999,"Can not store as input XForm if full XForm used");
   }   
   if (numNuisanceDim > 0) {
      if (IntVecSize(accs->blockSize) != 1)
         HError(999,"Can only use HLDA projections with a single block");
   }
   if (staticSemiTied) {
      bsize = accs->blockSize[1];
      for (b=2;b<=IntVecSize(accs->blockSize);b++)
         if (accs->blockSize[b] != bsize)
            HError(999,"Inconsistent block size [1 %d] and [%d %d]",bsize,b,accs->blockSize[b]);
   }
}

static void EstSemiTXForm(AdaptXForm *xform, AccStruct *accs, LinXForm *xf, IntVec classes)
{
   int iter,size=0,stream;
   TriMat totCov;
   Vector totMean;
   SVector vFloor;
   HMMSet *hset;
   IntVec streams;
   double logdet,tocc;
   LinXForm *ixf;

   hset = xform->hset; 
   CheckSemiTiedOptions(xform,accs);
   if (strmProj) { 
      size =  hset->vecSize;
   } else 
      size = accs->dim;
   
   ixf = NULL;
   vFloor = CreateSVector(xform->mem,size);
   streams = CreateIntVec(&gstack,hset->swidth[0]);
   totCov = CreateTriMat(&gstack,size);
   totMean = CreateVector(&gstack,size);
   if (strmProj) stream = 1;
   else stream = GetXFormStream(xform,classes,streams);
   /* handle initialisation for HLDA */
   /* get the global covariance matrix */
   GetSemiTiedTotCov(xform,classes,totCov,totMean);
   if (numNuisanceDim > 0) {
      if (initNuisanceFR) {
         /* compute Fisher ratios to initialise nuisance dimensions */
         InitSemiTiedFR(xform,xf,classes,totCov);
         ixf = xf;
         if (stream>0) { 
            /* 
               if there are multiple baseclases and a semitied transform already being used
               swap to a fraction of the global covariance matrix and warn the user
            */               
            if ((xform->bclass->numClasses>1) && (hset->semiTied != NULL)) {
               printf("WARNINING: using total baseclass variance, not average state variance\n");
               UpdateSemiTiedVFloor(hset,xf,totCov,vFloor);
            } else
               UpdateSemiTiedVFloor(hset,xf,avCov[stream],vFloor);
         }
         else 
            ZeroVector(vFloor);
         logdet = UpdateSemiTiedVars(hset,xf,xform->bclass,classes,vFloor,totCov);         
      } else {
         InitSemiTiedVars(xform,classes,totCov);
         logdet = InitSemiTiedDet(xform->bclass,classes);
      }
   } else 
      logdet = InitSemiTiedDet(xform->bclass,classes);
   for (iter=1;iter<=maxSemiTiedIter;iter++) {
      UpdateSemiTiedAccs(xform,accs,classes,totCov);
      if (staticSemiTied) tocc = accs->occ/(IntVecSize(accs->blockSize));
      else tocc = accs->occ;
      if (iter == 1) {
         logdet += GetSemiTiedLike(accs,ixf);
         printf("Initial LogLikelihood = %f\n",logdet/tocc);
         UpdateSemiTiedXForm(accs, xf, ixf);
      }
      else UpdateSemiTiedXForm(accs, xf, xf);
      printf("Iteration %d\n",iter); fflush(stdout);
      if (stream>0) {
         /* 
            if there are multiple baseclases and a semitied transform already being used
            swap to a fraction of the global covariance matrix and warn the user
         */               
         if ((xform->bclass->numClasses>1) && (hset->semiTied != NULL)) {
            printf("WARNINING: using total baseclass variance, not average state variance\n");
            UpdateSemiTiedVFloor(hset,xf,totCov,vFloor);
         } else
            UpdateSemiTiedVFloor(hset,xf,avCov[stream],vFloor);
      } else 
         ZeroVector(vFloor);
      logdet = UpdateSemiTiedVars(hset,xf,xform->bclass,classes,vFloor,totCov);
      logdet += GetSemiTiedLike(accs,xf);
      printf("  LogLikelihood = %f\n",logdet/tocc);
   }
   /* Tidy Stats */
   xf->vFloor = vFloor;
   UpdateSemiTiedMeans(hset,xf,xform->bclass,classes,totMean);
   FreeIntVec(&gstack,streams);
}


static void EstMLLRCovXForm(AccStruct *accs, LinXForm *xf)
{
  DMatrix invG,u,v;
  DMatrix *InvG;
  DVector w;
  DMatrix A;
  int i,j,k,dim;
  int iter;
  int cnt, cnti,b,bsize;
  double det=0.0,tdet=0.0;
  double beta, likeNew, likeOld;
  DVector W, iniW;
  DVector cofact;
  
   /* Reset xform  to identity matrix */
  
   InvG = (DMatrix *)New(&gstack,sizeof(DMatrix)*(accs->dim+1)); 
   for (b=1,cnt=1;b<=IntVecSize(accs->blockSize);b++) {
     bsize = accs->blockSize[b];
     dim = bsize;
     cofact = CreateDVector(&gstack,bsize);
     ZeroDVector(cofact); 
     /* set up the matrices for the inversion */
     u = CreateDMatrix(&gstack, dim, dim);
     v = CreateDMatrix(&gstack, dim, dim);
     w = CreateDVector(&gstack, dim);
     /* and the transforms to be estimated */
     A = CreateDMatrix(&gstack,bsize,bsize);
     ZeroDMatrix(A); 
     W = CreateDVector(&gstack,dim);
     iniW = CreateDVector(&gstack,dim);
     /* initialise with the diagonal transform */
     for (i=1,cnti=cnt;i<=bsize;i++, cnti++) {
        A[i][i] = sqrt(accs->G[cnti][i][i]/accs->occ); 
        A[i][i] = 1/A[i][i];
        InvG[cnti] = CreateDMatrix(&gstack,dim,dim);
        Tri2DMat(accs->G[cnti],InvG[cnti]);
        InvSVD(InvG[cnti], u, w, v, InvG[cnti]);
     }
     for (iter=1;iter<=maxXFormIter;iter++) {
       ZeroDVector(iniW);
       for (i=1,cnti=cnt;i<=bsize;i++,cnti++) {
         for (j=1;j<=bsize;j++)      iniW[j] = A[i][j];
         invG = InvG[cnti];    
         det = DMatCofact(A,i,cofact);     
         beta = 0;
         for(j=1;j<=bsize;j++){
            for(k=1;k<=bsize;k++)
               beta += cofact[j]*invG[j][k]*cofact[k];
         }
         beta = sqrt(accs->occ/beta);
         ZeroDVector(W);
         for(j=1;j<=bsize;j++){
            for(k=1;k<=bsize;k++)
               W[j] += cofact[k]*invG[j][k];
            W[j] *= beta;          
         }
         ZeroDVector(w);
         likeNew = GetRowLike(accs->G[cnti],w,cofact,accs->occ,W);
         likeOld = GetRowLike(accs->G[cnti],w,cofact,accs->occ,iniW);
         /* printf("Iteration %d (row %d): ",iter,cnt); */
         if (likeNew>likeOld) {
           for(j=1;j<=bsize;j++)  A[i][j] = W[j];
         } else {
            if (likeOld/likeNew>1.00001) /* put a threshold on this! */
               printf("  Not updating transform (Block: %d Row: %d Iter: %d (%f %f))\n",
		      b,i,iter,likeNew/accs->occ,likeOld/accs->occ);
	 }
       }
     }
     cnt += bsize;
     tdet += log(fabs(det));
     /* Copy the transform into single precision for storage */
     for (i=1;i<=bsize;i++)
        for (j=1;j<=bsize;j++)
           xf->xform[b][i][j] = A[i][j];
     FreeDVector(&gstack, cofact);
   }
   xf->det = tdet*2;
   Dispose(&gstack,InvG);
}

static void EstXForm(AccStruct *accs, AdaptXForm *xform, IntVec classes)
{
  XFormSet *xformSet;
  LinXForm *xf, *dxf;
  IntVec diagBlockSize;
  int i;
  
  xformSet = xform->xformSet;
  xformSet->numXForms++;
  xf = xformSet->xforms[xformSet->numXForms] 
    = CreateLinXForm(xform->mem,accs->dim,accs->blockSize);
  switch (accs->xkind) {
  case MLLRMEAN:    
    EstMLLRMeanXForm(accs, xf);
    if (mllrDiagCov) { /* additional code to allow efficient diagonal cov */
      diagBlockSize = CreateIntVec(xform->mem,accs->dim);
      for (i=1;i<=accs->dim;i++) diagBlockSize[i] = 1;
      xformSet = diagCovXForm->xformSet;
      xformSet->numXForms++;
      dxf = xformSet->xforms[xformSet->numXForms] 
	= CreateLinXForm(xform->mem,accs->dim,diagBlockSize);
      dxf->bias = NULL;
      EstMLLRDiagCovXForm(accs,xf,dxf);
      FixDet(dxf);
    }
    break;
  case MLLRCOV:    
    EstMLLRCovXForm(accs, xf);
    break;
  case CMLLR: 
    EstCMLLRXForm(accs, xf);
    break;
  case SEMIT:
     EstSemiTXForm(xform,accs,xf,classes);
     break;
  default :
    HError(999,"Transform kind not currently supported");
    break;
  }
  if (trace&T_XFM)
    printf("Estimated XForm %d using %f observations\n",xformSet->numXForms,accs->occ);
}

void GenXForm(RegNode *node, AdaptXForm *xform, IntVec classes)
{
   AccStruct *accs;
   int class=1,b;
  
   /* First get dimension of data associated with this set of transforms */
   if (trace&T_TRE) {
      printf("Using node %d (%f) to adapt baseclasses: \n",node->nodeIndex,node->nodeOcc);
      for (b=1;b<=IntVecSize(classes);b++)
         if (classes[b] == 1) printf(" %d",b);
      printf("\n");
   }
   while (classes[class] == 0) class++;
   if (strmProj)
      accs = CreateAccStruct(&gstack,xform,xform->hset->vecSize,GetBlockSize(xform,class));
   else 
      accs = CreateAccStruct(&gstack,xform,node->vsize,GetBlockSize(xform,class));
   AccNodeStats(node,accs,xform,classes);
   EstXForm(accs,xform,classes);
   for (b=1;b<=IntVecSize(classes);b++)
      if (classes[b] == 1) {
	if (HardAssign(xform)) {
	  xform->xformWgts.assign[b] = xform->xformSet->numXForms;
	  if (mllrDiagCov) 
	    diagCovXForm->xformWgts.assign[b] = diagCovXForm->xformSet->numXForms;
	}
         else 
            HError(999,"Not currently supported");
      }
   Dispose(&gstack,accs);
}

static Boolean GenClassXForm(BaseClass *bclass, AdaptXForm *xform)
{
  AccStruct *accs;
  int b;
  ILink i;
  MixPDF *mp = NULL;
  IntVec classes;

  /* reset the number of transforms */
  xform->xformSet->numXForms = 0;
   /* 
      SEMIT is NOT possible in incremental mode and the stats are stored
      in a non-zero-mean form. Fix statistics once to be of appropriate form.
      Prior to fixing stats sort out the covariance matrix for variance
      flooring.
      
   */
  if (xform->xformSet->xkind == SEMIT) {
     SetSemiTiedAvCov(xform->hset);
     FixSemiTiedStats(xform->hset);
  }
  classes = CreateIntVec(&gstack,bclass->numClasses);
  for (b=1;b<=bclass->numClasses;b++) {
     ZeroIntVec(classes); classes[b] = 1;
    /* Accumulate structure regenerated each time as this will handle
       streams of different sizes simply */
     if (strmProj)
        accs = CreateAccStruct(&gstack,xform,xform->hset->vecSize,GetBlockSize(xform,b));
     else     
        accs = CreateAccStruct(&gstack,xform,GetVecSizeClass(bclass,b), GetBlockSize(xform,b));
    for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
      mp = ((MixtureElem *)i->item)->mpdf;
      AccMixPDFStats(xform->hset,mp,accs);
    }
    /* Use last component of the baseclass to access baseclass stats */
    if (AccAdaptBaseTriMat(xform))  AccBaseClassStats(mp,accs);
    if ((xform->xformSet->xkind == SEMIT) || (accs->occ > GetSplitThresh(xform))) {
      EstXForm(accs,xform,classes);
      xform->xformWgts.assign[b] = xform->xformSet->numXForms;
      if (mllrDiagCov) 
	diagCovXForm->xformWgts.assign[b] = diagCovXForm->xformSet->numXForms;
    } else {
       xform->xformWgts.assign[b] = 0;
    }
    Dispose(&gstack,accs);
  }
  return TRUE;
}

/* The transform generated from this can be applied to a model set */
static AdaptXForm *CreateBaseAdaptXForm(HMMSet *hset, char *xformName)
{
   AdaptXForm *xform;
   XFormAccInfo *info;
   XFormSet *xformSet;
   int b;
   char buf[MAXSTRLEN];

   xform = (AdaptXForm *)New(hset->hmem,sizeof(AdaptXForm));
   xform->xformName = xformName;
   xform->mem = hset->hmem;
   xform->hset = hset;
   xform->nUse = 0;
   xform->swapXForm = NULL;
   /* setup the baseclass */
   /* define a baseline xformset with no transforms */
   xform->xformSet = xformSet = (XFormSet *)New(hset->hmem,sizeof(XFormSet));
   xformSet->numXForms = 0;
   xformSet->nUse = 0;
   /* setup default values from config variables */
   xformSet->xkind = xKind; 
   xform->akind = GetAdaptKind(xform);
   /* Now sort out the correct set of baseclasses */
   switch (xform->akind) {
   case BASE:
      xform->bclass = GetBaseClass(hset,xform);
      break;
   case TREE:
      xform->rtree = GetRegTree(hset,xform);
      xform->bclass = xform->rtree->bclass;
      break;
   default:
      HError(999,"Transform kind not current supported");
   }
   /* confirm that the current model set is valid with this baseclass */
   if ((hset->hmmSetId != NULL) && (!MaskMatch(xform->bclass->mmfIdMask,buf,hset->hmmSetId)))
      HError(999,"HMM set ID %s is not compatible with base class mask %s",
	     hset->hmmSetId,xform->bclass->mmfIdMask);
   /* Create space for the weight vectors */
   if (HardAssign(xform)) {
      xform->xformWgts.assign = CreateIntVec(hset->hmem,xform->bclass->numClasses);
      for (b=1;b<=xform->bclass->numClasses;b++) xform->xformWgts.assign[b] = 0;
   } else 
      HError(999,"Not currently supported");
   /* create space for maximum number of transforms */
   xformSet->xforms = 
      (LinXForm **)New(hset->hmem,(xform->bclass->numClasses+1)*sizeof(LinXForm *));
   /* setup the xform accumulation information */
   xform->info = info = (XFormAccInfo *)New(hset->hmem,sizeof(XFormAccInfo));
   info->accSum = AccAdaptMean(xform);
   info->accSumSq = AccAdaptVar(xform);
   info->accBTriMat = AccAdaptBaseTriMat(xform);
   if (hset->parentXForm != NULL) {
      xform->parentXForm = hset->parentXForm;
      xform->parentXForm->nUse++;
   } else 
      xform->parentXForm = NULL;     
   return xform;
} 

/*------------------------------------------------------------------------*/
/*                  Manipulation of Adaptation Transforms                 */
/*------------------------------------------------------------------------*/

/* Product between two given matrices */
static void MatrixMult(Matrix m1, Matrix m2, Matrix m)
{  
   float tempElem;
   int i,j,k;
   Matrix mat;

   mat = CreateMatrix(&gstack,NumRows(m1),NumCols(m2));
   if (NumCols(m1)==NumRows(m2)){
      for (i=1;i<=NumRows(m);i++){
         for (j=1;j<=NumCols(m);j++){
            tempElem=0.0;
            for (k=1;k<=NumCols(m1);k++){
               tempElem+=m1[i][k]*m2[k][j];
            }
            mat[i][j]=tempElem;
         }
      }
      CopyMatrix(mat,m);
   }
   else {
      HError(999,"HMath: MatrixMult: Matrices not the same size!\n");
   }
   FreeMatrix(&gstack,mat);
}

static Boolean CompBlockSizes(IntVec blocks1, IntVec blocks2)
{
   int nblock1, nblock2;
   int i;

   nblock1 = IntVecSize(blocks1);
   nblock2 = IntVecSize(blocks2);
   if (nblock1 == nblock2) {
      for (i=1;i<=nblock1;i++) 
         if (blocks1[i] != blocks2[i])
            return FALSE;
   } else 
      return(FALSE);
   return (TRUE);
}

static void MultCovMeanLinXForms(LinXForm *xf1, LinXForm *xf2, LinXForm *xf)
{
   /* 
      Needs to handle general case: xf may be xf1, or xf2
   */
   int i,j;
   int cnt, cnti, cntj, bl, bsize;
   Matrix mat,imat,res;
   Vector bres,bias;
   
   bres = CreateVector(&gstack,xf->vecSize);
   ZeroVector(bres);
   if (CompBlockSizes(xf1->blockSize,xf2->blockSize)) {
      /* simplest case treat each block independently */
      for (bl=1,cnti=1,cnt=1;bl<=IntVecSize(xf1->blockSize);bl++) {
         bsize = xf1->blockSize[bl];
         mat = xf1->xform[bl];
         imat = CreateMatrix(&gstack,bsize,bsize);
         res = CreateMatrix(&gstack,bsize,bsize);
         MatInvert(mat,imat);
         MatrixMult(mat,xf2->xform[bl],res);
         if (mllrCov2CMLLR) {
            CopyMatrix(res,xf2->xform[bl]);
         } else {
            MatrixMult(res,imat,xf2->xform[bl]);
         }
         if (xf2->bias != NULL) {
            bias = xf2->bias;
            for (i=1;i<=bsize;i++,cnti++)
               for (j=1,cntj=cnt;j<=bsize;j++,cntj++) 
                  bres[cnti] += mat[i][j] * bias[cntj];
         }
         cnt += bsize;
      }
      if (xf2->bias != NULL) 
         CopyVector(bres,xf2->bias);
   } else {
      HError(999,"Not currently supported");
   }
   FreeVector(&gstack,bres);
}

static void SwapAdaptXForms(AdaptXForm *xform, AdaptXForm *paxform)
{
   AdaptXForm *txform;
   /* char buf[MAXSTRLEN]; */
   
   txform = paxform->parentXForm;
   paxform->parentXForm = xform;
   xform->parentXForm = txform;
   /* 
      Now correct the xformNames. Need to swap over the extension (if 
      any), of the current xform and the parent xform.
      MakeFN(paxform->xformName,NULL,"swap",buf);
      paxform->xformName= CopyString(paxform->mem,buf);
   */
}


static void SwapMLLRCovMLLRMean(AdaptXForm *xform, AdaptXForm *paxform)
{
   XFormSet *xfset, *paxfset;
   int i;

   xfset = xform->xformSet;
   paxfset = paxform->xformSet;
   if (xfset->numXForms == 1) { /* simplest case */
      /* The MLLRCov transform is not altered */
      for (i=1;i<=paxfset->numXForms;i++) {
         MultCovMeanLinXForms(xfset->xforms[1],paxfset->xforms[i],paxfset->xforms[i]);
      }
   }
   if (mllrCov2CMLLR)   xfset->xkind = CMLLR;       
   SwapAdaptXForms(xform, paxform);
   xform->swapXForm = paxform;
}

/* swaps the transform and it's parent around */
static void SwapXForm(HMMSet *hset, AdaptXForm *xform)
{
   AdaptXForm *paxform;
   XFormKind xkind1, xkind2;
   char buf1[MAXSTRLEN], buf2[MAXSTRLEN];

   if (xform->parentXForm == NULL) { /* no transform to swap */
      if (trace&T_SWP)
         printf("No transform to swap for %s\n",xform->xformName);
      return;
   }
   paxform = CopyAdaptXForm(hset->hmem,xform->parentXForm);

   /* 
      Currently check that the baseclasses are not going to be an issue 
      Need to alter code at some stage for general xform support.
   */
   if ((xform->bclass == paxform->bclass) || 
       (xform->bclass->numClasses == 1) ||
       (paxform->bclass->numClasses == 1)) {
      xkind1=xform->xformSet->xkind;
      xkind2=paxform->xformSet->xkind;
      if ((xkind2==MLLRMEAN) && (xkind1==MLLRCOV))
         SwapMLLRCovMLLRMean(xform,paxform);
      else {
         if (trace&T_SWP) {
            HError(999,"Inappropriate combination of transform kinds %s %s\n",
                   XFormKind2Str(xkind1,buf1),XFormKind2Str(xkind2,buf2));
            HError(999,"General transform swapping not currently supported\n");
         }
      }
   } else {
      if (trace&T_SWP) {
         printf("Inappropriate combination  of  baseclasses %s %s\n",
                xform->bclass->fname,paxform->bclass->fname);
         printf("General case not currently supported\n");
      }
   }
}

/*------------------------------------------------------------------------*/
/*                  External Routines for Adaptation code                 */
/*------------------------------------------------------------------------*/

/* ---------------- Accumulation Control Functions ---------------------- */

void AccAdaptFrame(HMMSet *hset, double Lr, Vector svec, MixPDF *mp, int t)
{
   RegAcc *ra;
   int i, vsize;

   vsize = VectorSize(svec);
   ra = GetRegAcc(mp);
   ra->occ += Lr;
   if (ra->spSum != NULL)
      for (i=1;i<=vsize;i++)
         ra->spSum[i] += Lr*svec[i];
   if (ra->spSumSq != NULL)
      for (i=1;i<=vsize;i++)
         ra->spSumSq[i] += Lr*svec[i]*svec[i];
   if (ra->bTriMat != NULL) 
      AccBaseTriMat(hset,Lr,svec,mp,t);
}

void ZeroAdaptAccs(HMMSet *hset, AdaptXForm *xform)
{
   int b;
   BaseClass *bclass;
   MixPDF *mp;
   RegAcc *ra = NULL;
   ILink i;
  
   if (hset->attRegAccs) {
      bclass = xform->bclass;
      for (b=1;b<=bclass->numClasses;b++) {
         for (i=bclass->ilist[b]; i!=NULL; i=i->next) {
            mp = ((MixtureElem *)i->item)->mpdf;
            ra = GetRegAcc(mp);
            ra->occ = 0;
            if (ra->spSum != NULL) ZeroVector(ra->spSum);
            if (ra->spSumSq != NULL) ZeroVector(ra->spSumSq);
         }
         /* Use last component of the baseclass to access baseclass stats */
         if (ra->bTriMat != NULL) ZeroBaseTriMat(ra->bTriMat);
      }
   }
}

/* ---------------- Applying Transform Functions ------------------------ */

void SetXForm(HMMSet *hset, AdaptXForm *xform)
{
   if (!(CompareXFormInfo(hset->curXForm, xform))) {
      SetAInfo(hset,xform,FALSE);
   } 
   SetObsCache(xform,FALSE);  
   if (storeMInfo) {
      if (hset->attMInfo) {
         if (!CompareMInfo(hset,xform))
            UpdateMInfo(hset,xform);
      } else {
         SetMInfo(hset,xform);
      }
   }
   /* set the flag whether input transform changes covariance or not */
   covarChanged = XFormModCovar(xform);
   hset->curXForm = xform;
}

void SetParentXForm(HMMSet *hset, AdaptXForm *xform)
{
   if (!(CompareXFormInfo(hset->parentXForm, xform))) {
      SetAInfo(hset,xform,TRUE);
   }
   SetObsCache(xform, TRUE);
   if (storeMInfo) {
      if (hset->attMInfo) {
         if (!CompareMInfo(hset,xform))
            UpdateMInfo(hset,xform);
      } else {
         SetMInfo(hset,xform);
      }
   }
   /* set the flag whether parent transform changes covariance or not */
   covarPChanged = XFormModCovar(xform);
   hset->parentXForm = xform;
   SetAccCache(outXForm);
}

void ApplyCompXForm(MixPDF *mp, AdaptXForm *xform)
{
   AInfo *ai = NULL;
   HMMSet *hset;

   if (xform != NULL) {
      hset = xform->hset;
      if (mp->info == NULL)
         HError(999,"No adaptation information for component");
      if (xform == hset->curXForm)  /* use adapt information from current Xform */
         ai = GetAInfo(mp);
      else if (xform == hset->parentXForm) {
         ai = GetPAInfo(mp);
      } else 
         HError(999,"Can only apply parent and current transform");
      CompXForm(mp,xform,ai);
   }
}

Vector ApplyCompFXForm(MixPDF *mp, Vector svec, AdaptXForm *xform, LogFloat *det, int t)
{
  AInfo *ai = NULL;
  ObsCache *oc = NULL;
  Vector vec;
  HMMSet *hset;

  if (xform == NULL) {
     *det = 0;
     return svec;
  }  else {
    hset = xform->hset;
    if (mp->info == NULL)
      HError(999,"No adaptation information for component");
    if (xform == hset->curXForm)  { /* use adapt information from current Xform */
      ai = GetAInfo(mp);
      oc = GetObsCache(mp);
    }
    else if (xform == hset->parentXForm) {
      ai = GetPAInfo(mp);
      oc = GetPAObsCache(mp);
    } else 
      HError(999,"Can only apply parent and current transform");

    *det = 0; 
    if ( oc != NULL ) {
      if (oc->time != t ) {
	vec = CreateVector(&gstack,VectorSize(svec));
	CopyVector(svec,vec);
        CompFXForm(mp,vec,xform,ai,det);
        UpdateObsCache(oc, vec, *det,t);  
	FreeVector(&gstack,vec); 
	if (trace&T_OBC) printf("Updated ObsCache %p at time %d\n",oc,t);
      } 
      vec = oc->obs;
      *det = oc->det;
      return vec;
    } else {
       *det = 0;
       return svec;
    }
  }
}

void ApplyHMMSetXForm(HMMSet *hset, AdaptXForm *xform)
{
  HMMScanState hss;
  int nAdpt=0;

  /* Only created XForm and parent Xform strutures */
  if ((xform != hset->curXForm) && (xform != hset->parentXForm))
    HError(999,"Can only apply parent and current transform");
  NewHMMScan(hset,&hss);
  do {
    while (GoNextState(&hss,TRUE)) {
      while (GoNextStream(&hss,TRUE)) {            
	if (hss.isCont)                     /* PLAINHS or SHAREDHS */
	  while (GoNextMix(&hss,TRUE)) {
	    ApplyCompXForm(hss.mp,xform);
	    nAdpt++;
	  }
      }
    }
  } while (GoNextHMM(&hss));
  EndHMMScan(&hss);
  if (trace&T_ADT) printf("Adapted %d components\n",nAdpt);
}

void ResetXFormHMMSet(HMMSet *hset)
{
   HMMScanState hss;

   NewHMMScan(hset,&hss);
   do {
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {            
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  ResetComp(hss.mp);
               }
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
}


/* ---------------  Transform Copying Functions  ----------------------- */

LinXForm *CopyLinXForm(MemHeap *x, LinXForm *xf)
{
   LinXForm *nxf;
   int bl,bsize;

   nxf = (LinXForm *)New(x,sizeof(LinXForm));
   nxf->vecSize = xf->vecSize;
   nxf->blockSize = CreateIntVec(x,IntVecSize(xf->blockSize));
   CopyIntVec(xf->blockSize,nxf->blockSize);
   nxf->det = xf->det;
   nxf->nUse = 0; /* new linXForm */
   if (xf->bias != NULL) {
      nxf->bias = CreateSVector(x,xf->vecSize);
      CopyVector(xf->bias,nxf->bias);
   } else
      nxf->bias = NULL;
   if (xf->vFloor != NULL) {
      nxf->vFloor = CreateSVector(x,VectorSize(xf->vFloor));
      CopyVector(xf->vFloor,nxf->vFloor);
   } else
      nxf->vFloor = NULL;
   nxf->xform = (Matrix *)New(x,(IntVecSize(xf->blockSize)+1)*sizeof(Matrix));
   for (bl=1;bl<=IntVecSize(xf->blockSize);bl++) {
      bsize = xf->blockSize[bl];
      nxf->xform[bl] = CreateSMatrix(x,bsize,bsize);
      CopyMatrix(xf->xform[bl],nxf->xform[bl]);
   }
   return nxf;
}

XFormSet *CopyXFormSet(MemHeap *x, XFormSet *xfset)
{
   XFormSet *nxfset;
   int i;

   nxfset = (XFormSet *)New(x,sizeof(XFormSet));
   nxfset->numXForms = xfset->numXForms;
   nxfset->xkind = xfset->xkind;
   nxfset->nUse = 0; /* new XFormSet */
   nxfset->xforms = (LinXForm **)New(x,(xfset->numXForms+1)*sizeof(LinXForm *));
   for (i=1;i<=xfset->numXForms;i++) {
      if (xfset->xforms[i]->nUse == 0)
         nxfset->xforms[i] = CopyLinXForm(x,xfset->xforms[i]);
      else 
         nxfset->xforms[i] = xfset->xforms[i];
   }
   return nxfset;
}

AdaptXForm *CopyAdaptXForm(MemHeap *x, AdaptXForm *xform)
{
   AdaptXForm *nxform;

   nxform = (AdaptXForm *)New(x,sizeof(AdaptXForm));
   nxform->xformName = CopyString(x,xform->xformName);
   nxform->fname = CopyString(x,xform->fname);
   nxform->mem  = x;
   nxform->akind = xform->akind;
   nxform->swapXForm = NULL;
   nxform->bclass = xform->bclass;
   nxform->rtree = xform->rtree;
   nxform->nUse = 0; /* This is a new transform */
   nxform->info = xform->info;
   nxform->parentXForm = xform->parentXForm;
   nxform->hset = xform->hset;
   if (xform->xformSet->nUse == 0) 
      nxform->xformSet = CopyXFormSet(x,xform->xformSet);
   else
      nxform->xformSet = xform->xformSet;
   if (HardAssign(xform)) {
      nxform->xformWgts.assign = CreateIntVec(x,IntVecSize(xform->xformWgts.assign));
      CopyIntVec(xform->xformWgts.assign,nxform->xformWgts.assign);
   } else {
      HError(999,"Not currently supported");
   }
   
   return nxform;
}

/* ---------------  Transform Estimation Functions ----------------------- */

AdaptXForm *CreateAdaptXForm(HMMSet *hset, char *xformName)
{
   AdaptXForm *xform;

   /* The macroname is not defined at this stage, to avoid
      over-writing the old version */
   xform = CreateBaseAdaptXForm(hset,xformName);
   if (mllrDiagCov) { /* additional code for efficient diag Cov */     
     diagCovXForm = CreateBaseAdaptXForm(hset,xformName);
     /* fix the parent xform and attributes */
     if (xform->parentXForm != NULL) xform->parentXForm->nUse--;
     diagCovXForm->parentXForm = xform;
     diagCovXForm->xformSet->xkind = MLLRVAR;
     diagCovXForm->info = xform->info;
   }
   if (!hset->attRegAccs) AttachRegAccs(hset,xform);
   outXForm = xform;
   return xform;
}

InputXForm *AdaptXForm2InputXForm(HMMSet *hset, AdaptXForm *xform)
{
   InputXForm *ixform;
   int s,i,j,cnt,hldasize;
   char mac[256], num[10];
   LabId id;
   MLink m;
   SVector v;
   SMatrix mat;

   if (xform->bclass->numClasses != 1)
      HError(999,"Can only use Stream Projections with single base classes");
   ixform = (InputXForm *)New(hset->hmem,sizeof(InputXForm));
   ixform->xformName = xform->xformName;
   ixform->mmfIdMask = xform->bclass->mmfIdMask;
   ixform->fname = NULL;
   ixform->pkind = xform->hset->pkind;
   ixform->preQual = FALSE;
   ixform->xform = xform->xformSet->xforms[1];
   if ((numNuisanceDim > 0) || (hset->projSize>0)) { /* adjust size to remove HLDA */
      if (numNuisanceDim>0) hldasize = hset->vecSize-numNuisanceDim;
      else hldasize = hset->vecSize-hset->projSize;
      mat = CreateSMatrix(hset->hmem,hldasize,hset->vecSize);
      for (i=1;i<=hldasize;i++)
         for (j=1;j<=hset->vecSize;j++)
            mat[i][j] = ixform->xform->xform[1][i][j];
      ixform->xform->xform[1] = mat;
   } 
   /* 
      Update the HMMSet vFloor and then zero xf vFloor. For input
      xforms the HMMSet vFloor always  used.
   */
   if ((numNuisanceDim > 0) || (hset->projSize>0)) {
      strcpy(mac,"varFloor1");
      id = GetLabId(mac,FALSE);
      if (id != NULL  && (m=FindMacroName(hset,'v',id)) != NULL){
         /* found the varFloor to update */
         if (numNuisanceDim>0) hldasize = hset->vecSize-numNuisanceDim;
         else hldasize = hset->vecSize-hset->projSize;
         v = CreateSVector(hset->hmem,hldasize);
         for (i=1;i<=hldasize;i++) 
            v[i] = ixform->xform->vFloor[i];
         m->structure = v;
      }
   } else {
      for (s=1,cnt=1;s<=hset->swidth[0];s++) {
         strcpy(mac,"varFloor");
         sprintf(num,"%d",s); strcat(mac,num);
         id = GetLabId(mac,FALSE);
         if (id != NULL  && (m=FindMacroName(hset,'v',id)) != NULL){
            /* found the varFloor to update */
            v = (SVector)m->structure;
            for (i=1;i<=hset->swidth[s];i++,cnt++) 
               v[i] = ixform->xform->vFloor[cnt];
         }
      }
   }
   ixform->xform->vFloor = NULL;
   ixform->nUse = 0;

   return ixform;
}

Boolean GenAdaptXForm(HMMSet *hset, AdaptXForm* xform)
{
   AdaptKind akind;
   Boolean genXForm = FALSE;

   akind = xform->akind;
   switch(akind) {
   case TREE: 
      genXForm = ParseTree(xform->rtree,xform);
      break;
   case BASE:
      genXForm = GenClassXForm(xform->bclass,xform);
      break;
   default:
      HError(999,"Only TREE and BASE adaptation kinds currently supported");
   }
   /* support ability to generate and swap transforms */
   if (swapXForms) SwapXForm(hset, xform);
   return genXForm;
}

void TidyBaseAccs()
{
   SetBaseAccsTime(-1);
   if (headac == NULL )
      UpdateBaseAccs(NULL);
   else { 
      UpdateBaseAccsWithPaac();
      ResetAccCache();
   }
}

AdaptXForm *GetMLLRDiagCov(AdaptXForm *xform)
{
   if (diagCovXForm == NULL)
      return xform;
   else 
      return diagCovXForm;
}

/* 
   UpdateSpkrStats: monitor speaker changes and generate transforms
   at each speaker boundary, returns TRUE when the output speaker
   has changed
*/
Boolean UpdateSpkrStats(HMMSet *hset, XFInfo *xfinfo, char *datafn)
{
   char newFn[MAXSTRLEN];
   char newMn[MAXSTRLEN];
   char spkr[MAXSTRLEN];
   char paspkr[MAXSTRLEN];
   static char coutspkr[MAXSTRLEN];
   static char cinspkr[MAXSTRLEN];
   static char cpaspkr[MAXSTRLEN];
   static int nspkr = 0;
   Boolean resetHMMSet = FALSE, maskMatch;
   Boolean spkrChange = FALSE;

   if (!((hset->hsKind == PLAINHS) || (hset->hsKind == SHAREDHS))
       && (xfinfo->useOutXForm || xfinfo->useInXForm || xfinfo->usePaXForm )) {
      HError(999,"Adaptation only supported for PLAINHS and SHAREDHS systems");
   }
   /* First: handle output transform generation */
   if (xfinfo->useOutXForm) { /* if there is an output transform to be generated */
      maskMatch = MaskMatch(xfinfo->outSpkrPat,spkr,datafn);
      if ((!maskMatch) && (datafn != NULL))
         HError(999,"Output xform mask %s does not match filename %s",xfinfo->outSpkrPat,datafn);
      if ((datafn == NULL) || ((coutspkr!=NULL) && strcmp(spkr,coutspkr))) {
         /* end of current speaker, so complete his/her transform */
         if (nspkr>0) { /* nothing to generate if first speaker */
            if (trace&T_SXF)
               printf("Generating transform %s (%i)\n",coutspkr,nspkr);
            /* Tidy the statistics of the last frame */
            SetBaseAccsTime(-1);
            if (headac == NULL )
               UpdateBaseAccs(NULL);
            else { 
               UpdateBaseAccsWithPaac();
               ResetAccCache();
            }
            /* Generate the new transform */
            MakeFN(coutspkr,NULL,xfinfo->outXFormExt,newMn);
            xfinfo->outXForm = CreateAdaptXForm(hset, newMn);
            GenAdaptXForm(hset,xfinfo->outXForm);
	    if (mllrDiagCov) xfinfo->outXForm = diagCovXForm;
            /* After generating a transform need to reset parameters */
            resetHMMSet = TRUE;
            if (keepXFormDistinct) {  /* Output individual transform */
               MakeFN(coutspkr,xfinfo->outXFormDir,xfinfo->outXFormExt,newFn);
               SaveOneXForm(hset,xfinfo->outXForm,newFn,xfinfo->saveBinary);
            } else { /* Create macro from the masked speaker name and extension */
               MakeFN(coutspkr,NULL,xfinfo->outXFormExt,newMn);
               CreateXFormMacro(hset,xfinfo->outXForm,newMn);
            }
            if (saveSpkrModels) { 
               /* 
                  output distinct model for each speaker. The macro name
                  including extension is used to distinguish models.
                  First set the adaptation up.
               */
               SetXForm(hset,xfinfo->outXForm);
               ApplyHMMSetXForm(hset, xfinfo->outXForm);	   
               ForceDiagC(hset);
               SaveHMMSet(hset,xfinfo->outXFormDir,newMn,newMn,xfinfo->saveBinary);
               ConvDiagC(hset,TRUE);
               /* remembering to reset the transform */
               if (xfinfo->useInXForm) {
		 SetXForm(hset,xfinfo->inXForm);
               }else{
		 SetXForm(hset,NULL);
	       }
            }
            spkrChange = TRUE;
            ZeroAdaptAccs(hset,xfinfo->outXForm);
            if (xfinfo->usePaXForm && (datafn != NULL)) {
               maskMatch = MaskMatch(xfinfo->paSpkrPat,paspkr,datafn);
               if (!maskMatch)
                  HError(999,"Parent xform mask %s does not match filename %s",xfinfo->paSpkrPat,datafn);
               /* parent transform changed and not the last file? */
               if (strcmp(paspkr,cpaspkr)) { 
                  strcpy(cpaspkr,paspkr);
                  MakeFN(cpaspkr,xfinfo->paXFormDir,xfinfo->paXFormExt,newFn);
                  MakeFN(cpaspkr,NULL,xfinfo->paXFormExt,newMn);
                  xfinfo->paXForm = LoadOneXForm(hset,newMn,newFn);
                  SetParentXForm(hset,xfinfo->paXForm);
               }
            } else if (hset->semiTied != NULL) { /* model set uses a semitied transform */
               if (xfinfo->paXForm == NULL) { /* semi-tied transform is for all files */
                  xfinfo->paXForm = hset->semiTied;
                  SetParentXForm(hset,xfinfo->paXForm);
               }
            }
         } else if (xfinfo->usePaXForm) { /* set-up the initial parent transform information */
            maskMatch = MaskMatch(xfinfo->paSpkrPat,paspkr,datafn);
            if (!maskMatch)
               HError(999,"Parent xform mask %s does not match filename %s",xfinfo->paSpkrPat,datafn);
            strcpy(cpaspkr,paspkr);
            MakeFN(cpaspkr,xfinfo->paXFormDir,xfinfo->paXFormExt,newFn);
            MakeFN(cpaspkr,NULL,xfinfo->paXFormExt,newMn);
            xfinfo->paXForm = LoadOneXForm(hset,newMn,newFn);
            SetParentXForm(hset,xfinfo->paXForm);
         } else if (hset->semiTied != NULL) { /* model set uses a semitied transform */
            if (xfinfo->paXForm == NULL) { /* semi-tied transform is for all files */
               xfinfo->paXForm = hset->semiTied;
               SetParentXForm(hset,xfinfo->paXForm);
            }
         }
         nspkr++;
         strcpy(coutspkr,spkr);      
      } else if (xfinfo->usePaXForm) { 
         /* check to see whether the parent transform changes */
         /* this should not happen */
         MaskMatch(xfinfo->paSpkrPat,paspkr,datafn);
         if (strcmp(paspkr,cpaspkr)) 
            HError(999,"Changing parent transform out of sync with output transform (%s %s)",
                   paspkr,cpaspkr);
      }
   } else if (xfinfo->usePaXForm && (datafn != NULL)) {
      /* Parent transform specified with no output transform */
      maskMatch = MaskMatch(xfinfo->paSpkrPat,paspkr,datafn);
      if (!maskMatch)
         HError(999,"Parent xform mask %s does not match filename %s",xfinfo->paSpkrPat,datafn);
      /* parent transform changed and not the last file? */
      if (strcmp(paspkr,cpaspkr)) { 
         strcpy(cpaspkr,paspkr);
         MakeFN(cpaspkr,xfinfo->paXFormDir,xfinfo->paXFormExt,newFn);
         MakeFN(cpaspkr,NULL,xfinfo->paXFormExt,newMn);
         xfinfo->paXForm = LoadOneXForm(hset,newMn,newFn);
         SetParentXForm(hset,xfinfo->paXForm);
      }
      spkrChange=TRUE;
   } else if (hset->semiTied != NULL) { /* model set uses a semitied transform */
      if (xfinfo->paXForm == NULL) { /* semi-tied transform is for all files */
         xfinfo->paXForm = hset->semiTied;
         SetParentXForm(hset,xfinfo->paXForm);
      }
      spkrChange=TRUE;
   } else 
      spkrChange=TRUE;

   /* Second: handle input transform */
   if (xfinfo->useInXForm  && (datafn != NULL)) {
      maskMatch = MaskMatch(xfinfo->inSpkrPat,spkr,datafn);
      if (!maskMatch)
         HError(999,"Input xform mask %s does not match filename %s",xfinfo->inSpkrPat,datafn);
      /* if a transform has been changed the input transform must be 
         reapplied */
      if (((cinspkr!=NULL) && strcmp(spkr,cinspkr)) || (resetHMMSet)) {
         if (trace&T_SXF)
            printf("Using input transform %s\n",spkr);
         strcpy(cinspkr,spkr);      
         MakeFN(cinspkr,NULL,xfinfo->inXFormExt,newMn);
         xfinfo->inXForm = LoadOneXForm(hset,newMn,NULL);
         SetXForm(hset,xfinfo->inXForm);
         ApplyHMMSetXForm(hset,xfinfo->inXForm);
         if (xfinfo->al_hset != NULL) {
            MakeFN(cinspkr,xfinfo->alXFormDir,xfinfo->alXFormExt,newFn);
            MakeFN(cinspkr,NULL,xfinfo->alXFormExt,newMn);
            xfinfo->al_inXForm = LoadOneXForm(xfinfo->al_hset,newMn,newFn);
            SetXForm(xfinfo->al_hset,xfinfo->al_inXForm);
            ApplyHMMSetXForm(xfinfo->al_hset,xfinfo->al_inXForm);
         } else {
            xfinfo->al_inXForm = xfinfo->inXForm;
         }
      }
   } else if (resetHMMSet && (xfinfo->usePaXForm || (xfinfo->inXForm != NULL) || saveSpkrModels)) { 
      /* 
         Reset model parameters as transform generated using 
         a parent transform - it is possible to be more efficient 
         if the nature of the transform is also considered 
      */
      ResetXFormHMMSet(hset);
   } else { /* handle the fact that alignment and update HMMSet may use semi-tied transforms */
      if ((xfinfo->inXForm == NULL) && (xfinfo->al_inXForm == NULL)) { /* semi-tied transform is for all files */
         if (hset->semiTied != NULL) { /* model set uses a semitied transform */
            xfinfo->inXForm = hset->semiTied;
            SetXForm(hset,xfinfo->inXForm);
         }
         if (xfinfo->al_hset != NULL) {
            if (xfinfo->al_hset->semiTied != NULL) { /* alignment model set uses a semitied transform */
               xfinfo->al_inXForm = xfinfo->al_hset->semiTied;
               SetXForm(xfinfo->al_hset,xfinfo->al_inXForm);
            }
         } else {
            xfinfo->al_inXForm = xfinfo->inXForm;
         }
      }
   }

   /* All the files have been handled - store xforms */
   if (datafn == NULL) { 
      if (!keepXFormDistinct) {
         if (xfinfo->xformTMF == NULL) {
            MakeFN("TMF",xfinfo->outXFormDir,NULL,newFn);
            SaveAllXForms(hset,newFn,xfinfo->saveBinary);
         } else 
            MakeFN(xfinfo->xformTMF,xfinfo->outXFormDir,NULL,newFn);
            SaveAllXForms(hset,newFn,xfinfo->saveBinary);
      }
   }
   return spkrChange;
}

void UpdateSemiTiedModels(HMMSet *hset, XFInfo *xfinfo)
{
   char newFn[MAXSTRLEN];
   char *newMn;
   InputXForm *ixform;

   if (semiTiedMacro == NULL) {
      if (numNuisanceDim > 0) newMn = "HLDA";
      else newMn = "SEMITIED";
   } else newMn = semiTiedMacro;
   xfinfo->outXForm = CreateAdaptXForm(hset, newMn);
   GenAdaptXForm(hset,xfinfo->outXForm);
   CreateXFormMacro(hset,xfinfo->outXForm,newMn);
   MakeFN(newMn,xfinfo->outXFormDir,xfinfo->outXFormExt,newFn);
   if ((strmProj) || (semiTied2InputXForm)) {
      hset->semiTied = NULL;
      ixform = AdaptXForm2InputXForm(hset,xfinfo->outXForm);
      hset->xf = ixform;
      if (numNuisanceDim>0) UpdateHLDAModel(hset);
      SaveInputXForm(hset,ixform,newFn,saveSemiTiedBinary);
   } else {
      hset->projSize = numNuisanceDim;
      hset->semiTied = xfinfo->outXForm;
      SaveOneXForm(hset,xfinfo->outXForm,newFn,saveSemiTiedBinary);
   }
   /* All parameters updated - now do GConst */
   FixAllGConsts(hset);
}

void UpdateProjectModels(HMMSet *hset, char *dir)
{
   char newFn[MAXSTRLEN];
   char *newMn;
   InputXForm *ixform;
   AdaptXForm* xform;

   if (semiTiedMacro == NULL) newMn = hset->semiTiedMacro;
   else newMn = semiTiedMacro;
   xform = hset->semiTied;
   ixform = AdaptXForm2InputXForm(hset,xform);
   hset->xf = ixform; hset->semiTied = NULL;
   UpdateHLDAModel(hset);
   MakeFN(newMn,dir,NULL,newFn);
   SaveInputXForm(hset,ixform,newFn,saveSemiTiedBinary);
   /* gconsts fixed in main script */
}


