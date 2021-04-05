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
/* main authors: Valtcho Valtchev, Steve Young,                */
/*               Julian Odell, Gareth Moore                    */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*          1994-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*      File: LModel:    ARPA style LM handling                */
/* ----------------------------------------------------------- */


char *lmodel_version = "!HVER!LModel:   3.4.1 [CUED 12/03/09]";
char *lmodel_vc_id = "$Id: LModel.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";


#include "HShell.h"     /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#ifdef ULTRA_LM
#include "HDict.h"
#endif
#include "LWMap.h"
#include "LUtil.h"
#include "LModel.h"
#include "HLM.h"

#define T_TOP    0001       /* top level tracing */
#define T_LOAD   0002       /* loading of LMs */
#define T_SAVE   0004       /* saving of LMs */
#define T_MAPS   0010       /* word mappings */
#define T_PROB   0020       /* n-gram lookup */

static int trace = 0;

typedef struct _AccessInfo{
   int count;           /* count for access */
   int nboff;           /* times computed using the back-off weight */
   int nmiss;           /* times not available */
   int nhits;           /* times available */
   double prob;         /* sum of prob   returned */
   double prob2;        /* sum of prob^2 returned */
} accessinfo;


static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

static char *nGramName[LM_NSIZE] = {
   "NULLGRAM", "UNIGRAM",
   "BIGRAM", "TRIGRAM",
   "FOURGRAM", "PENTAGRAM",
   "HEXAGRAM","SEPTAGRAM","OCTAGRAM",
   "NONAGRAM","DECAGRAM","11-GRAM",
   "12-GRAM","13-GRAM","14-GRAM","15-GRAM"
};

static char *dcTypeName[] = {
   "Katz",
   "Absolute",
   "Linear"
};

static Boolean defIntID = FALSE;        /* Don't use 4-byte IDs */
static Boolean htkEsc = FALSE;          /* Don't use HTK quoting and escapes */
static Boolean natReadOrder = FALSE;    /* Preserve natural read byte order */
static Boolean natWriteOrder = FALSE;   /* Preserve natural write byte order */
extern Boolean vaxOrder;                /* True if byteswapping needed to preserve SUNSO */
#ifdef ULTRA_LM
static short   ultraKey[KEY_LENGTH];    /* Key used to identify ultra LMs */
#endif

/* EXPORT->InitLModel: initialise module */
void InitLModel(void)
{
   int i;
   Boolean b;

   Register(lmodel_version,lmodel_vc_id);
   nParm = GetConfig("LMODEL", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
#ifdef HTK_TRANSCRIBER
      if (trace&T_PROB) trace=trace^T_PROB;
#endif
      if (GetConfBool(cParm,nParm,"RAWMITFORMAT",&b)) htkEsc = !b;
      if (GetConfBool(cParm,nParm,"USEINTID",&b)) defIntID = b;
      if (GetConfBool(cParm,nParm,"NATURALREADORDER",&b)) natReadOrder = b;
      if (GetConfBool(cParm,nParm,"NATURALWRITEORDER",&b)) natWriteOrder = b;
   }
#ifdef ULTRA_LM
   COMPOSE_KEY(ultraKey);
#endif
}

/*----------------------- Input scanner ------------------------*/

#define MAXSYMLEN  2048

/* GetInLine: read a complete line from source */
static char *GetInLine(Source *src,char *buf)
{
   int  i, c;

   if ((c = GetCh(src))==EOF)
      return NULL;
   i = 0;
   while (c!='\n' && i<MAXSYMLEN) {
      buf[i++] = c;
      c = GetCh(src);
   }
   buf[i] = '\0';
   return buf;
}

/* SyncStr: read input until str found */
void SyncStr(Source *src, char *str)
{
   char buf[MAXSYMLEN];

   do {
      if (GetInLine(src,buf)==NULL)
         HError(15450,"SyncStr: EOF searching for %s", str);
   } while (strcmp(buf,str)!=0);
}

/*----------------------- Access statistics ------------------------*/

void ResetAccessInfo(BackOffLM *lm)
{
   int i;
   NGramInfo *gi;
   AccessInfo *ai;

   for (gi=lm->gInfo+1,i=1; i<=lm->nSize; i++,gi++) {
      if ((ai=gi->aInfo)==NULL)
	 HError(15490,"ResetAccessInfo: Access info not present");
      ai->count = 0;
      ai->nboff = ai->nmiss = ai->nhits = 0;
      ai->prob = ai->prob2 = 0.0;
   }
}

/* EXPORT->AttachAccessInfo: attach and initialise access info */
void AttachAccessInfo(BackOffLM *lm)
{
   int i;
   NGramInfo *gi;

   for (gi=lm->gInfo+1,i=1; i<=lm->nSize; i++,gi++) {
      if (gi->aInfo!=NULL)
	 HError(15490,"AttachAccessInfo: Access info already present");
      gi->aInfo = (AccessInfo *) New(lm->heap,sizeof(AccessInfo));
   }
   ResetAccessInfo(lm);
}

/* ShowStats: print back-off statistics */
static void ShowStats(FILE *f, AccessInfo *acs, char *lmstr)
{
   int count;
   float f1, f2, f3;
   double a, b, avg, stdev;

   count = (acs->count>0) ? acs->count : 1;

   a = acs->prob  / (double) count;
   b = acs->prob2 / (double) count;
   avg = a; stdev = sqrt(b - a*a);
   f1 = 100.0 * (float) acs->nhits / (float) count;
   f2 = 100.0 * (float) acs->nboff / (float) count;
   f3 = 100.0 * (float) acs->nmiss / (float) count;
   fprintf(f,"%10s %10d %5.1f%% %5.1f%% %5.1f%% %8.2f %8.2f\n",
	   lmstr, acs->count, f1, f2, f3, avg, stdev);
}

/* EXPORT -> PrintTotalAccessStats: print access statistics */
void PrintTotalAccessStats(FILE *f,BackOffLM *lm)
{
   int i;
   NGramInfo *gi;
   static char *lmstr[] = {
      "nullgram", "unigram", "bigram", "trigram", "fourgram", "pentagram",
      "hexagram", "septagram", "octagram", "nonagram", "decagram"
   };
   static int max_text = 10; /* size of lmstr[] array */
   char tmpstr[10];

   fprintf(f,"%10s %10s %6s %6s %6s %8s %8s\n", "Lang model",
	  "requested", "exact", "backed", "n/a", "mean", "stdev");
   for (gi=lm->gInfo+2,i=2; i<=lm->nSize; i++, gi++)
      ShowStats(f,gi->aInfo, i<=max_text?lmstr[i]:(sprintf(tmpstr, "%d", i), tmpstr));
}

/*----------------------- float compression ----------------------*/

#define MIN_PROB -8.0

#ifdef LM_COMPACT

static UShort Prob2Shrt(float f)
{
   if (f < MIN_PROB)
      return USHRT_MAX;
   return (f / MIN_PROB * (float) (USHRT_MAX-1));
}

static float Shrt2Prob(UShort s)
{
   if (s == USHRT_MAX)
      return LZERO;
   return ((float) s / (float) (USHRT_MAX-1)) * MIN_PROB;
}

#endif

/*-------------------------- LM access ----------------------------*/

/* EXPORT-> CmpSE: qsort comparison for short LM entries */
int CmpSE(const void *p1, const void *p2)
{
   if (((SMEntry *)p1)->ndx < ((SMEntry *)p2)->ndx)
      return -1;
   if (((SMEntry *)p1)->ndx > ((SMEntry *)p2)->ndx)
      return +1;
   return 0;
}

/* EXPORT-> CmpFE: qsort comparison for full LM entries */
int CmpFE(const void *p1, const void *p2)
{
   if (((FLEntry *)p1)->ndx < ((FLEntry *)p2)->ndx)
      return -1;
   if (((FLEntry *)p1)->ndx > ((FLEntry *)p2)->ndx)
      return +1;
   return 0;
}

/* EXPORT-> FindSE: find SEntry in a sorted list */
SMEntry *FindSE(SMEntry *sptr, int lo, int hi, LM_Id key)
{
   int cen;
   LM_Id cmp;

   if (sptr==NULL)
      return NULL;
   hi--;
   if ((key < sptr[lo].ndx) || (key > sptr[hi].ndx))
      return NULL;
   do {
      cen = (lo + hi) / 2;
      cmp = sptr[cen].ndx;
      if (key == cmp)
	 return sptr+cen;
      if (key > cmp)
	 lo = cen+1;
      else
	 hi = cen-1;
   } while (lo <= hi);
   return NULL;
}

/* EXPORT-> FindFE: find FEntry in a sorted list */
FLEntry *FindFE(FLEntry *fptr, int lo, int hi, LM_Id key)
{
   int cen;
   LM_Id cmp;

   if (fptr==NULL)
      return NULL;
   hi--;
   if ((key < fptr[lo].ndx) || (key > fptr[hi].ndx))
      return NULL;
   do {
      cen = (lo + hi) / 2;
      cmp = fptr[cen].ndx;
      if (key == cmp)
	 return fptr+cen;
      if (key > cmp)
	 lo = cen+1;
      else
	 hi = cen-1;
   } while (lo <= hi);
   return NULL;
}

/* FindSE1: find bigram entry in a sorted list, also return index */
static SMEntry *FindSE1(SMEntry *sptr, int lo, int hi, LM_Id key, int *fcen)
{
   int cen;
   LM_Id cmp;

   hi--;
   if ((key < sptr[lo].ndx) || (key > sptr[hi].ndx))
      return NULL;
   do {
      cen = (lo + hi) / 2;
      cmp = sptr[cen].ndx;
      if (key == cmp) {
	 *fcen = cen; return sptr+cen;
      }
      if (key > cmp)
	 lo = cen+1;
      else
	 hi = cen-1;
   } while (lo <= hi);
   return NULL;
}

/* -------------------- Ultra format I/O ---------------------- */

#ifdef ULTRA_LM

static CNEntry *qs_cneBuf;      /* global table of read CNEntry */

/* FRead: fread spec function for Source src */
static size_t FRead(void *ptr, size_t size, size_t nitems, Source *src)
{
   int nr,i;
   unsigned char *c;

   nr = fread(ptr,size,nitems,src->f);
#ifdef HTK_CRYPT
   if (src->crypt!=NULL) {
      for (c=ptr,i=0; i<size*nr; i++,c++)
	 *c = DecryptChar(src->crypt,*c);
   }
#endif
   src->chcount+=nr*size;
   return nr;
}

#ifdef LMPROB_SHORT
/*
   The following compress/decompress LOG10 float to/from short.
*/

#define PROB_LOG_TO_SHORT(prob) \
  ((int) (-prob/0.0002+0.5) > 65534 ? 65535 : (int) (-prob/0.0002+0.5))

#define PROB_SHORT_TO_LOG(prob) \
  (prob<=65534 ? -prob*0.0002 : LZERO)

#define BOWT_LOG_TO_SHORT(bowt) \
   ((floor(bowt/0.0002+0.5)>32766)?32767:\
    (floor(bowt/0.0002+0.5)<-32767)?-32768:\
    (int)(floor(bowt/0.0002+0.5)))

#define BOWT_SHORT_TO_LOG(bowt) \
   (bowt*0.0002)

#else
#define PROB_LOG_TO_SHORT(prob) (prob)
#define PROB_SHORT_TO_LOG(prob) (prob)
#define BOWT_LOG_TO_SHORT(bowt) (bowt)
#define BOWT_SHORT_TO_LOG(bowt) (bowt)
#endif

#define CNE2FE(cndx,fe) {  \
   CNEntry *cne = cneBuf + cndx; \
   fe->nse  = cne->nse;  \
   fe->sea  = smeTab[cndx]; \
   fe->ndx  = cne->word[0]; \
   bowt = BOWT_SHORT_TO_LOG(cne->bowt); \
   fe->bowt = (ptype==LMP_FLOAT) ? LOG10_TO_FLT(bowt) : bowt*scale; \
}

#define INIT_CNE(cne) { \
   int  i;         \
   cne.nse = 0;    \
   cne.bowt = 0.0; \
   for (i=0; i<NSIZE-1; i++) cne.word[i]=0;   \
}

static int nep_cmp(const void *v1,const void *v2)
{
   CNEntry *n1,*n2;
   int res,i;

   res = 0;
   n1=qs_cneBuf + *((int *)v1);
   n2=qs_cneBuf + *((int *)v2);
   for(i=NSIZE-2;i>=0;i--)
      if (n1->word[i]!=n2->word[i]) {
	 res=(n1->word[i]-n2->word[i]);
	 break;
      }
   return(res);
}

static void LoadUltraNGrams(Source *src, BackOffLM *lm)
{
   float prob,bowt,scale;
   Boolean newCTX;
   LMProbType ptype;
   int context[NSIZE+1];
   int i,j,idx,cneCnt,seCnt;
   SEntry se;
   int *cneTab;
   CNEntry *cne,*cneBuf;
   FLEntry *cfe,*feBuf,*parent;
   SMEntry *sme,*smeBuf,**smeTab;
   Boolean mustSwap = (vaxOrder && !natReadOrder);

   SyncStr(src,"\\N-grams:");

   scale = lm->gScale*LN10;
   ptype = lm->probType;
   if (FRead(&cneCnt,sizeof(int32),1,src)!=1)
      HError(15450,"LoadUltraNGrams: Unable to read CNEntry count");
   if (mustSwap) SwapInt32(&cneCnt);
   if (FRead(&seCnt,sizeof(int32),1,src)!=1)
      HError(15450,"LoadUltraNGrams: Unable to read SEntry count");
   if (mustSwap) SwapInt32(&seCnt);

   if (trace&T_LOAD)
      printf("Loading %d NEntry(s)\n",cneCnt);
   cneBuf = (CNEntry *) New(&gstack,cneCnt*sizeof(NEntry));
   if (FRead(cneBuf,sizeof(CNEntry),cneCnt,src)!=cneCnt)
      HError(15450,"LoadUltraNGrams: Unable to read CNEntry array");
   if (mustSwap) {
      for (cne=cneBuf,i=0; i<cneCnt; i++,cne++) SWAP_CE(cne);
   }

   if (trace&T_LOAD)
      printf("Loading %d SEntry(s)\n",seCnt);
   /* create and read SMEntry block */
   smeBuf = (SMEntry *) New(lm->heap,seCnt*sizeof(SMEntry));
   for (sme=smeBuf,i=0; i<seCnt; i++,sme++) {
      if (FRead(&se,sizeof(SEntry),1,src)!=1)
	 HError(15450,"LoadUltraNGrams: Unable to read SEntry (%d)",i);
      if (mustSwap) SWAP_SE((&se));
#ifdef LM_COMPACT
      sme->prob = se.prob;
#else
      prob = PROB_SHORT_TO_LOG(se.prob);
      sme->prob = (ptype==LMP_FLOAT) ? LOG10_TO_FLT(prob) : prob * scale;
#endif
      sme->ndx = se.word;
   }
   /* create table of pointers to SMEntry arrays */
   smeTab = (SMEntry **) New(&gstack,cneCnt*sizeof(SMEntry *));
   for (sme=smeBuf,j=0; j<cneCnt; j++) {
     smeTab[j] = (cneBuf[j].nse==0) ? NULL : (SMEntry *) sme;
     sme += cneBuf[j].nse;
   }
   /* create and sort lookup index array */
   cneTab = (int *) New(&gstack,sizeof(int)*cneCnt);
   for (i=0; i<cneCnt; i++) cneTab[i] = i;
   qs_cneBuf = cneBuf;
   qsort(cneTab,cneCnt,sizeof(int),nep_cmp);

   feBuf = (FLEntry *) New(lm->heap,cneCnt*sizeof(FLEntry));

   parent = &lm->root;
   CNE2FE(cneTab[0],parent);
   parent->nfe = 0;
   parent->fea = cfe = feBuf;
   parent->parent = 0;
   for (i=0; i<NSIZE-1; i++) context[i] = cneBuf[cneTab[1]].word[i];

   for (i=1; i<cneCnt; i++) {
      cne = cneBuf+cneTab[i];
      for (newCTX=FALSE,j=1; j<NSIZE-1; j++) {
	 if (context[j]!=cne->word[j]) {
	    newCTX=TRUE; break;
	 }
      }
      if (newCTX) {
	 for (parent=&lm->root,j=NSIZE-2; j>0; j--) {
	    if ((idx=cne->word[j])==0) continue;
	    if ((parent = FindFE(parent->fea,0,parent->nfe,idx))==NULL) {
	       HError(15450,"LoadUltraNGrams: Items not in order %d",i);
	    }
	 }
	 parent->fea = cfe; parent->nfe = 0;
	 for (j=0; j<NSIZE-1; j++) context[j] = cne->word[j];
      }
      parent->nfe++; CNE2FE(cneTab[i],cfe); cfe++;
   }
   Dispose(&gstack,cneBuf);
}

static int WriteNEntry(FILE *f, BackOffLM *lm, int lev, FLEntry **feBuf,
		       FLEntry **feTab, int *fetCount)
{
   int i,total;
   CNEntry ne;
   float scale,bowt;
   LMProbType ptype;
   FLEntry *fe,*tgtFE;
   Boolean mustSwap = (vaxOrder && !natWriteOrder);

   if (lev==lm->nSize)
      return 0;
   ptype = lm->probType;
   scale = 1.0/(lm->gScale*LN10);
   tgtFE = feBuf[lev-1];
   total = 0;

   INIT_CNE(ne);
   for (i=1; i<lev; i++) ne.word[lev-i] = feBuf[i]->ndx;
   for (fe = tgtFE->fea, i=0; i<tgtFE->nfe; i++, fe++) {
      if (fe->nse==0)
	 continue;
      feTab[(*fetCount)++] = fe;
      ne.word[0] = fe->ndx;
      ne.nse = fe->nse;
      bowt = (ptype==LMP_FLOAT) ? FLT_TO_LOG10(fe->bowt) : fe->bowt*scale;  /* convert to LOG10 */
      ne.bowt = BOWT_LOG_TO_SHORT(bowt);   /* compress LOG10 to short */
      if (mustSwap) {
	 SWAP_CE((&ne));
	 fwrite(&ne,sizeof(CNEntry),1,f);
	 SWAP_CE((&ne));
      } else {
	 fwrite(&ne,sizeof(CNEntry),1,f);
      }
      total++;
   }
   if (++lev < lm->nSize) {
      for (fe = tgtFE->fea, i=0; i<tgtFE->nfe; i++, fe++) {
	 feBuf[lev-1] = fe;
	 total += WriteNEntry(f,lm,lev,feBuf,feTab,fetCount);
      }
   }
   return total;
}

static int WriteSEntry(FILE *f,BackOffLM *lm,FLEntry **feTab, int fetCount)
{
   SEntry se;
   SMEntry *sme;
   FLEntry *fe;
   int i,j,total = 0;
   float scale,prob;
   LMProbType ptype;
   Boolean mustSwap = (vaxOrder && !natWriteOrder);

   ptype = lm->probType;
   scale = 1.0/(lm->gScale*LN10);
   total = 0;
   for (i=0; i<fetCount; i++) {
      fe = feTab[i];
      for (sme=fe->sea,j=0; j<fe->nse; j++,sme++) {
	 prob = (ptype==LMP_FLOAT) ? FLT_TO_LOG10(sme->prob) : sme->prob*scale;
	 se.prob = PROB_LOG_TO_SHORT(prob);   /* LOG10 -> short */
	 se.word = sme->ndx;
	 if (mustSwap) {
	    SWAP_SE((&se));
	    fwrite(&se,sizeof(SEntry),1,f);
	    SWAP_SE((&se));
	 } else {
	    fwrite(&se,sizeof(SEntry),1,f);
	 }
	 total++;
      }
   }
   return total;
}

static void CountEntries(int lev, int nSize, FLEntry *tgtFE, int *nfe, int *nse)
{
   int i;
   FLEntry *fe;

   *nse += tgtFE->nse;
   if (lev < nSize)
      *nfe += tgtFE->nfe;
      for (fe = tgtFE->fea, i=0; i<tgtFE->nfe; i++, fe++)
	 CountEntries(lev+1,nSize,fe,nfe,nse);
}

static void SaveUltraNGrams(FILE *f, BackOffLM *lm)
{
   int n,neCnt,seCnt,fetCount;
   CNEntry ne;
   FLEntry *feBuf[LM_NSIZE], **feTab;
   Boolean mustSwap = (vaxOrder && !natWriteOrder);

   fprintf(f,"\n\\N-grams:\n");
   neCnt = seCnt = 0;
   CountEntries(1,lm->nSize,&lm->root,&neCnt,&seCnt);
   neCnt++;

   if (mustSwap) {
      SwapInt32(&neCnt);
      fwrite(&neCnt,sizeof(int32),1,f);
      SwapInt32(&neCnt);
   } else {
      fwrite(&neCnt,sizeof(int32),1,f);
   }
   if (mustSwap) {
      SwapInt32(&seCnt);
      fwrite(&seCnt,sizeof(int32),1,f);
      SwapInt32(&seCnt);
   } else {
      fwrite(&seCnt,sizeof(int32),1,f);
   }

   INIT_CNE(ne);                         /* write the root entry */
   ne.nse = lm->root.nse;
   if (mustSwap) {
      SWAP_CE((&ne));
      fwrite(&ne,sizeof(CNEntry),1,f);
      SWAP_CE((&ne));
   } else {
      fwrite(&ne,sizeof(CNEntry),1,f);
   }
   feTab = (FLEntry **) New(&gstack,neCnt*sizeof(FLEntry *));
   fetCount = 0;
   feTab[fetCount++] = &lm->root;

   if (lm->nSize > 1) {
      feBuf[0] = &lm->root;
      WriteNEntry(f,lm,1,feBuf,feTab,&fetCount);
      if (trace&T_SAVE) {
	 printf("saved %d CNEntry(s), (%d)\n",fetCount,neCnt); fflush(stdout);
      }
   }
   n = WriteSEntry(f,lm,feTab,fetCount);
   if (trace&T_SAVE) {
      printf("saved %d SEntry(s), (%d)\n",n,seCnt); fflush(stdout);
   }
   Dispose(&gstack,feTab);
}
#endif  /* ULTRA_LM */

/*------------------------- LM loading -------------------------*/

#define READ_FLOAT(src,x,bin) { \
   char buf[100]; \
   if (!ReadFloat(src,x,1,bin)) \
      HError(15490,"ReadFloat: Float expected at %s",SrcPosition(*src,buf)); \
}


/* EXPORT-> StoreFEA: move fea array into permanent location */
void StoreFEA(FLEntry *fe, MemHeap *heap)
{
   FLEntry *febuf;

   if (fe==NULL)
      return;
   if (fe->nfe==0) {
      fe->fea = NULL;
   } else {
      qsort(fe->fea, fe->nfe, sizeof(FLEntry), CmpFE);
      febuf = (FLEntry *) New(heap,fe->nfe*sizeof(FLEntry));
      fe->fea = memcpy(febuf, fe->fea, fe->nfe*sizeof(FLEntry));
   }
}

/* EXPORT-> StoreSEA: move fea array into permanent location */
void StoreSEA(FLEntry *fe, MemHeap *heap)
{
   SMEntry *sebuf;

   if (fe==NULL)
      return;
   if (fe->nse==0) {
      fe->sea = NULL;
   } else {
      qsort(fe->sea, fe->nse, sizeof(SMEntry), CmpSE);
      sebuf = (SMEntry *) New(heap,fe->nse*sizeof(SMEntry));
      fe->sea = memcpy(sebuf, fe->sea, fe->nse*sizeof(SMEntry));
   }
}

/* LoadUnigram: read the unigram part of a file */
static int LoadUnigram(Source *src, BackOffLM *lm, int *itran)
{
   char word[256];
   int i,tndx,nItem;
   float bowt,prob,scale;
   LM_Id ndx;
   SMEntry *se;
   FLEntry *fe;
   NameId wdid;
   LMProbType ptype;
   Boolean has_bowt;

   nItem = 0;
   scale = lm->gScale*LN10;
   ptype = lm->probType;
   lm->root.sea = se = lm->se_buff; lm->root.nse = 0;
   lm->root.fea = fe = lm->fe_buff; lm->root.nfe = 0;
   lm->root.bowt = 0.0;
   lm->root.parent = 0;

   SyncStr(src,"\\1-grams:");
   for (i=1; i<=lm->gInfo[1].nEntry; i++){
      READ_FLOAT(src,&prob,FALSE);
      if (!GetSrcString(src,word,htkEsc))
	 HError(15413,"LoadUnigram: Unable to read unigram %d",i);
      SkipWhiteSpace(src);
      if (!src->wasNewline) {            /* process backoff weight */
	 READ_FLOAT(src,&bowt,FALSE);
	 has_bowt = TRUE;
      } else {
	 has_bowt = FALSE;
      }
      if (itran!=NULL) {
	 if ((wdid = GetNameId(lm->htab,word,FALSE))==NULL) {
	    itran[i] = -1;
	    continue;
	 }
	 if ((tndx = LM_INDEX(wdid)) > 0)
	    HError(15450,"LoadUnigram: Duplicate unigram %s",word);
	 ndx = itran[i] = -tndx;  /* indices pre-assigned as negative */
      } else {
	 wdid = GetNameId(lm->htab,word,TRUE);
	 ndx = i;
      }
      nItem++;
      lm->binMap[ndx] = wdid; /* This is where the wordlist is built */
      se->ndx = LM_INDEX(wdid) = ndx;
      switch(ptype) {
         case LMP_FLOAT :
	   se->prob = LOG10_TO_FLT(prob); break;
         case LMP_LOG :
#ifdef LM_COMPACT
	   se->prob = Prob2Shrt(prob); break;
#else
	   se->prob = prob * scale; break;
#endif
         default:
	   if (prob < 0.0)
	      HError(15450,"LoadUnigram: Negative probability (%.4f) for unigram %d",
		     prob,i);
	   se->prob = prob; break;
      }
      se++; lm->root.nse++;
      if (has_bowt) {         /* process backoff weight */
	 fe->ndx = ndx;
	 fe->nse = 0; fe->sea = NULL;
	 fe->nfe = 0; fe->fea = NULL;
	 switch(ptype) {
	    case LMP_FLOAT :
	      fe->bowt = LOG10_TO_FLT(bowt); break;
	    case LMP_LOG :
	      fe->bowt = bowt*scale; break;
	    default :
	      fe->bowt = bowt; break;
	 }
	 fe++; lm->root.nfe++;
      }
   }
   if (itran!=NULL && nItem!=lm->vocSize) {
      /* create dummy entries for unseen unigrams */
      for (i=1; i<=lm->vocSize; i++) {
	 if ((tndx = LM_INDEX(lm->binMap[i])) > 0)
	    continue;
	 LM_INDEX(lm->binMap[i]) = -tndx;
	 se->ndx = -tndx; se->prob = 0.0;
	 se++; lm->root.nse++;
      }
   }
   StoreFEA(&(lm->root),lm->heap);
   StoreSEA(&(lm->root),lm->heap);

   /* check unigram consistency */
   for (se=lm->root.sea, i=0; i<lm->root.nse; i++, se++) {
      if (se->ndx!=i+1)
	 HError(15450, "LoadUnigram: Mismatched unigram index %d should be %d", se->ndx, i+1);
   }
   return nItem;
}

#define TRINDEX(itran,i) (itran==NULL) ? i : itran[i]

/* LoadNGram: read n-gram (N>1) from file f */
static int LoadNGram(Source *src, int nSize, BackOffLM *lm, int *itran)
{
   LM_Id ndx=0;
   NGramInfo *gi;
   LMProbType ptype;
   Byte fsize, flags;
   SMEntry *se=NULL;
   FLEntry *feptr=NULL, *fe=NULL;
   float prob,bowt,scale;
   int i,j,k,num_fe,num_se; /*,n*/
   char *s,lnBuf[256],word[256];
   Boolean has_bowt, hasOOV, newCTX, isBin=FALSE;
   NameId wdid[LM_NSIZE], keyid[LM_NSIZE];

   if (nSize==1) {
      return LoadUnigram(src,lm,itran);
   }

   scale = lm->gScale*LN10;
   ptype = lm->probType;
   gi = lm->gInfo+nSize;
   if (gi->fmt==LMF_BINARY || gi->fmt==LMF_TEXT)
      isBin = (gi->fmt==LMF_BINARY);
   else
      HError(15450,"LoadNGram: Unknown LM file format (%d)\n",gi->fmt);
   if (trace&T_LOAD) {
      printf("Loading %d %d-grams (%s)\n",
	     lm->gInfo[nSize].nEntry,nSize,isBin ? "bin":"text");
      fflush(stdout);
   }
   num_fe = num_se = 0;
   keyid[0] = NULL; /* Previous context */

   sprintf(lnBuf, "\\%d-grams:",nSize); SyncStr(src,lnBuf);
   for (i=0; i<lm->gInfo[nSize].nEntry; i++) {
      has_bowt = FALSE; hasOOV = FALSE;
      if (isBin) {  /* binary model */
	 fsize = (Byte) GetCh(src);
	 flags = (Byte) GetCh(src);
	 READ_FLOAT(src,&prob,TRUE);
	 for (j=0; j<nSize; j++) {
	    if (flags&INT_LMID) {
	       UInt a;
	       ReadInt(src,(int *)&a,1,TRUE);
	       ndx = (LM_Id) a;
	    } else {
	       UShort a;
	       ReadShort(src,(short *)&a,1,TRUE);
	       ndx = (LM_Id) a;
	    }
	    if (itran!=NULL && itran[ndx]<0) {
	       hasOOV = TRUE;
	    } else {
	       if (itran!=NULL) ndx = itran[ndx];
	       if ((ndx > 0) && (ndx <=lm->vocSize))
		  wdid[j] = lm->binMap[ndx];
	       else
		  HError(15450,"LoadNGram: LM index out of bounds (%d)", ndx);
	    }
	 }
	 if (flags&HAS_BOWT) {
	    READ_FLOAT(src,&bowt,TRUE);
	    has_bowt = TRUE;
	 }
      } else { 	/* text model */
	 READ_FLOAT(src,&prob,FALSE);
	 for (j=0; j<nSize; j++) {     /* read n-gram words */
	    if (!GetSrcString(src,word,htkEsc))
	       HError(15450,"LoadNGram: Unable to read word %d of %d-gram",j,nSize);
	    if ((wdid[j] = GetNameId(lm->htab,word,FALSE))==NULL) {
	       if (itran==NULL)
		  HError(-15450, "LoadNGram: Word %s not in unigrams, skipping n-gram", word);
	       hasOOV = TRUE;
	    } else {
	       ndx = LM_INDEX(wdid[j]);
	    }
	 }
 	 SkipWhiteSpace(src);
	 if (!src->wasNewline) {
	    READ_FLOAT(src,&bowt,FALSE);
	    has_bowt = TRUE;
	 }
      }
      if (hasOOV) continue;

      /* See if the context has changed */
      for (newCTX=FALSE, j=0; j<nSize-1; j++) {
	 if (keyid[j]!=wdid[j]) {
	    newCTX=TRUE; break;
	 }
      }
      /* Guaranteed to execute the first time through the loop because the context
	 has not been seen before (thus defining fe, se etc) */
      if (newCTX) {             /* new n-gram context */
	 if (keyid[0]!=NULL) {  /* copy to permanent storage */
	    StoreFEA(feptr,lm->heap); num_fe += feptr->nfe;
	    StoreSEA(feptr,lm->heap); num_se += feptr->nse;
	 }
	 for (feptr = &(lm->root), j=0; j<nSize-1; j++) {
	    if ((feptr = FindFE(feptr->fea, 0, feptr->nfe, LM_INDEX(wdid[j])))==NULL) {
	       for (s=lnBuf,k=0; k<nSize; k++) {
		  sprintf(s,"%s[%d] ",wdid[k]->name,LM_INDEX(wdid[k])); s+=strlen(s);
	       }
	       HError(15420, "LoadNGram: Cannot find component %d of (%d) %d-gram %s",
		      j,i,nSize,lnBuf);
	    }
	    keyid[j] = wdid[j];
	 }
	 feptr->fea = fe = lm->fe_buff; feptr->nfe = 0;
	 feptr->sea = se = lm->se_buff; feptr->nse = 0;
      }
      se->ndx = ndx;
      switch(ptype) {
         case LMP_FLOAT :
	   se->prob = LOG10_TO_FLT(prob); break;
         case LMP_LOG :
#ifdef LM_COMPACT
	   se->prob = Prob2Shrt(prob); break;
#else
	   se->prob = prob * scale; break;
#endif
         default:
	   se->prob = prob; break;
      }
      se++; (feptr->nse)++;
      if (has_bowt) {
	 /* also store as full entry */
	 fe->ndx = ndx;
	 fe->nse = 0; fe->sea = NULL;
	 fe->nfe = 0; fe->fea = NULL;
	 switch(ptype) {
	    case LMP_FLOAT :
	      fe->bowt = LOG10_TO_FLT(bowt); break;
	    case LMP_LOG :
	      fe->bowt = bowt*scale; break;
	    default :
	      fe->bowt = bowt; break;
	 }
	 fe++; (feptr->nfe)++;
      }
   }
   if (keyid[0]!=NULL) {  /* store the last accumulated */
      StoreFEA(feptr,lm->heap); num_fe += feptr->nfe;
      StoreSEA(feptr,lm->heap); num_se += feptr->nse;
   }
   /*
      if (isBin) {        // read the last 2 zero bytes
   //         ReadShort(src,&ndx,1,TRUE);
      }
   */
   if (trace&T_LOAD) {
      printf("  SMEntry: %8d x %2d bytes = %d bytes\n",
	     num_se, sizeof(SMEntry), num_se*sizeof(SMEntry));
      printf("  FLEntry: %8d x %2d bytes = %d bytes\n",
	     num_fe, sizeof(FLEntry), num_fe*sizeof(FLEntry));
   }
   lm->gInfo[0].nEntry+=num_fe;
   return num_se;
}

/* Create reverse lookup pointers in FLEntry context tree */
/* Call with (lm->root, 0) and let it recurse its way down */
void CreateReverseLookup(FLEntry *fes)
{
   int i; /* loop counter */

   for (i=0; i<fes->nfe; i++) {
      fes->fea[i].parent = fes;
      CreateReverseLookup(&(fes->fea[i]));
   }
}

/* EXPORT-> ReadHeaderInfo: read header information */
/* First parameter is source file, second is LM structure, and
   third parameter is first input line or NULL to read from file */
void ReadHeaderInfo(Source *src, BackOffLM *lm, char *line1)
{
   float ff;
   int i,j,n;
   char *s,*s1,*s2=NULL;
   DiscountType dt;
   BackOffInfo *bo;
   DiscountInfo *di;
   char lnBuf[MAXSYMLEN],*sbuf;

   lm->probType = LMP_FLOAT|LMP_LOG;
   for (i=1; i<LM_NSIZE; i++)
      lm->gInfo[i].boInfo = NULL;

   while(line1 || GetInLine(src, lnBuf)) {
      if (line1) {
         strcpy(lnBuf, line1);
         line1 = NULL; /* Read the rest from the file */
      }

      if ((s=strstr(lnBuf,"\\data\\"))!=NULL && s==lnBuf) {
	 break;                   /* gone past header, so exit */
      }
      if (strcmp(lnBuf,"COUNTS")==0) {
	lm->probType = LMP_COUNT; continue;
      }
      for (i=1; i<LM_NSIZE; i++) {   /* try each n-gram name in turn */
	 if ((s=strstr(lnBuf,nGramName[i]))==NULL || s!=lnBuf)
	    continue;
	 bo = (BackOffInfo *) New(lm->heap,sizeof(BackOffInfo));
	 di = &(bo->dcInfo);

	 if (trace&T_LOAD)
	    printf("Parsing %s header info\n",nGramName[i]);
	 for (dt=DC_LAST,j=0; j<DC_LAST; j++) {
	    if (strstr(lnBuf,dcTypeName[j])!=NULL) {
	       dt = j; break;
	    }
	 }
	 if (dt==DC_LAST)
	    HError(15450,"LoadHeaderInfo: Unable to parse d-type in %s",lnBuf);
	 bo->dcType = dt;
	 if ((s1=strstr(lnBuf,"cutoff"))==NULL)
	    HError(15450,"LoadHeaderInfo: Unable to find 'cutoff' in %s",lnBuf);
	 if (sscanf(s1,"cutoff %d",&n)!=1)
	    HError(15450,"LoadHeaderInfo: Unable to parse cutoff value in %s",lnBuf);
	 bo->cutOff = n; bo->wdThresh = 0.0;
	 if (!GetInLine(src,lnBuf))
	    HError(15450,"LoadHeaderInfo: EOF reading d-coefs for %s",nGramName[i]);
	 switch (dt) {
	 case DC_KATZ:
	   if ((s1 = strchr(lnBuf,'['))==NULL || (s2 = strchr(lnBuf,']'))==NULL)
	     HError(15450,"LoadHeaderInfo: Unable to find array bounds in %s",lnBuf);
	   *s2='\0'; sbuf = s2+1;
	   di->tgInfo.kRange = n = atoi(s1+1);
	   di->tgInfo.coef = (float *) New(lm->heap,(n+1)*sizeof(float));
	   for (j=1; j<=n; j++) {
	     s1 = strtok((j==1)?sbuf:NULL," \t\r\n:");
	     if (s1==NULL)
	       HError(15450,"LoadHeaderInfo: Unable to parse coef %d in %s",j,lnBuf);
	     di->tgInfo.coef[j]=atof(s1);
	   }
	   break;
	 case DC_ABSOLUTE:
	   if ((s1=strstr(lnBuf,"coef:"))==NULL)
	     HError(15450,"LoadHeaderInfo: Unable to find 'coef:' in %s",lnBuf);
	   if (sscanf(s1,"coef: %f",&ff)!=1)
	     HError(15450,"LoadHeaderInfo: Unable to parse float value in %s",s1);
	   di->bCoef=ff;
	   break;
	 default :
	   HError(15450,"LoadHeaderInfo: Unsupported LM type (%d)",dt);
	   break;
	 }
	 lm->gInfo[i].boInfo = bo;
      }
   }
}

/* EXPORT-> WriteHeaderInfo: write header information */
void WriteHeaderInfo(FILE *f, BackOffLM *lm)
{
   int i,j;
   BackOffInfo *bo;
   DiscountInfo *di;

   if (lm->probType==LMP_COUNT)
      fprintf(f,"COUNTS\n\n");
   for (i=2; i<=lm->nSize; i++) {
      if ((bo = lm->gInfo[i].boInfo)==NULL)
	 continue;
      di = &(bo->dcInfo);
      if (bo->wdThresh>0)
	 fprintf(f, "%s: method %s, cutoff %d, wdThresh %.3f\n",
		 nGramName[i], dcTypeName[bo->dcType], bo->cutOff, bo->wdThresh);
      else
	 fprintf(f, "%s: method %s, cutoff %d\n",
		 nGramName[i], dcTypeName[bo->dcType], bo->cutOff);
      switch (bo->dcType) {
         case DC_KATZ :
	    fprintf(f, "  coef[%d]:", di->tgInfo.kRange);
	    for (j=1; j<=di->tgInfo.kRange; j++)
	       fprintf(f," %.6f", di->tgInfo.coef[j]);
	    fprintf(f,"\n");
	    break;
	 case DC_ABSOLUTE :
	    fprintf(f, "  coef: %.6f\n", di->bCoef);
	    break;
	 case DC_LINEAR :
	 default:
            break;
      }
      fprintf(f, "\n");
   }
}


/* ReadClassProbsHeader: read in word|class probabilities header */
static void ReadClassProbsHeader(char *fname, int *nWords, Source *src, BackOffLM *lm)
{
   char   line[MAXSYMLEN];   /* Current input line */
   char  *ptr;               /* Temporary pointers */

   *nWords = -1;

   if (!src->f) {
      /* Open file if necessary */
      if (InitSource(fname, src, LangModFilter)!=SUCCESS) {
         HError(15410, "ReadClassProbsHeader: Unable to open language model word|class file '%s'", fname);
      }
   }

   strcpy(line, "");
   GetInLine(src, line);
   if (strncmp(line, "Word|Class probabilities", 25)==0) {
      lm->classCounts = FALSE;
   }
   else if (strncmp(line, "Word|Class counts", 17)==0) {
      lm->classCounts = TRUE;
   }
   else {
      HError(15450, "ReadClassProbsHeader: Language model word|class file is in unknown format");
   }
   if (trace & T_LOAD) {
      printf("Word|class file uses word %s\n", lm->classCounts?"counts":"probabilities");
   }

   while (GetInLine(src, line)) {
      if (strncmp(line, "Number of classes", 17)==0) {
         ptr = strchr(line, ':');
         if (!ptr) {
            HError(15450, "ReadClassProbsHeader: Corrupt 'Number of classes' line in word|class file");
         }
         ptr++;
         while (*ptr==' ' || *ptr=='\t') ptr++;
         if (trace & T_LOAD) {
            printf("Number of classes = %d\n", atoi(ptr));
         }
      }
      else if (strncmp(line, "Number of words", 15)==0) {
         ptr = strchr(line, ':');
         if (!ptr) {
            HError(15450, "ReadClassProbsHeader: Corrupt 'Number of words' line in word|class file");
         }
         ptr++;
         while (*ptr==' ' || *ptr=='\t') ptr++;
         *nWords = atoi(ptr);
         if (trace & T_LOAD) {
            printf("Number of words = %d\n", *nWords);
         }
      }
      else if ((strncmp(line, "Word", 4)==0) || (strncmp(line, "Class", 5)==0)) {
         break;
      }
   }
   if (feof(src->f)) {
      HError(15450, "ReadClassProbsHeader: Word|Class language model file contains no %s", lm->classCounts?"counts":"probabilities");
   }

   if (*nWords == -1) {
      HError(15450, "ReadClassProbsHeader: Failed to find number of words header in word|class file");
   }
}


/* ReadClassCounts: read in word|class counts file */
static void ReadClassCounts(Source *src, int nWords, BackOffLM *lm)
{
   char   line[MAXSYMLEN];   /* Current input line */
   char  *ptr, *ptr2;        /* Temporary pointers */
   int    i;                 /* Loop counter */
   WordProb *wordProb;       /* Temporary pointer */
   int    loop=1;            /* Array index counter */
   NameId nid, nid2;         /* Word ids */
   int    class_id=0;        /* Number classes from 0 */
   int    floor_count = 0;   /* Number of counts floored */

   /* Add labels and wordlist entries for words */
   for (i=0; i<nWords; i++) {
      ptr = GetInLine(src, line);
      if (!ptr || strlen(ptr)==0) {
         HError(15450, "ReadClassCounts: Blank line/end of file in word|class language model file");
      }

      /* Segment line into word, class and count */
      /* Don't use strtok() in case a client program is using it */
      ptr2 = ptr + strcspn(ptr, " \t"); /* Find end of word */
      *ptr2 = '\0';

      /* Get name ID */
      nid = GetNameId(lm->classH, ptr, TRUE);

      /* Find class name */
      ptr = ptr2 + 1; /* Pass over NULL */
      ptr += strspn(ptr, " \t"); /* Skip whitespace */
      ptr2 = ptr + strcspn(ptr, " \t"); /* Find end of class name */
      *ptr2 = '\0';

      nid2 = GetNameId(lm->htab, ptr, TRUE); /* Get name id of class */
      class_id = atoi(ptr+5) - 1; /* assume called CLASSn */ /* GLM */
      nid2->ptr = (void*) class_id;

      ptr = ptr2 + 1; /* Pass over NULL */
      ptr += strspn(ptr, " \t"); /* Skip over whitespace */

      lm->word[i] = atoi(ptr); /* Store word count */

      if (lm->word[i]<=0) {
         floor_count++;
         lm->word[i] = 1; /* Force zero counts to 1 in order to avoid 0 probabilities */
         if (floor_count==5)
            HError(-15450, "ReadClassCounts: too many floored counts to list");
         else if (floor_count<5)
            HError(-15450, "ReadClassCounts: flooring zero count to one for '%s'", nid->name);
      }

      /* Create structure storing word|class probability and class of word */
      wordProb = New(&gcheap, sizeof(WordProb));
      wordProb->class = nid2;
      wordProb->prob = 0; /* we haven't calculated this yet */
      wordProb->id = i;

      nid->ptr = wordProb; /* Point word name id here */

      /* Set up binMap equivalent */
      lm->classBM[loop] = nid;
      LM_INDEX(nid) = -i; /* assign negative indices (copied code) */
      loop++;
   }

   /* Check for left over lines */
   while (GetInLine(src, line)) {
      if (strlen(line)>0) {
         HError(15450, "ReadClassCounts: Extraneous line on end of Word|Class probabilities file\n('%s')", line);
      }
   }
   if (floor_count)
      HError(-15450, "ReadClassCounts: a total of %d counts were floored", floor_count);
}


/* CountClassTotals: calculate class count totals for LM */
static void CountClassTotals(BackOffLM *lm)
{
   register int i; /* Loop counter */
   int word_id, class_id;

   lm->totals = New(&gcheap, lm->vocSize * sizeof(int));
   for (i=0; i<lm->vocSize; i++) {
      lm->totals[i] = 0;
   }

   for (i=0; i<(lm->classW); i++) {
      word_id = ((WordProb*)(lm->classBM[i+1]->ptr))->id;
      if (word_id!=i) HError(15490, "CountClassTotals: Inconsistent word ids found");

      class_id = (int)(((WordProb*)(lm->classBM[i+1]->ptr))->class->ptr);
      lm->totals[class_id] += lm->word[i];
   }
}


/* CalcWordClassProbs: calculate initial/static word|class probabilities */
static void CalcWordClassProbs(BackOffLM *lm)
{
   int i; /* loop counter */
   int class_id;
   double prob=0;

   /* For each word */
   for (i=0; i<lm->classW; i++) {
      class_id = (int)(((WordProb*)(lm->classBM[i+1]->ptr))->class->ptr);
      prob = (((double)(lm->word[i]))) / ((double)(lm->totals[class_id]));
      ((WordProb*)(lm->classBM[i+1]->ptr))->prob = LOG_NATURAL(prob);
   }
}


/* ReadClassProbs: read in word|class probabilities file */
static void ReadClassProbs(Source *src, int nWords, BackOffLM *lm)
{
   char   line[MAXSYMLEN];   /* Current input line */
   char  *ptr, *ptr2;        /* Temporary pointers */
   int    i;                 /* Loop counter */
   WordProb *wordProb;       /* Temporary pointer */
   int loop=1;
   NameId nid, nid2;

   /* Add labels and wordlist entries for words */
   for (i=0; i<nWords; i++) {
      ptr = GetInLine(src, line);
      if (!ptr || strlen(ptr)==0) {
         HError(15450, "ReadClassProbs: Blank line/end of file in word|class language model file");
      }

      /* Segment line into word, class and log probability */
      /* We could use strtok(), but I can't be sure that this isn't being
         used elsewhere wrapped around this call, so I won't! */
      ptr2 = ptr + strcspn(ptr, " \t"); /* Find end of word */
      *ptr2 = '\0';

      /* Get name ID */
      nid = GetNameId(lm->classH, ptr, TRUE);

      /* Find class name */
      ptr = ptr2 + 1; /* Pass over NULL */
      ptr += strspn(ptr, " \t"); /* Skip whitespace */
      ptr2 = ptr + strcspn(ptr, " \t"); /* Find end of class name */
      *ptr2 = '\0';

      nid2 = GetNameId(lm->htab, ptr, TRUE); /* Get name id of class */

      ptr = ptr2 + 1; /* Pass over NULL */
      ptr += strspn(ptr, " \t"); /* Skip over whitespace */

      /* Create structure storing word|class probability and class of word */
      wordProb = New(&gcheap, sizeof(WordProb));
      wordProb->class = nid2;
      wordProb->prob = atof(ptr);

      wordProb->id = -1;

      nid->ptr = wordProb; /* Point word name id here */

      /* Set up binMap equivalent */
      lm->classBM[loop] = nid;
      LM_INDEX(nid) = -i; /* assign negative indices (copied code) */
      loop++;
   }

   /* Check for left over lines */
   i=0;
   while (GetInLine(src, line)) {
      if (strlen(line)>0) {
         if (i>10) {
            HError(-15451, "ReadClassProbs: Further extraneous lines not shown");
            break;
         }
         HError(-15451, "ReadClassProbs: Extraneous line on end of Word|Class probabilities file\n('%s')", line);
         i++;
      }
   }
}


/* EXPORT-> LoadLangModel: read N-gram language model from fn */
BackOffLM *LoadLangModel(char *fn, WordMap *wl, float gramScale,
			 LMProbType tgtPType, MemHeap *heap)
{
   Source src;
   NGramInfo *gi;
   BackOffLM *lm;
   int *itran,nSize,i,n;
   char c,sfmt[256];
   char lnBuf[MAXSYMLEN];
   Boolean isUltra;
   char *first_line;         /* First line of input file */
   char wc_fname[MAXSYMLEN]; /* Filename of word|class probs */
   Source wcSrc;             /* word|class probs/counts file */
   int nWords;               /* Number of words in total over all classes */
   char *ptr;

   if ((tgtPType&LMP_FLOAT) && (tgtPType&LMP_LOG))
      HError(15430,"LoadLangModel: Incompatible probability kind requested: %d",tgtPType);
   if (InitSource(fn,&src,LangModFilter)!=SUCCESS)   /* Open LM file */
      HError(15410,"Unable to open language model file");
   if (trace&T_LOAD) {
      printf("Loading language model from %s\n", fn); fflush(stdout);
   }

   lm = (BackOffLM *) New(heap,sizeof(BackOffLM) * 2);
   lm->heap = heap;
   lm->htab = CreateHashTable(11731,"Back-off LM hash table");
   lm->gScale = gramScale;
   lm->fe_buff = NULL;
   lm->se_buff = NULL;
   lm->binMap  = NULL;
   lm->classH = NULL;
   lm->classLM = FALSE; /* default to not a class-based LM */
   lm->classBM = NULL;
   lm->classW = 0;
#ifdef HTK_CRYPT
   lm->encrypt = (src.crypt!=NULL);
#endif
   for (gi=lm->gInfo, i=1; i<LM_NSIZE; i++,gi++) {
      gi->nEntry = 0; gi->fmt = LMF_OTHER;
      gi->aInfo = NULL; gi->boInfo = NULL;
   }

   /* Have a look at the input file to see if it's a word|class count/probability
      file. If so it will link to the 'real' class language model, so load in
      these probabilities, and then continue to load the class gram counts from
      a standard language model as if it was the only original input. */

   /* Read first line from input LM file */
   GetInLine(&src, lnBuf);

   /* See if it's a multi-file class-based LM */
   if (strncmp(lnBuf, "Class-based LM", 14)==0) {
      /* Class-based LM */
      if (trace & T_LOAD) {
         printf("Loading a multi-file class-based language model\n");
      }

      /* Read filename of word|class probs/counts */
      GetInLine(&src, lnBuf);

      ptr = strchr(lnBuf, ':');
      if (!ptr) HError(15450, "LoadLangModel: Class language model file is in unknown format");
      ptr++;
      ptr += strspn(ptr, " \t");
      strcpy(wc_fname, ptr);

      /* Read filename of class|class bigrams */
      GetInLine(&src, lnBuf);
      ptr = strchr(lnBuf, ':');
      if (!ptr) HError(15450, "LoadLangModel: Class language model file is in unknown format");
      ptr++;
      ptr += strspn(ptr, " \t");
      /* NOTE: ptr content is used later on in this function to load in the class n-grams */

      /* Close input file (ignore anything left in the file) */
      CloseSource(&src);

      /* Load in word|class counts/probabilities file header */
      wcSrc.f = NULL; /* No existing file */
      ReadClassProbsHeader(wc_fname, &nWords, &wcSrc, lm);
      /* This sets lm->classCounts if it reads the appropriate header; otherwise probabilities */

      /* Allocate hash table for words */
      lm->classH = CreateHashTable((nWords/3)+1, "LM word/classes map");
      /* Allocate space for vocabulary map for words */
      lm->classBM = (NameId *) New(lm->heap, nWords*sizeof(NameId));
      lm->classBM--;  /* indexed from 1 (this is to make it work the same way as binMap) */
      /* This is really nasty so be careful if modifying code using classBM (or binMap) */

      /* This is a class-based LM (flag is toggled when backing off in GetNGramProb) */
      lm->classLM = TRUE;
      /* Store number of vocab words */
      lm->classW = nWords;

      /* We can either load probabilities or counts; counts require extra storage */
      if (lm->classCounts) {
         int j;
         /* Allocate word count storage space (totals allocated once we know #classes) */
         lm->word = New(&gcheap, nWords * sizeof(int));
         for (j=0; j<nWords; j++) {
            lm->word[j] = 0;
         }
      }

      /* Open class|class n-grams */
      if (InitSource(ptr, &src, LangModFilter)!=SUCCESS) /* ptr is n-gram file name */
         HError(15410, "LoadLangModel: Unable to open class|class n-gram language model file");
      if (trace&T_LOAD) {
         printf("Loading class n-grams from %s\n", ptr);
         fflush(stdout);
      }

      first_line = NULL; /* Read first line from class n-gram LM */
   }
   /* See if it's a single-file class LM */
   else if (strncmp(lnBuf, "CLASS MODEL", 11)==0) {
      if (trace & T_LOAD) {
         printf("Loading a class-based language model\n");
      }

      /* Load in word|class counts/probabilities header */
      wcSrc = src; /* Copy structure */
      ReadClassProbsHeader("", &nWords, &wcSrc, lm);
      /* This sets lm->classCounts if it reads the appropriate header; otherwise probabilities */

      /* Allocate hash table for words */
      lm->classH = CreateHashTable((nWords/3)+1, "LM word/classes map");
      /* Allocate space for vocabulary map for words */
      lm->classBM = (NameId *) New(lm->heap, nWords*sizeof(NameId));
      lm->classBM--;  /* indexed from 1 (this is to make it work the same way as binMap) */
      /* This is really nasty so be careful if modifying code using classBM (or binMap) */

      /* This is a class-based LM (flag is toggled when backing off in GetNGramProb) */
      lm->classLM = TRUE;
      /* Store number of vocab words */
      lm->classW = nWords;

      /* We can either load probabilities or counts; counts require extra storage */
      if (lm->classCounts) {
         int j;
         /* Allocate word count storage space (totals allocated once we know #classes) */
         lm->word = New(&gcheap, nWords * sizeof(int));
         for (j=0; j<nWords; j++) {
            lm->word[j] = 0;
         }
      }

      /* Open class|class n-grams */
      if (trace&T_LOAD) {
         printf("Reading class n-gram counts\n");
         fflush(stdout);
      }

      first_line = NULL; /* Read first line from current open file */
   }
   else {
      first_line = lnBuf; /* We've already read the first line */
   }

   ReadHeaderInfo(&src, lm, first_line);  /* First line of input is passed (or NULL) */

   if ((lm->probType&tgtPType)==0)
      HError(15430,"LoadLangModel: Unable to convert %d to %d pkind",
	     lm->probType,tgtPType);
   lm->probType &= tgtPType;

   isUltra = FALSE;
   for (gi=lm->gInfo+1, nSize=1; nSize<LM_NSIZE; nSize++,gi++) {
      sprintf(sfmt, "ngram %d%%c%%d", nSize);
      if (GetInLine(&src,lnBuf)==NULL)
	  HError(15450,"LoadLangModel: EOF whilst parsing n-gram info");
      if (sscanf(lnBuf, sfmt, &c, &n)==2) {
	 if (trace&T_LOAD)
	    printf("%s\n", lnBuf);
	 gi->nEntry = n;
	 switch (c) {
	    case '=': gi->fmt = LMF_TEXT;   break;
	    case '~': gi->fmt = LMF_BINARY; break;
	    case '#': gi->fmt = LMF_ULTRA;  isUltra = TRUE; break;
            default :
	       HError(15450,"LoadLangModel: Unknown LM file format (%s)",lnBuf);
	 }
      } else
	 break;
   }
   if (--nSize < 1)
      HError(15450, "LoadLangModel: Unable to identify file %s", fn);
   lm->nSize = nSize;

   /* initialise vocabulary size and lookup table */
   lm->vocSize = (wl==NULL) ? lm->gInfo[1].nEntry : wl->used;
   lm->binMap = (NameId *) New(lm->heap,(lm->vocSize)*sizeof(NameId));
   lm->binMap--;  /* indexed from 1 - beware if altering the code! This is really nasty! */
   if (wl!=NULL) {
      NameId wdid;
      if (isUltra)
	 HError(15440,"LoadLangModel: Cannot prune models in ultra format");
      itran = (int *) New(&gstack,(lm->gInfo[1].nEntry+1)*sizeof(int));
      for (i=1; i<=lm->vocSize; i++) {
	 wdid = GetNameId(lm->htab,wl->id[i-1]->name,TRUE);
	 lm->binMap[i] = wdid; LM_INDEX(wdid) = -i; /* assign negative indices */
      }
   } else {
      itran = NULL;
      for (i=1; i<=(lm->vocSize); i++) lm->binMap[i]=NULL;
   }

   if ((lm->vocSize > USHRT_MAX) && (sizeof(LM_Id)==sizeof(UShort)))
      HError(15445,"LoadLangModel: Unable to load %d unigrams using %d-byte IDs",
	     lm->vocSize,sizeof(LM_Id));
   /* initialise auxilliary structures */
   lm->lmvec = (float *) New(lm->heap,(lm->vocSize )*sizeof(float));
   lm->lmvec--;   /* indexed from 1 (hmmmmm) */
   lm->fe_buff = (FLEntry *) New(lm->heap,(lm->vocSize )*sizeof(FLEntry));
   lm->se_buff = (SMEntry *) New(lm->heap,(lm->vocSize )*sizeof(SMEntry));

   if (isUltra) {                        /* ultra file format */
#ifdef ULTRA_LM
      unsigned short key[KEY_LENGTH];

      if (strstr(lnBuf,"KEY: ")==NULL)
	 HError(15450,"LoadLangModel: Unable to find KEY (%s)",lnBuf);
      ultraKey[KEY_LENGTH-1] = (vaxOrder && natReadOrder) ? 1 : 0;
      for (strtok(lnBuf," "),i=0; i<KEY_LENGTH; i++) {
	 if ((s=strtok(NULL," "))==NULL)
	    HError(15450,"LoadLangModel: Unable to read key[%d] (%s)",i,lnBuf);
	 key[i] = strtol(s,(char **)NULL,16);
	 if (key[i]!=ultraKey[i])
	    HError(15450,"LoadLangModel: key[%d] mismatch %02x - should be %02x\n",
		   i, key[i], ultraKey[i]);
      }
      LoadNGram(&src,1,lm,NULL);
      LoadUltraNGrams(&src,lm);
#else
      HError(15490,"LoadLangModel: Ultra format LMs not supported");
#endif
   } else {                              /* text or binary file format */
      for (i=1; i<=nSize; i++)
	 lm->gInfo[i].nEntry = LoadNGram(&src,i,lm,itran);
   }
   if (itran!=NULL) Dispose(&gstack,itran);
   SyncStr(&src,"\\end\\");
   if (wcSrc.f != src.f) CloseSource(&src);

   for (i=1; i<lm->nSize; i++) {
      if (lm->gInfo[i].nEntry==0) {
	 HError(-15460,"LoadLangModel: Model order changed from %d-gram to %d-gram",
		lm->nSize,i-1);
	 lm->nSize=i-1; break;
      }
   }

   /* Build reverse look-up for use when recreating context from an FLEntry pointer */
   CreateReverseLookup(&(lm->root));

   if (lm->classLM) {
      if (lm->classCounts) {
         /* Load in given word|class count file(s) */
         if (trace & T_LOAD)
            printf("Loading word-in-class counts\n");
          ReadClassCounts(&wcSrc, nWords, lm);
          /* Allocate space for and count class totals for each LM */
          CountClassTotals(lm);
          /* Calculate static/initial word|class probabilities */
          CalcWordClassProbs(lm);
      }
      else {
         if (trace & T_LOAD)
            printf("Loading word-in-class probabilities\n");
         /* Load in word|class probabilities file */
         ReadClassProbs(&wcSrc, nWords, lm);
      }
      CloseSource(&wcSrc);
   }
   if (trace & T_LOAD)
      printf("Language model import complete (%d words; %s model)\n",
             lm->classW, lm->classLM?"class":"word");
   return lm;
}


/*------------------------- LM saving -------------------------*/

/* WriteNGram: recursive write routine */
static int WriteNGram(FILE *f, BackOffLM *lm, FLEntry **feStack,
		      int g, int nSize, Boolean intId)
{
   NGramInfo *gi;
   int i,j,ndx,nItem;
   SMEntry *se;
   FLEntry *fe,*topFE;
   Byte fsize,flags;
   float prob,bowt,iScale;
   Boolean has_bowt, isBin=FALSE;
   char *s, *word, context[MAXSYMLEN];
   LMProbType ptype;

   nItem = 0; iScale = 1.0/(lm->gScale*LN10);
   ptype = lm->probType;
   if (g < nSize) {
      topFE  = feStack[g-1];
      for (fe=topFE->fea, i=0; i<topFE->nfe; i++, fe++) {
	 feStack[g] = fe;
	 nItem += WriteNGram(f,lm,feStack,g+1,nSize,intId);
      }
   } else {
      gi = lm->gInfo+nSize;
      if (gi->fmt==LMF_BINARY || gi->fmt==LMF_TEXT)
	 isBin = (gi->fmt==LMF_BINARY);
      else
	 HError(15490,"LoadNGram: Unknown LM file format (%d)\n",gi->fmt);
      for (*context = '\0',s = context,j=1; j<nSize; j++) {
	 ndx = feStack[j]->ndx;
	 if ((ndx < 1) || (ndx > lm->vocSize))
	    HError(15490,"WriteNGram: Component %d of %d-gram, FE index (%d)",
		   j,nSize,ndx);
	 word = lm->binMap[ndx]->name;
	 if (htkEsc)
	    word = ReWriteString(word,NULL,ESCAPE_CHAR);
	 sprintf(s,"%s ",word); s+=strlen(s);
      }

      topFE = feStack[nSize-1];
      for (se = topFE->sea,i=0; i<topFE->nse; i++, se++)  {
	 if ((se->ndx < 1) || (se->ndx > lm->vocSize)) {
	    HError(15490,"WriteNGram: Invalid SE index (%d)",se->ndx);
	 }
	 switch (ptype) {
	    case LMP_FLOAT :
	       prob = FLT_TO_LOG10(se->prob); break;
	    case LMP_LOG :
#ifdef LM_COMPACT
	       prob = Shrt2Prob(se->prob); break;
#else
	       prob = se->prob * iScale; break;
#endif
	    default:
	       prob = se->prob; break;
	 }
	 if ((nSize < lm->nSize)   &&
	     (topFE->nfe>0)        &&
	     (fe = FindFE(topFE->fea,0,topFE->nfe,se->ndx))!=NULL) {
            /*            if (fe->nse>0) {*/
               has_bowt = TRUE;
               switch (ptype) {
	       case LMP_FLOAT :
	          bowt = FLT_TO_LOG10(fe->bowt); break;
	       case LMP_LOG :
	          bowt = fe->bowt * iScale; break;
	       default:
	          bowt = fe->bowt; break;
                  /* }*/
               }
	 } else {
	    has_bowt = FALSE;
	 }
	 if (isBin) {
	    flags = 0; fsize = sizeof(float);
	    if (has_bowt) {
	       flags |= HAS_BOWT; fsize += sizeof(float);
	    }
	    if (intId) {
	       fsize += nSize*sizeof(UInt);
	       flags |= INT_LMID;
	    } else {
	       fsize += nSize*sizeof(UShort);
	    }
	    fwrite(&fsize, sizeof(Byte),1,f);  /* size field */
	    fwrite(&flags, sizeof(Byte),1,f);  /* flags field */
	    WriteFloat(f,&prob,1,TRUE);        /* probability */
	    if (flags&INT_LMID) {
	       UInt x;
	       for (j=1; j<nSize; j++) {
		  x = (UInt) feStack[j]->ndx;
		  WriteInt(f,(int *)&x,1,TRUE);
	       }
	       x = (UInt) se->ndx;
	       WriteInt(f,(int *)&x,1,TRUE);
	    } else {
	       UShort x;
	       for (j=1; j<nSize; j++) {
		  x = (UShort) feStack[j]->ndx;
		  WriteShort(f,(short *)&x,1,TRUE);
	       }
	       x = (UShort) se->ndx;
	       WriteShort(f,(short *)&x,1,TRUE);
	    }
	    if (flags&HAS_BOWT)
	       WriteFloat(f,&bowt,1,TRUE);      /* back-off weight */
	 } else {
             fprintf(f, "%+.4f",prob);
	     fprintf(f, "\t%s",context);
	     word = lm->binMap[se->ndx]->name;
	     if (htkEsc)
		word = ReWriteString(word,NULL,ESCAPE_CHAR);
	     fprintf(f, "%s",word);
	     if (has_bowt)
		fprintf(f, "\t%+.4f",bowt);
	     fprintf(f, "\n");
	 }
	 nItem++;
      }
   }
   return nItem;
}

/* SaveNGram: write LM in to file f */
static int SaveNGram(FILE *f, int G, BackOffLM *lm)
{
   int total;
   Byte fsize;
   FLEntry *feStack[LM_NSIZE];
   Boolean useIntID;

   if (lm->vocSize > USHRT_MAX) {
      if (sizeof(LM_Id) <= sizeof(UShort))
	 HError(15445,"SaveNGram: vocSize = %d but using %d-byte IDs",
		lm->vocSize, sizeof(LM_Id));
      useIntID = TRUE;
   } else {
      useIntID = defIntID;
   }

   fprintf(f, "\n\\%d-grams:\n", G);
   feStack[0] = &(lm->root);
   total = WriteNGram(f,lm,feStack,1,G,useIntID);
   if (lm->gInfo[G].fmt==LMF_BINARY) {       /* write out 2 zero bytes */
      fsize = 0;
      fwrite(&fsize, sizeof(unsigned char), 1, f);
      fwrite(&fsize, sizeof(unsigned char), 1, f);
   }
   if (trace&T_SAVE)
      printf("Wrote %d %d-grams\n", total, G);
   return total;
}

/* SaveLangModel: save language model lm to fn */
void SaveLangModel(char *lmFn, BackOffLM *lm)
{
   char c=' ';
   int i,n;
   FILE *f;
   NGramInfo *gi;
   Boolean isPipe,isUltra;

#ifdef HTK_CRYPT
   if (lm->encrypt) {
      TMP_OPEN(f,lmFn,HError(15411,"SaveLangModel: Cannot create lm file %s",lmFn));
   }
   else
#endif
   if ((f = FOpen(lmFn, LangModOFilter, &isPipe))==NULL)
      HError(15411,"SaveLangModel: Unable to open output file %s",lmFn);
   WriteHeaderInfo(f,lm);
   fprintf(f, "\\data\\\n");
   isUltra = FALSE;
   for (gi=lm->gInfo+1,i=1; i<=lm->nSize; i++,gi++) {
      switch (gi->fmt) {
	 case LMF_TEXT:   c = '='; break;
	 case LMF_BINARY: c = '~'; break;
	 case LMF_ULTRA:  c = '#'; isUltra = TRUE; break;
	 default:
	    HError(15490,"SaveLangModel: Unknown LM file format (%d) for %d-gram",gi->fmt,i);
      }
      fprintf(f, "ngram %d%c%d\n",i,c,gi->nEntry);
   }
   if (isUltra) {
#ifdef ULTRA_LM
      ultraKey[KEY_LENGTH-1] = (vaxOrder && natWriteOrder) ? 1 : 0;
      fprintf(f,"KEY: ");
      for (i=0; i<KEY_LENGTH; i++) fprintf(f,"%02x ",ultraKey[i]);
      fprintf(f,"\n");
      SaveNGram(f,1,lm);
      SaveUltraNGrams(f,lm);
#else
      HError(15490,"SaveLangModel: Ultra format LMs not supported");
#endif
   } else {
      for (i=1; i<=lm->nSize; i++) {
	 if ((n=SaveNGram(f,i,lm))!=lm->gInfo[i].nEntry) {
	    HError(-15490,"SaveLangModel: %d-gram nEntry = %d, actual saved %d",
		   i,lm->gInfo[i].nEntry,n);
            lm->gInfo[i].nEntry = n;
         }
      }
   }
   fprintf(f, "\n\\end\\\n");
#ifdef HTK_CRYPT
   if (lm->encrypt) {
      FILE *crf;
      TMP_REWIND(f);
      if ((crf = FOpen(lmFn,LangModOFilter,&isPipe)) == NULL) {
	 TMP_CLOSE(f,lmFn);
	 HError(15411,"SaveLangModel: Cannot create LM file %s",lmFn);
      }
      EncryptFile(lmFn,crf,f);
      FClose(crf,isPipe);
      TMP_CLOSE(f,lmFn);
   }
   else
#endif
   FClose(f,isPipe);
}


/*---------------------- N-gram access ---------------------- */

/* EXPORT-> GetNGramProb: generic LM access V2 */
float GetNGramProb(BackOffLM *lm, NameId *words, int nSize)
{
   int i;
   float prob;
   SMEntry *se;
   FLEntry *fe;
   AccessInfo *acs;
   LMProbType ptype;
   char *s, sbuf[256];
   static int rLev = -1;
   float prob_mult = 0.0;

   /* NGram probability lookup works like this:
      1) We see if we're looking for a unigram and if so search for an
         appropriate leaf SMEntry at the root level. If we don't find
	 one then we must abort with an error at this point.
      2) For other lengths we search for the path down the tree to the
         FLEntry for the given history. If we don't find a full history
	 path we reduce the context and call ourselves recursively.
      3) If we found the context then we look at the SMEntry elements
         at the FLEntry node to see if we can find our word with the
	 given history. If we can then we return the stored probability
	 otherwise we recursively call ourselves again with a reduced
	 history, multiplying by the back-off weight associated with
	 the given history (at the FLEntry node) when we return.
   */

   /* If we're using a class-based language model then we still get passed
      a word history which must be converted into a class history */
   if (lm->classLM) {
      /* Retrieve word|class probability for word we want to predict */
      prob_mult = ((WordProb*)(words[nSize-1]->ptr))->prob;
      if (trace&T_PROB) {
         if (lm->probType & LMP_FLOAT) { /* this never happens in practice */
            printf("<w|c mult=%5.2f> ", UNLOG_NATURAL(prob_mult));
         }
         else {
            printf("<w|c mult=%5.2f> ", prob_mult);
         }
      }

      /* Convert word N-gram into class N-gram */
      for (i=0; i<nSize; i++) {
	 words[i] = ((WordProb*)(words[i]->ptr))->class;
      }
   }

   rLev++;
   ptype = lm->probType;
   if (nSize > lm->nSize) {
      words += nSize-lm->nSize; nSize = lm->nSize;
   }
   acs = lm->gInfo[nSize].aInfo; acs->count++;
   if (trace&T_PROB) {
      printf("[ ");
      printf("(%s",words[nSize-1]->name);
      if (nSize > 1) {
	 printf(" |");
	 for(i=0; i<nSize-1; i++) printf(" %s",words[i]->name);
      }
      printf(") ");
   }
   if (nSize==1) {  /* lookup unigram separately */
      if ((se = FindSE(lm->root.sea,0,lm->root.nse,LM_INDEX(words[0])))==NULL)
	 HError(15490,"GetNGramProb: Unable to find %s in unigrams",words[0]->name);
#ifdef LM_COMPACT
      prob = Shrt2Prob(se->prob) * lm->gScale;
#else
      prob = se->prob;
#endif
      if (trace&T_PROB)
	 printf("exact, ");
   } else {         /* generic n-gram lookup, n>1 */
      for (fe=&(lm->root), i=0; i<nSize-1; i++) {
	 if ((fe=FindFE(fe->fea, 0, fe->nfe, LM_INDEX(words[i])))==NULL)
	    break;
      }
      if ((fe == NULL) || (fe->nse == 0)) {
	 if (lm->classLM) {
            lm->classLM = FALSE;
            prob = GetNGramProb(lm,words+1,nSize-1);
            lm->classLM = TRUE;
	 }
	 else prob = GetNGramProb(lm,words+1,nSize-1);

	 if (trace&T_PROB)
	    printf("replaced, ");
	 acs->nmiss++;
	 if ((trace&T_TOP) &&  (fe != NULL) && (fe->nse == 0)) {
	    for (s = sbuf, i=0; i<nSize-1; i++) {
	       sprintf(s,"%s ",words[i]->name); s+=strlen(s);
	    }
	    HError(-15492, "GetNGramProb: FLEntry.nse==0; original ARPA LM?\n%s",sbuf);
	 }
      } else {
	 if ((se = FindSE(fe->sea, 0, fe->nse, LM_INDEX(words[nSize-1])))!=NULL) {
#ifdef LM_COMPACT
	    prob = Shrt2Prob(se->prob) * lm->gScale;
#else
	    prob = se->prob;
#endif
	    if (trace&T_PROB)
	       printf("exact, ");
	    acs->nhits++;
	 } else {
	    if (lm->classLM) {
               lm->classLM = FALSE;
               prob = GetNGramProb(lm,words+1,nSize-1);
               lm->classLM = TRUE;
	    }
	    else prob = GetNGramProb(lm,words+1,nSize-1);

	    if (ptype==LMP_FLOAT)
	       prob *= fe->bowt;
	    else
	       prob += fe->bowt;
	    if (trace&T_PROB)
	       printf("backed-off %.4f, ",fe->bowt);
	    acs->nboff++;
	 }
      }
   }
   if (lm->classLM)
   {
      if (lm->probType & LMP_FLOAT) {
         /* This looks nasty but in fact we never execute this */
         prob *= UNLOG_NATURAL(prob_mult);
      }
      else {
         prob += prob_mult;
      }
   }

   acs->prob += prob; acs->prob2 += prob*prob;
   if (trace&T_PROB)
      printf("prob %.4f ]%s",prob,(rLev==0) ? "\n" : " ");
   rLev--;

   return prob;
}


/* EXPORT-> LMTrans: calls GetNGramProb, but instead of taking a full
   n-gram of context we take a pointer to a context and a single word;
   we also return a langage model context state */
LogFloat LMTrans2(LModel *LM, LMState src, LabId word, LMState *dest)
{
   NameId ngram[LM_NSIZE], ngramRev[LM_NSIZE];
   int nSize;
   float prob;
   NameId nid;
   LogFloat prob_mult = 0.0;
   FLEntry *context, *fe;
   SMEntry *se;
   BackOffLM *lm;
   float bo_weight;
   LMProbType ptype;
   int i, index;
   int nShorten;  /* Amount to shorten n-gram by when searching for prob */

   lm = LM->data.hlmModel;
   ptype = lm->probType;

   if (src) {
      context = (FLEntry *) src;
   }
   else {
      context = &(lm->root); /* No context yet */
   }

   /* Convert word text to NameId */
   if (lm->classLM) { /* class model */
      nid = GetNameId(lm->classH, word->name, FALSE);
      if (!nid)
         HError(15499, "LMTrans: Attempt to predict token '%s' which is not in vocabulary", word);
      /* Find word-given-class probability and convert to a class */
      prob_mult = ((WordProb*)(nid->ptr))->prob;
      if (trace&T_PROB) {
         if (ptype & LMP_FLOAT) { /* this first never happens in practice */
            printf("<w|c mult=%5.2f> ", UNLOG_NATURAL(prob_mult));
         }
         else {
            printf("<w|c mult=%5.2f> ", prob_mult);
         }
      }
   }
   else { /* not a class model */
      nid = GetNameId(lm->htab, word->name, FALSE);
      if (!nid)
         HError(15499, "LMTrans: Attempt to predict token '%s' which is not in vocabulary", word);
   }

   /* We need to reconstruct the context later so do it now incase we need to back off */
   fe = context;
   nSize = 0;
   while (fe && fe!=&(lm->root) && nSize<LM_NSIZE) {
      ngramRev[nSize] = lm->binMap[fe->ndx];
      fe = fe->parent;
      nSize++;
   }
   if (nSize>=LM_NSIZE)
      HError(15499, "LMTrans: Context rebuilt to longer than compiled ngram size limit of %d", LM_NSIZE);
   /* And now we know the length we can reverse it */
   for (i=0; i<nSize; i++) ngram[i] = ngramRev[nSize-(i+1)];
   ngram[nSize] = nid;
   nSize++;


   /* For debugging purposes, print out the full ngram */
   /*printf("nsize=%d  ", nSize);
     for (i=0; i<nSize; i++) printf("%s ", ngram[i]->name); printf("\n");*/

   /* Search for probability */
   if (ptype & LMP_FLOAT)
      bo_weight = 1;
   else
      bo_weight = 0;
   se = FindSE(context->sea, 0, context->nse, LM_INDEX(nid));
   nShorten = 0;
   fe = context;
   while (!se) {
      /* Multiply BO weight and shorten context */
      if (ptype & LMP_FLOAT)
         bo_weight *= fe->bowt;
      else
         bo_weight += fe->bowt;
      nShorten++;
      if (nShorten==nSize) { /* Unigram probability */
         se = FindSE(lm->root.sea, 0, lm->root.nse, LM_INDEX(nid));
         if (!se)
            HError(15490, "LMTrans: Unable to find %s in unigrams", nid->name);
      }
      else { /* n>1 */
         fe = &(lm->root);
         for (i=nShorten; i<nSize-1; i++) {
            fe = FindFE(fe->fea, 0, fe->nfe, LM_INDEX(ngram[i]));
            if (!fe) HError(15491, "LMTrans: Unable to find shortened context in LM");
         }
         se = FindSE(fe->sea, 0, fe->nse, LM_INDEX(ngram[i]));
      }
   }
#ifdef LM_COMPACT
   prob = Shrt2Prob(se->prob) * lm->gScale;
#else
   prob = se->prob;
#endif
   if (ptype & LMP_FLOAT) {
      prob = prob * bo_weight;
   }
   else {
      prob = prob + bo_weight;
   }

   /* Now look for FLEntry for new context for any further following word */

   /* Decide from which point in the context we start searching */
   if (nSize == lm->nSize)
      index = 1;
   else
      index = 0;
   do {
      fe = &(lm->root);
      for (i=index; i<nSize; i++) {
         fe = FindFE(fe->fea, 0, fe->nfe, LM_INDEX(ngram[i]));
         if (!fe) {
            /* Context not found, so shorten and retry */
            index++;
            break;
         }
      }
   }
   while (!fe); /* Works because if no context then we don't execute inner loop and fe=&(lm->root) */

   *dest = fe;

   if (lm->classLM) {
      if (lm->probType & LMP_FLOAT) {
         /* This looks nasty but in fact it never executes in practice */
         prob *= UNLOG_NATURAL(prob_mult);
      }
      else {
         prob += prob_mult;
      }
   }

   return prob;
}

/* EXPORT-> GetNGramAddress: same as GetNGramProb but returns address
   of structure. This is used to provide a unique id for a particular
   context. This is used with Lattice Toolkit.

   The final word in words[] is a dummy entry which is never
   used.  Its value is undefined and should not be interpreted.
   (ie. words[nSize-1]).  It works like this in order to parallel 
   GetNGramProb() */
void *GetNGramAddress(BackOffLM *lm, NameId *words, int nSize)
{
   int i;
   FLEntry *fe;
   char *s, sbuf[256];
   static int rLev = -1;
   void *address = NULL; /* This is the return address */

   /* If we're using a class-based language model then we still get passed
      a word history which must be converted into a class history */
   if (lm->classLM) {
      /* Convert word N-gram into class N-gram */
      for (i=0; i<nSize-1; i++) {
	 words[i] = ((WordProb*)(words[i]->ptr))->class;
      }
   }

   rLev++;
   if (nSize > lm->nSize) {
      HError(-99999, "Context too large!");
      words += nSize - lm->nSize; nSize = lm->nSize;
   }


   if (nSize==1) {  /* unigram means no context */
      address = (void*) &(lm->root);
   } else {         /* generic n-gram lookup, n>1 */
      for (fe=&(lm->root), i=0; i<nSize-1; i++) {
	 if ((fe=FindFE(fe->fea, 0, fe->nfe, LM_INDEX(words[i])))==NULL)
            {address = fe; break;}
      }
      if ((fe == NULL) || (fe->nse == 0)) {
	 if (lm->classLM) {
            lm->classLM = FALSE;
            address = GetNGramAddress(lm,words+1,nSize-1);
            lm->classLM = TRUE;
	 }
	 else address = GetNGramAddress(lm,words+1,nSize-1);

	 if ((trace&T_TOP) &&  (fe != NULL) && (fe->nse == 0)) {
	    for (s = sbuf, i=0; i<nSize-1; i++) {
	       sprintf(s,"%s ",words[i]->name); s+=strlen(s);
	    }
	    HError(-15492, "GetNGramAddress: FLEntry.nse==0; original ARPA LM?\n%s",sbuf);
	 }
      }
      else {
         address = fe;
      }
   }

   rLev--;
   return address;
}

/* EXPORT-> GetNGramProbs: get probabilities for selected SEs */
void GetNGramProbs(BackOffLM *lm, UInt *nId, int nSize, SMEntry *seBuf, int seBufSize)
{
   int i;
   FLEntry *fe;
   SMEntry *se,*tse;
   LMProbType ptype;

   /* This fn only called from LPMerge and LPCalc when creating LMs, so this does
      not need to cater for class model w|c component */
   if (lm->classLM) {
      HError(15499, "GetNGramProbs: Language model is of unexpected class type");
   }

   ptype = lm->probType;
   if (ptype!=LMP_FLOAT && ptype!=LMP_LOG)
      HError(15430,"GetNGramProbs: Wrong probability kind (%d)",ptype);
   if (nSize>1)
      GetNGramProbs(lm,nId+1,nSize-1,seBuf,seBufSize);
   for (fe=&(lm->root), i=0; i<nSize-1; i++) {
      if ((fe=FindFE(fe->fea,0,fe->nfe,nId[i]))==NULL)
	 break;
   }
   if (fe!=NULL && fe->nse>0) {
      for (tse=seBuf,i=0; i<seBufSize; i++, tse++) {
	 if (tse->ndx < 1 || tse->ndx > lm->vocSize)
	    continue;
	 if ((se = FindSE(fe->sea,0,fe->nse,tse->ndx))!=NULL) {
#ifdef LM_COMPACT
	    tse->prob = Shrt2Prob(se->prob);
#else
	    tse->prob = se->prob;
#endif
	 } else {
	    if (ptype==LMP_FLOAT)
	       tse->prob *= fe->bowt;
	    else
	       tse->prob += fe->bowt;
	 }
      }
   }
}

/* EXPORT-> GetNGramProbVecSE: vector LM access */
float *GetNGramProbVecSE(BackOffLM *lm, int *nId, int nSize, FLEntry *parent)
{
   /*
      this function assumes that all requested ngrams can be found
      as explicit SEntries - ie. no backing-off is required.
   */
   FLEntry *fe;
   float *lmvec;
   int i,lo,ndx,cen;
   SMEntry *se,*sse,*unigram;

   if (lm->classLM) {
      /* Reach here via LPMerge.c - that means via LAdapt and LMerge in practice */
      HError(15498, "Sorry - you cannot apply this operation to full class-based models; you may however try again using only a class n-gram component");
   }

   lmvec = lm->lmvec;
   if (nSize==1) {    /* treat unigrams specially - lookup entries directly */
      unigram = lm->root.sea;
      for (sse=parent->sea, i=0; i<parent->nse; i++, sse++) {
	 if ((ndx = sse->ndx)==0)
	    continue;
#ifdef LM_COMPACT
	 lmvec[ndx] = Shrt2Prob(unigram[ndx-1].prob) * lm->gScale;
#else
	 lmvec[ndx] = unigram[ndx-1].prob;
#endif
      }
   } else {
      GetNGramProbVecSE(lm,nId+1,nSize-1,parent);  /* not needed */
      for (fe=&(lm->root), i=0; i<nSize-1; i++) {
	 if ((fe=FindFE(fe->fea,0,fe->nfe,nId[i]))==NULL)
	    break;
      }
      if (fe!=NULL && fe->nse>0) {
	 for (lo=0, sse=parent->sea, i=0; i<parent->nse; i++, sse++) {
	    ndx = sse->ndx;
	    if ((se = FindSE1(fe->sea,lo,fe->nse,ndx,&cen))!=NULL) {
#ifdef LM_COMPACT
	       lmvec[ndx] = Shrt2Prob(se->prob);
#else
	       lmvec[ndx] = se->prob;
#endif
	       lo = cen+1;
	    } else {
	       if (lm->probType==LMP_FLOAT)
		  lmvec[ndx] *= fe->bowt; /* not needed */
	       else
		  lmvec[ndx] += fe->bowt; /* not needed */
	    }
	 }
      }
   }
   return lmvec;
}

/* ---------------------- End of LModel.c  ---------------------- */
