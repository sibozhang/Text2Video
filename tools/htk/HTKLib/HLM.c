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
/*          2001-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HLM.c  language model handling                */
/* ----------------------------------------------------------- */

char *hlm_version = "!HVER!HLM:   3.4.1 [CUED 12/03/09]";
char *hlm_vc_id = "$Id: HLM.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "HLM.h"

/* --------------------------- Trace Flags ------------------------- */

#define T_TIO 1  /* Progress tracing whilst performing IO */

static int trace=0;

/* --------------------------- Initialisation ---------------------- */

#define LN10 2.30258509299404568 /* Defined to save recalculating it */

static Boolean rawMITFormat = FALSE;    /* Don't use HTK quoting and escapes */


static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

/* EXPORT->InitLM: initialise configuration parameters */
void InitLM(void)
{
   Boolean b;
   int i;

   Register(hlm_version,hlm_vc_id);
   nParm = GetConfig("HLM", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"RAWMITFORMAT",&b)) rawMITFormat = b;
   }
}

/*------------------------- Input Scanner ---------------------------*/

static Source source;           /* input file */

/* GetInLine: read a complete line from source */
static char *GetInLine(char *buf)
{
   int  i, c;

   c = GetCh(&source);
   if (c==EOF)
      return NULL;
   i = 0;
   while (c!='\n' && i<MAXSTRLEN) { 
      buf[i++] = c;
      c = GetCh(&source);
   } 
   buf[i] = '\0';
   return buf;
}

/* SyncStr: read input until str found */
static void SyncStr(char *buf,char *str)
{
   while (strcmp(buf, str)!=0) {
      if (GetInLine(buf)==NULL)
         HError(8150,"SyncStr: EOF searching for %s", str);
   }
}

/* GetInt: read int from input stream */
static int GetInt(void)
{
   int x;
   char buf[100];
   
   if (!ReadInt(&source,&x,1,FALSE))
      HError(8150,"GetInt: Int Expected at %s",SrcPosition(source,buf));
   return x;
}

/* GetFLoat: read float from input stream */
static float GetFloat(Boolean bin)
{
   float x;
   char buf[100];

   if (!ReadFloat(&source,&x,1,bin))
      HError(8150,"GetFloat: Float Expected at %s",SrcPosition(source,buf));
   return x;
}

/* ReadLMWord: read a string from input stream */
static char *ReadLMWord(char *buf)
{
   int i, c;
   
   if (rawMITFormat) {
      while (isspace(c=GetCh(&source)));
      i=0;
      while (!isspace(c) && c!=EOF && i<MAXSTRLEN){
         buf[i++] = c; c=GetCh(&source);
      }
      buf[i] = '\0';
      UnGetCh(c,&source);
      if (i>0)
         return buf;
      else
         return NULL;
   }
   else {
      if (ReadString(&source,buf))
         return buf;
      else
         return NULL;
   }
}

/*------------------------- NEntry handling ---------------------------*/

static int hvs[]= { 165902236, 220889002, 32510287, 117809592,
                    165902236, 220889002, 32510287, 117809592 };

/* EXPORT->GetNEntry: Access specific NGram entry indexed by ndx */
NEntry *GetNEntry(NGramLM *nglm,lmId ndx[NSIZE],Boolean create)
{
   NEntry *ne;
   unsigned int hash;
   int i;
   /* #define LM_HASH_CHECK */
  
   hash=0;
   for (i=0;i<NSIZE-1;i++)
      hash=hash+(ndx[i]*hvs[i]);
   hash=(hash>>7)&(nglm->hashsize-1);
  
   for (ne=nglm->hashtab[hash]; ne!=NULL; ne=ne->link) {
      if (ne->word[0]==ndx[0]
#if NSIZE > 2
          && ne->word[1]==ndx[1]
#endif
#if NSIZE > 3
          && ne->word[2]==ndx[2]
#endif
#if NSIZE > 4
          && ne->word[3]==ndx[3]
#endif
          )
         break;
   }

   if (ne==NULL && create) {
      ne=(NEntry *) New(nglm->heap,sizeof(NEntry));
      nglm->counts[0]++;
      
      for (i=0;i<NSIZE-1;i++)
         ne->word[i]=ndx[i];
      ne->user=0;
      ne->nse=0;
      ne->se=NULL;;
      ne->bowt=0.0;
      ne->link=nglm->hashtab[hash];
      nglm->hashtab[hash]=ne;
   }

   return(ne);
}

static int se_cmp(const void *v1,const void *v2)
{
   SEntry *s1,*s2;

   s1=(SEntry*)v1;s2=(SEntry*)v2;
   return((int)(s1->word-s2->word));
}

/*--------------------- ARPA-style NGrams ------------------------*/

static int nep_cmp(const void *v1,const void *v2)
{
   NEntry *n1,*n2;
   int res,i;

   res=0; n1=*((NEntry**)v1); n2=*((NEntry**)v2);
   for(i=NSIZE-2;i>=0;i--)
      if (n1->word[i]!=n2->word[i]) {
         res=(n1->word[i]-n2->word[i]);
         break;
      }
   return(res);
}


/* WriteNGram: Write n grams to file */
static int WriteNGrams(FILE *file,NGramLM *nglm,int n,float scale)
{
   NEntry *ne,*be,*ce,**neTab;
   SEntry *se;
   LogFloat prob;
   lmId ndx[NSIZE+1];
   int c,i,j,k,N,g=1,hash,neCnt,total;

   if (trace&T_TIO)
      printf("\nn%1d ",n),fflush(stdout);
   fprintf(file,"\n\\%d-grams:\n",n);
   N=VectorSize(nglm->unigrams);

   neTab=(NEntry **) New(&gstack,sizeof(NEntry*)*nglm->counts[0]);

   for (hash=neCnt=0;hash<nglm->hashsize;hash++)
      for (ne=nglm->hashtab[hash]; ne!=NULL; ne=ne->link) {
         for (i=1,ce=ne;i<n;i++)
            if (ne->word[i-1]==0) {
               ce=NULL;
               break;
            }
         if (ce!=NULL)
            for (i=n;i<NSIZE;i++)
               if (ne->word[i-1]!=0) {
                  ce=NULL;
                  break;
               }
         if (ce!=NULL && ce->nse>0)
            neTab[neCnt++]=ce;
      }
   qsort(neTab,neCnt,sizeof(NEntry*),nep_cmp);

   total=0;
   for (c=n;c<=NSIZE;c++) ndx[c]=0;
   for (j=0;j<neCnt;j++) {
      ne=neTab[j];
      for (c=1;c<n;c++) ndx[c]=ne->word[c-1];
      if (ne!=NULL && ne->nse>0) {
         for (i=0,se=ne->se;i<ne->nse;i++,se++) {
            if (trace&T_TIO) {
               if ((g%25000)==0)
                  printf(". "),fflush(stdout);
               if ((g%800000)==0)
                  printf("\n   "),fflush(stdout);
               g++;
            }
            ndx[0]=se->word;

            if (n<nglm->nsize) be=GetNEntry(nglm,ndx,FALSE);
            else be=NULL;
            if (be==NULL || be->nse==0) be=NULL;
            total++;
            if (n==1) prob=nglm->unigrams[se->word];
            else prob=se->prob;
            if (prob*scale<-99.999)
               fprintf(file,"%+6.3f",-99.999);
            else
               fprintf(file,"%+6.4f",prob*scale);
            c='\t';
            for (k=n-1;k>=0;k--)
               if (rawMITFormat)
                  fprintf(file,"%c%s",c,nglm->wdlist[ndx[k]]->name),c=' ';
               else
                  fprintf(file,"%c%s",c,
                          ReWriteString(nglm->wdlist[ndx[k]]->name,
                                        NULL,ESCAPE_CHAR)),c=' ';
            if (be!=NULL)
               fprintf(file,"\t%+6.4f\n",be->bowt*scale);
            else
               fprintf(file,"\n");
         }
      }

   }
   Dispose(&gstack,neTab);
   if (trace&T_TIO)
      printf("\n"),fflush(stdout);
   return(total);
}

#define PROGRESS(g) \
   if (trace&T_TIO) { \
      if ((g%25000)==0) \
         printf(". "),fflush(stdout); \
      if ((g%800000)==0) \
         printf("\n   "),fflush(stdout); \
   }


#define NGHSIZE1 8192
#define NGHSIZE2 32768
#define NGHSIZE3 131072

/* EXPORT->CreateBoNGram: Allocate and create basic NGram structures */
NGramLM *CreateBoNGram(LModel *lm,int vocSize, int counts[NSIZE])
{
   lmId ndx[NSIZE];
   int i,k;
   NGramLM *nglm;

   nglm = (NGramLM *) New(lm->heap, sizeof(NGramLM));
   lm->data.ngram = nglm;
   nglm->heap = lm->heap;

   for (i=0;i<=NSIZE;i++) nglm->counts[i]=0;
   for (i=1;i<=NSIZE;i++)
      if (counts[i]==0) break;
      else nglm->counts[i]=counts[i];
   nglm->nsize=i-1;

   /* Don't count final layer */
   for (k=0,i=1;i<nglm->nsize;i++) 
      k+=nglm->counts[i];
   /* Then use total to guess NEntry hash size */
   if (k<25000) 
      nglm->hashsize=NGHSIZE1;
   else if (k<250000) 
      nglm->hashsize=NGHSIZE2;
   else 
      nglm->hashsize=NGHSIZE3;

   nglm->hashtab=(NEntry **) New(lm->heap,sizeof(NEntry*)*nglm->hashsize);
   for (i=0; i<nglm->hashsize; i++) 
      nglm->hashtab[i]=NULL;

   nglm->vocSize = vocSize;
   nglm->unigrams = CreateVector(lm->heap,nglm->vocSize);
   nglm->wdlist = (LabId *) New(lm->heap,nglm->vocSize*sizeof(LabId)); nglm->wdlist--;
   for (i=1;i<=nglm->vocSize;i++) nglm->wdlist[i]=NULL;

   for (i=0;i<NSIZE;i++) ndx[i]=0;
   GetNEntry(nglm,ndx,TRUE);

   return(nglm);
}   

#define BIN_ARPA_HAS_BOWT 1
#define BIN_ARPA_INT_LMID 2

/* ReadNGrams: read n grams list from file */
static int ReadNGrams(NGramLM *nglm,int n,int count, Boolean bin)
{
   float prob;
   LabId wdid;
   SEntry *cse;
   char wd[255];
   lmId ndx[NSIZE+1];
   NEntry *ne,*le=NULL;
   int i, g, idx, total;
   unsigned char size, flags=0;

   cse = (SEntry *) New(nglm->heap,count*sizeof(SEntry));
   for (i=1;i<=NSIZE;i++) ndx[i]=0;

   if (trace&T_TIO)
      printf("\nn%1d ",n),fflush(stdout);

   total=0;
   for (g=1; g<=count; g++){
      PROGRESS(g);

      if (bin) {
         size = GetCh (&source);
         flags = GetCh (&source);
      }
      
      prob = GetFloat(bin)*LN10;

      if (n==1) { /* unigram treated as special */
         ReadLMWord(wd);
         wdid = GetLabId(wd, TRUE);
         if (wdid->aux != NULL)
            HError(8150,"ReadNGrams: Duplicate word (%s) in 1-gram list",
                   wdid->name);
         wdid->aux = (Ptr)g;
         nglm->wdlist[g] = wdid;
         nglm->unigrams[g] = prob;
         ndx[0]=g;
      } else {    /* bigram, trigram, etc. */
         for (i=0;i<n;i++) {
            if (bin) {
               if (flags & BIN_ARPA_INT_LMID) {
                  unsigned int ui;
                  if (!ReadInt (&source, (int *) &ui, 1, bin))
                     HError (9999, "ReadNGrams: failed reading int lm word id");
                  idx = ui;
               }
               else {
                  unsigned short us;
                  if (!ReadShort (&source, (short *) &us, 1, bin))
                     HError (9999, "ReadNGrams: failed reading short lm word id at");
                  idx = us;
               }
            }
            else {
               ReadLMWord(wd);
               wdid = GetLabId(wd, FALSE);
               idx = (wdid==NULL?0:(int)wdid->aux);
            }
            if (idx<1 || idx>nglm->vocSize)
               HError(8150,"ReadNGrams: Unseen word (%s) in %dGram",wd,n);
            ndx[n-1-i]=idx;
         }
      }

      total++;
      ne = GetNEntry(nglm,ndx+1,FALSE);
      if (ne == NULL)
         HError(8150,"ReadNGrams: Backoff weight not seen for %dth %dGram",g,n);
      if (ne!=le) {
         if (le != NULL && ne->se != NULL)
            HError(8150,"ReadNGrams: %dth %dGrams out of order",g,n);
         if (le != NULL) {
            if (le->nse==0) {
               le->se=NULL;
            } else {
               qsort(le->se,le->nse,sizeof(SEntry),se_cmp);
            }
         }
         ne->se = cse;
         ne->nse = 0;
         le = ne;
      }
      cse->prob = prob;
      cse->word = ndx[0];
      ne->nse++; cse++;

      /* read back-off weight */
      if (bin) {
         if (flags & BIN_ARPA_HAS_BOWT) {
            ne = GetNEntry(nglm,ndx,TRUE);
            ne->bowt = GetFloat (TRUE)*LN10;
         }
      }
      else {
         SkipWhiteSpace(&source);
         if (!source.wasNewline) {
            ne=GetNEntry(nglm,ndx,TRUE);
            ne->bowt = GetFloat(FALSE)*LN10;
         }
      }
   }

   /* deal with the last accumulated set */
   if (le != NULL) {
      if (le->nse==0) {
         le->se=NULL;
      } else {
         qsort(le->se,le->nse,sizeof(SEntry),se_cmp);
      }
   }

   if (trace&T_TIO)
      printf("\n"),fflush(stdout);

   return(total);
}

/* ReadBoNGram: read and store WSJ/DP format ngram */
static void ReadBoNGram(LModel *lm,char *fn)
{
   NGramLM *nglm;
   int i,j,k,counts[NSIZE+1];
   Boolean ngBin[NSIZE+1];
   char buf[MAXSTRLEN+1],syc[64];
   char ngFmtCh;

   if (trace&T_TIO)
      printf("\nBOffB "),fflush(stdout);

   if(InitSource(fn,&source,LangModFilter)<SUCCESS)
      HError(8110,"ReadBoNGram: Can't open file %s", fn);
   GetInLine(buf);
   SyncStr(buf,"\\data\\");
   for (i=1;i<=NSIZE;i++) counts[i]=0;
   for (i=1;i<=NSIZE;i++) {
      GetInLine(buf);
      if (sscanf(buf, "ngram %d%c%d", &j, &ngFmtCh, &k)!=3 && i>1)
         break;
      if (i!=j || k==0) 
         HError(8150,"ReadBoNGram: %dGram count missing (%s)",i,buf);

      switch (ngFmtCh) {
      case '=':
         ngBin[j] = FALSE;
         break;
      case '~':
         ngBin[j] = TRUE;
         break;
      default:
         HError (9999, "ReadARPALM: unknown ngram format type '%c'", ngFmtCh);
      }
      counts[j]=k;
   }

   if (ngBin[1])
      HError (8113, "ReadARPALM: unigram must be stored as text");

   nglm=CreateBoNGram(lm,counts[1],counts);
   for (i=1;i<=nglm->nsize;i++) {
      sprintf(syc,"\\%d-grams:",i);
      SyncStr(buf,syc);
      ReadNGrams(nglm,i,nglm->counts[i], ngBin[i]);
   }
   SyncStr(buf,"\\end\\");
   CloseSource(&source);

   if (trace&T_TIO) {
      printf("\n NEntry==%d ",nglm->counts[0]);
      for(i=1;i<=nglm->nsize;i++)
         printf(" %d-Grams==%d",i,nglm->counts[i]);
      printf("\n\n");
      fflush(stdout);
   }
}
/* WriteBoNGram: write out WSJ/DP format ngram */
static void WriteBoNGram(LModel *lm,char *fn,int flags)
{
   int i,k;
   FILE *file;
   NGramLM *nglm;
   Boolean isPipe;

   nglm = lm->data.ngram;
   file=FOpen(fn,LangModOFilter,&isPipe);
   fprintf(file,"\\data\\\n");

   for (i=1;i<=nglm->nsize;i++) {
      fprintf(file,"ngram %d=%d\n",i,nglm->counts[i]);
   }
   for (i=1;i<=nglm->nsize;i++) {
      k = WriteNGrams(file,nglm,i,1.0/LN10);
      if (k!=nglm->counts[i])
         HError(-8190,"WriteBoNGram: Counts disagree for %dgram (%d vs %d)",
                i, k, nglm->counts[i]);
   }
   fprintf(file,"\n\\end\\\n");
   FClose(file,isPipe);
}

void ClearBoNGram(LModel *lm)
{
   NGramLM *nglm = lm->data.ngram;
   int i;
   
   for(i=1;i<=nglm->vocSize;i++)
      if (nglm->wdlist[i]!=NULL) nglm->wdlist[i]->aux=0;
}

/* -------------- Matrix Bigram Handling Routines ----------- */

MatBiLM *CreateMatBigram(LModel *lm,int nw)
{
   MatBiLM *matbi;
  
   matbi = (MatBiLM *) New(lm->heap,sizeof(MatBiLM));
   lm->data.matbi = matbi;
   matbi->heap = lm->heap;
   
   matbi->numWords = nw;
   matbi->wdlist = (LabId *) New(lm->heap,sizeof(LabId)*(nw+1));
   matbi->bigMat = CreateMatrix(lm->heap,nw,nw);
   ZeroMatrix(matbi->bigMat);
   return(matbi);
}

/* ReadRow: read a row from bigram file f into v */
int ReadRow(Vector v)
{
   int i,j,N,cnt,c;
   float x;

   N = VectorSize(v);
   i=0; 
   while(!source.wasNewline) {
      x = GetFloat(FALSE);
      c=GetCh(&source);
      if (c == '*')
         cnt=GetInt();
      else {
         UnGetCh(c,&source);
         cnt=1;
      }
      SkipWhiteSpace(&source);
      for (j=0;j<cnt;j++) {
         i++;
         if (i<=N) v[i] = x;
      }
   }
   return(i);
}

/* ReadBigram: load a bigram from given file */
static void ReadMatBigram(LModel *lm,char *fn)
{
   Vector vec;
   char buf[132];
   int P,p,j;
   float sum,x;
   LabId id;
   MatBiLM *matbi;
  
   if (trace&T_TIO)
      printf("\nMB "),fflush(stdout);

   if(InitSource(fn,&source,LangModFilter)<SUCCESS)
      HError(8110,"ReadMatBigram: Can't open file %s", fn);
   vec = CreateVector(&gcheap,MAX_LMID);
   ReadLMWord(buf);SkipWhiteSpace(&source);
   id=GetLabId(buf,TRUE);
   P = ReadRow(vec);

   if (P<=0 || P >MAX_LMID)
      HError(8151,"ReadMatBigram: First row invalid (%d entries)",P);

   matbi=CreateMatBigram(lm,P);

   matbi->wdlist[1] = id;
   for (p=1;p<=P;p++) matbi->bigMat[1][p]=vec[p];
   id->aux=(Ptr) 1;
   Dispose(&gcheap,vec);

   for (sum=0.0, j=1; j<=P; j++) {
      x = matbi->bigMat[1][j];
      if (x<0)
         HError(8151,"ReadMatBigram: In bigram, entry %d for %s is -ve (%e)",
                j,buf,x);
      sum += x;
      matbi->bigMat[1][j]=((x<MINLARG)?LZERO:log(x));
   }
   if (sum < 0.99 || sum > 1.01)
      HError(-8151,"ReadMatBigram: Row %d of bigram %s adds up to %f",1,fn,sum);

   for (p=2; ReadLMWord(buf); p++) {
      if (trace&T_TIO) {
         if ((p%25)==0)
            printf(". "),fflush(stdout);
         if ((p%800)==0)
            printf("\n   "),fflush(stdout);
      }
      if (p>P)
         HError(8150,"ReadMatBigram: More rows than columns in bigram %s",fn);
      id=GetLabId(buf,TRUE);
      if ((int)id->aux != 0) 
         HError(8150,"ReadMatBigram: Duplicated name %s in bigram %s",buf,fn);
      id->aux = (Ptr) p;
      matbi->wdlist[p] = id;
      SkipWhiteSpace(&source);
      if (ReadRow(matbi->bigMat[p])!=P)
         HError(8150,"ReadMatBigram: Wrong number of items in row %d",p);
      for (sum=0.0, j=1; j<=P; j++) {
         x = matbi->bigMat[p][j];
         if (x<0)
            HError(8151,"ReadMatBigram: In bigram, entry %d for %s is -ve (%e)",
                   j,buf,x);
         sum += x;
         matbi->bigMat[p][j]=((x<MINLARG)?LZERO:log(x));
      }
      if (sum < 0.99 || sum > 1.01)
         HError(-8151,"ReadMatBigram: Row %d of bigram %s adds up to %f",p,fn,sum);
   }
   if (P>p)
      HError(8150,"ReadMatBigram: More columns than rows in bigram %s",fn);
   if (trace&T_TIO)
      printf("\n"),fflush(stdout);
   CloseSource(&source);
}

/* WriteMatBigram: write out old HVite format bigram */
static void WriteMatBigram(LModel *lm,char *fn,int flags)
{
   const float epsilon = 0.000001;
   MatBiLM *matbi;
   FILE *file;
   Boolean isPipe;
   Vector v;
   double x,y;
   int i,j,rep;

   if (trace&T_TIO)
      printf("\nMB "),fflush(stdout);

   matbi = lm->data.matbi;
   file=FOpen(fn,LangModOFilter,&isPipe);

   for (i=1;i<=matbi->numWords;i++) {
      if (trace&T_TIO) {
         if ((i%25)==0)
            printf(". "),fflush(stdout);
         if ((i%800)==0)
            printf("\n   "),fflush(stdout);
      }

      fprintf(file,"%-8s ",ReWriteString(matbi->wdlist[i]->name,
                                         NULL,ESCAPE_CHAR));

      v=matbi->bigMat[i];rep=0;x=-1.0;
      for (j=1;j<=matbi->numWords;j++){
         y = L2F(v[j]);
         if (fabs(y - x) <= epsilon) rep++;
         else {
            if (rep>0) {
               fprintf(file,"*%d",rep+1);
               rep=0;
            }
            x = y;
            if (x == 0.0)
               fprintf(file," 0");
            else if (x == 1.0)
               fprintf(file," 1");
            else
               fprintf(file," %e",x);
         }
      }
      if (rep>0)
         fprintf(file,"*%d",rep+1);
      fprintf(file,"\n");
   }
   FClose(file,isPipe);
   if (trace&T_TIO)
      printf("\n"),fflush(stdout);
}

/*------------------------- User Interface --------------------*/

/* EXPORT GetLMProb: return probability of word wd_id following pr_id[] */
float GetLMProb(LModel *lm, LabId prid[NSIZE], LabId wdid)
{
   LabId cpid[NSIZE];
   NEntry *ne;
   SEntry *se;
   lmId p, q, word, ndx[NSIZE];
   LogFloat bowt,prob;
   int i, s;
  
   switch (lm->type) {
   case boNGram:
      word = (int)wdid->aux;
      if (word==0 || word>lm->data.ngram->vocSize)
         return(LZERO);
      for (s=-1,i=0;i<NSIZE;i++)
         if (prid[i]!=NULL) 
            ndx[i]=(int)prid[i]->aux, cpid[i]=prid[i], s=i;
         else
            ndx[i]=0, cpid[i]=NULL;

      /* If no answer back-off to unigram */
      if (s<0) {
         if (word!=0)
            return(lm->data.ngram->unigrams[word]);
         else
            return(log(1.0/lm->data.ngram->vocSize));
      }

      cpid[s]=0;
      ne = GetNEntry(lm->data.ngram,ndx,FALSE);
      if (ne) {
         /* Replace with bsearch equivalent */
         for (i=0, se=ne->se; i<ne->nse; i++,se++)
            if (se->word==word) 
               return(se->prob); /* Ngram found */
         bowt=ne->bowt;
      }
      else {
         bowt=0.0;
      }
    
      if (s==0)
         return(lm->data.ngram->unigrams[word]+bowt); /* Backoff to unigram */
      else
         return(bowt+GetLMProb(lm,cpid,wdid)); /* else recurse */
      break;
   case matBigram:
      p=(int) prid[0]->aux;
      q=(int) wdid->aux;
      return(lm->data.matbi->bigMat[p][q]);
   default:
      prob=LZERO;
   }
   return(prob);
}

/* EXPORT ReadLModel: Determine LM type and then read-in */
LModel *ReadLModel(MemHeap *heap,char *fn)
{
   LModel *lm;
   LMType type;
   char buf[MAXSTRLEN+1];
   int i;

   lm=(LModel*)New(heap,sizeof(LModel));
   lm->heap=heap;
   lm->name=CopyString(heap,fn);

   if(InitSource(fn,&source,LangModFilter)<SUCCESS)
      HError(8110,"ReadLModel: Can't open file %s", fn);
   type=boNGram;i=0;
   do {
      if (i++==1000) {
         type=matBigram;
         break;
      }
      GetInLine(buf);
   }
   while (strcmp(buf, "\\data\\")!=0);
   CloseSource(&source);

   lm->type=type;
   switch(type) {
   case boNGram:
      ReadBoNGram(lm,fn);
      break;
   case matBigram:
      ReadMatBigram(lm,fn);
      break;
   }
   return(lm);
}


/* EXPORT WriteLModel: Determine LM type and then write-out */
void WriteLModel(LModel *lm,char *fn,int flags)
{
   switch(lm->type) {
   case boNGram:
      WriteBoNGram(lm,fn,flags);
      break;
   case matBigram:
      WriteMatBigram(lm,fn,flags);
      break;
   }
}

void ClearLModel(LModel *lm)
{
   switch(lm->type) {
   case boNGram:
      ClearBoNGram(lm);
      break;
   case matBigram:
      break;
   }
}

/*----------------------------------------------------------------------*/

#ifndef NO_LAT_LM
/* FindSEntry

     find SEntry for wordId in array using binary search
*/
static SEntry *FindSEntry (SEntry *se, lmId pronId, int l, int h)
{
   /*#### here l,h,c must be signed */
   int c;

   while (l <= h) {
      c = (l + h) / 2;
      if (se[c].word == pronId) 
         return &se[c];
      else if (se[c].word < pronId)
         l = c + 1;
      else
         h = c - 1;
   }

   return NULL;
}

/* LMTransProb_ngram

     return logprob of transition from src labelled word. Also return dest state.
     ngram case
*/
LogFloat LMTrans (LModel *lm, LMState src, LabId wdid, LMState *dest)
{
   NGramLM *nglm;
   LogFloat lmprob;
   lmId hist[NSIZE] = {0};      /* initialise whole array to zero! */
   int i, l;
   NEntry *ne;
   SEntry *se;
   lmId word;

   assert (lm->type == boNGram);
   nglm = lm->data.ngram;

   word = (int) wdid->aux;

   if (word==0 || word>lm->data.ngram->vocSize) {
      HError (-9999, "word %d not in LM wordlist", word);
      *dest = NULL;
      return (LZERO);
   }

   ne = src;
   
   if (!src) {          /* unigram case */
      lmprob = nglm->unigrams[word];
   }
   else {
      /* lookup prob p(word | src) */
      /* try to find pronid in SEntry array */
      se = FindSEntry (ne->se, word, 0, ne->nse - 1);

      assert (!se || (se->word == word));

      if (se)        /* found */
         lmprob = se->prob;
      else {             /* not found */
         lmprob = 0.0;
         l = 0;
         hist[NSIZE-1] = 0;
         for (i = 0; i < NSIZE-1; ++i) {
            hist[i] = ne->word[i];
            if (hist[i] != 0)
               l = i;
         } /* l is now the index of the last (oldest) non zero element */
         
         for ( ; l > 0; --l) {
            if (ne)
               lmprob += ne->bowt;
            hist[l] = 0;   /* back-off: discard oldest word */
            ne = GetNEntry (nglm, hist, FALSE);
            if (ne) {   /* skip over non existing hists. fix for weird LMs */
               /* try to find pronid in SEntry array */
               se = FindSEntry (ne->se, word, 0, ne->nse - 1);
               assert (!se || (se->word == word));
               if (se) { /* found it */
                  lmprob += se->prob;
                  l = -1;
                  break;
               }
            }
         }
         if (l == 0) {          /* backed-off all the way to unigram */
            assert (!se);
            lmprob += ne->bowt;
            lmprob += nglm->unigrams[word];
         }
      }
   }


   /* now determine dest state */
   if (src) {
      ne = (NEntry *) src;
      
      l = 0;
      hist[NSIZE-1] = 0;
      for (i = 1; i < NSIZE-1; ++i) {
         hist[i] = ne->word[i-1];
         if (hist[i] != 0)
            l = i;
      } /* l is now the index of the last (oldest) non zero element */
   }
   else {
      for (i = 1; i < NSIZE-1; ++i)
         hist[i] = 0;
      l = 1;
   }

   hist[0] = word;

   ne = (LMState) GetNEntry (nglm, hist, FALSE);
   for ( ; !ne && (l > 0); --l) {
      hist[l] = 0;              /* back off */
      ne = (LMState) GetNEntry (nglm, hist, FALSE);
   }
   /* if we left the loop because l=0, then ne is still NULL, which is what we want */

   *dest = ne;

#if 0
   printf ("lmprob = %f  dest %p\n", lmprob, *dest);
#endif

   return (lmprob);
}
#endif


/* ------------------------- End of HLM.c ------------------------- */
