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
/*         File: LGBase: Gram File Database Routines           */
/* ----------------------------------------------------------- */

char *lgbase_version = "!HVER!LGBase:   3.4.1 [CUED 12/03/09]";
char *lgbase_vc_id = "$Id: LGBase.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"
#include "LWMap.h"
#include "LGBase.h"

/* ------------------------ Trace Flags --------------------- */

static int trace = 0;
#define T_TOP   0001       /* top level tracing */
#define T_SQU   0002       /* trace squashing */
#define T_SRT   0004       /* trace NG Buffer sorting */
#define T_ITR   0010       /* print NG input set tree */
#define T_MOP   0020       /* print max parallel input streams */
#define T_IST   0040       /* trace parallel input streaming */
#define T_FOF   0100       /* print info on FoF i/o */

/* --------------------- Global Variables ------------------- */

static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;
static int sqOffset;                    /* squash offset, this depends on byte */
static Boolean checkOrder = FALSE;      /* Check n-gram ordering */
static Boolean natReadOrder = FALSE;    /* Preserve natural read byte order */
static Boolean natWriteOrder = FALSE;   /* Preserve natural write byte order */
extern Boolean vaxOrder;                /* True if byteswapping needed to preserve SUNSO */

/* --------------------- Initialisation --------------------- */

/* EXPORT -> InitGBase: initialise the module for n-grams */
void InitGBase(void)
{
   int i;
   Boolean b;

   Register(lgbase_version,lgbase_vc_id);
   /* get config variables for this module */
   nParm = GetConfig("LGBASE", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"NATURALREADORDER",&b)) natReadOrder = b;
      if (GetConfBool(cParm,nParm,"NATURALWRITEORDER",&b)) natWriteOrder = b;
      if (GetConfBool(cParm,nParm,"CHECKORDER",&b)) checkOrder = b;
   }
   /* Set byte order */
   sqOffset =  sizeof(UInt) - SQUASH;
   if (trace&T_SQU)  printf("Squash offset is %d\n",sqOffset);
}

/* SetNGInfo: init info struct for given N-gram */
static NGInfo SetNGInfo(int N)
{
   NGInfo i;

   i.N = N;
   i.ng_size = N*SQUASH + 1;
   i.ng_full = (N+1)*sizeof(UInt);
   return i;
}

/* ------------------- Squashing routines ---------------------- */

/* EXPORT->NGramSquash: compress each ngram to SQUASH bytes */
void NGramSquash(int N, NGram ng, Byte *comp)
{
   int i;
   UInt b;
   Byte *e,*c;
   Boolean mustSwap = (vaxOrder && !natWriteOrder);

   for (c = comp,i=0; i<N; i++, c+=SQUASH) {
      b = ng[i]; e = (Byte *) &b;
      if (mustSwap) SwapInt32((int *)&b);
      memcpy(c,e+sqOffset,SQUASH);
   }
}

/* EXPORT -> NGramExpand: expand ngrams from SQUASH num of bytes */
void NGramExpand(int N, Byte *comp, NGram ng)
{
   int i;
   UInt b;
   Byte *e,*c;
   Boolean mustSwap = (vaxOrder && !natReadOrder);

   for (c=comp,i=0; i<N; i++,c+=SQUASH){
      e = (Byte *) &b;
      memset(e,0x00,sizeof(UInt));
      memcpy(e+sqOffset,c,SQUASH);
      if (mustSwap) SwapInt32((int *)&b);
      ng[i] = b;
   }
   ng[N] = 0;
}

/* EXPORT -> SameGrams: true if grams (ignoring counts) are equal */
Boolean SameGrams(int N, NGram ng1, NGram ng2)
{
   int i;

   for (i=0; i<N; i++)
      if (ng1[i] != ng2[i]) return FALSE;
   return TRUE;
}

/* ------------------- NGram File Input/Output  --------------- */

/* EXPORT->PrintNGram: print given N-gram */
void PrintNGram(int N, NGram ng, WordMap *wm)
{
   int i;
   LabId id;

   for (i=0; i<N; i++) {
      id = WordLMName(ng[i],wm);
      printf("%-12s",id->name);
   }
   printf(" : %d\n",ng[N]);
}

/* LoadHGram: read text N-gram from header */
static void ReadHGram(char *name, LMFileHdr hdr, int N, LabId *ng, char *fn)
{
   int i;
   char *s,sbuf[MAXSTRLEN];

   if ((s=GetLMHdrStr(name,hdr,FALSE)) == NULL)
      HError(15350,"ReadHGram: No %s field in %s",name,fn);
   strcpy(sbuf,s);
   for (i=0; i<N; i++){
      s = strtok((i==0)?sbuf:NULL," \t\r\n");
      if (s==NULL)
	 HError(15350,"ReadHGram: Missing Sep in %s in %s",name,fn);
      ng[i] = GetLabId(s,TRUE);
   }
}

/* WriteHGram: write text N-gram to header */
static void WriteTxtHGram(FILE *f, char *name, int N, LabId *ng)
{
   int i;

   fprintf(f,"%s =",name);
   for (i=0; i<N; i++) {
      fprintf(f," %s",ng[i]->name);
   }
   fprintf(f,"\n");
}

/* WriteHGram: write a header for given NG Buffer */
void WriteRawHGram(FILE *f, char *name, int N, NGram ng, WordMap *wm)
{
   int i;
   LabId id;

   fprintf(f,"%s =",name);
   for (i=0; i<N; i++) {
      id = WordLMName(ng[i],wm);
      fprintf(f," %s",id->name);
   }
   fprintf(f,"\n");
}

/* SameHGrams: compare raw and text N-grams */
static Boolean SameHGrams(int N, NGram ng, LabId *tg)
{
   int i,ndx;

   for (i=0; i<N; i++) {
      if ((ndx = WordLMIndex(tg[i]))!=-1 && ndx!=ng[i])
	 return FALSE;
   }
   return TRUE;
}

/* CmpTxtNGram: compare two N-grams in text */
static int CmpTxtNGram(int N, LabId *ng1, LabId *ng2)
{
   int i,cmp;

   for (i=0; i<N; i++) {
      cmp = strcmp(ng1[i]->name,ng2[i]->name);
      if (cmp != 0) return cmp;
   }
   return 0;
}


/* CompareMapNames: compare map name and n-gram file map name */
static Boolean CompareMapNames(char *ngfMap, char *master)
{
  char *s;

  if (ngfMap==NULL || master==NULL)
     return FALSE;
  if ((s=strstr(master,ngfMap))==NULL)
     return FALSE;
  if (s!=master && *(s-1)!='%') /* not at the beginning and not preceeded by % */
     return FALSE;
  s += strlen(ngfMap);
  if (*s!='\0' && *s!='%')      /* not at the end and not followed by % */
     return FALSE;
  return TRUE;
}

/* SetNext: initialise ngs->nxt array with the first N-gram with
   all words in the map. */
static void SetNext(NGSource *ngs, Byte ngRawBuf[GSIZE])
{
   UInt *gp;
   int i, N, ng_size;
   Boolean same, hasOOM;

   N = ngs->info.N;
   ng_size = ngs->info.ng_size;
   while(ngs->nItems > 0) {
      memcpy(ngs->buf,ngRawBuf,ng_size);
      NGramExpand(N,ngs->buf,ngs->nxt);
      hasOOM = FALSE;
      for (gp=ngs->nxt,i=0; i<N; i++,gp++) {
	 if (GetMEIndex(ngs->wm,*gp) < 0) {
	    hasOOM = TRUE; break;
	 }
      }
      if (hasOOM) {  /* skip remaining N-grams, same as ngs->buf */
	 ngs->nItems--;
	 do {
	    if (fread(ngRawBuf,ng_size,1,ngs->src.f)==1) {
	       same = memcmp(ngs->buf,ngRawBuf,ng_size-1) == 0;
	    } else {
	       same = FALSE;
	    }
	 } while(same);
      } else {
	 break;
      }
   }
}

/* EXPORT->OpenNGramFile: open an ngram file and init NGSource */
void OpenNGramFile(NGSource *ngs, char *fn, WordMap *wm)
{
   LMFileHdr hdr;
   MemHeap mem;
   int i,n,N;
   char *s,buf[MAXSTRLEN];
   Byte ngRawBuf[GSIZE];
   UInt ngExpBuf[GSIZE];

   /* Create and Load Header */
   CreateHeap(&mem,"NGheader",MSTAK,1,0.0,1000,1000);
   if (InitSource(fn, &(ngs->src), LGramFilter) == FAIL)
      HError(15311,"OpenNGramFile: Can't open gram file '%s'", fn);
   if (ReadLMHeader(&mem, &(ngs->src), LGramFilter, &hdr, &n) != GRAM_HDR)
      HError(15350,"OpenNGramFile: Bad header in file %s",fn);
   ngs->nItems = n;
   /* Check Word map name and seqno */
   if ((s=GetLMHdrStr("WMAP",hdr,FALSE)) == NULL)
      HError(15350,"OpenNGramFile: No WMap field in %s",fn);
   if (!CompareMapNames(s,wm->name))
      HError(15330,"OpenNGramFile: Gram file map %s inconsistent with %s",
         s,wm->name);
   if (!GetLMHdrInt("SEQNO",&n,hdr))
      HError(15350,"OpenNGramFile: No SeqNo field in %s",fn);
   if (n > wm->seqno)
      HError(15330,"OpenNGramFile: SeqNo of map file is too low [%d vs %d]",
         n,wm->seqno);
   /* Check map matches WMCHECK */
   if ((s=GetLMHdrStr("WMCHECK",hdr,FALSE)) == NULL)
      HError(15350,"OpenNGramFile: No WMCheck field in %s",fn);
   strcpy(buf,s);
   if ((s=strchr(buf,' ')) == NULL)
      HError(15350,"OpenNGramFile: Missing Sep in WMCheck in %s",fn);
   *s = '\0'; n = atoi(s+1);
   if ((i=WordLMIndex(GetLabId(buf,FALSE)))!=-1 && i!=n)
      HError(15330,"OpenNGramFile: WMCheck FAILURE in %s, %d vs %d",fn,i,n);
   /* Ok, So Get Rest of Header Info */
   if (!GetLMHdrInt("NGRAM",&N,hdr))
      HError(15350,"OpenNGramFile: No Ngram field in %s",fn);
   ngs->info = SetNGInfo(N);
   s = GetLMHdrStr("SOURCE",hdr,FALSE);
   if (s==NULL) ngs->txtsrc[0] = '\0'; else strcpy(ngs->txtsrc,s);
   ReadHGram("GRAM1",hdr,N,ngs->firstGram,fn);
   ReadHGram("GRAMN",hdr,N,ngs->lastGram,fn);
   ngs->wm = wm;
   if (trace&T_TOP) {
      printf("Read Header for %s, [%d grams, size %d]\n",fn,ngs->nItems,N);
      fflush(stdout);
   }
   /* initialise the source by reading the first gram */
   if (fread(ngRawBuf,ngs->info.ng_size,1,ngs->src.f) !=1 )
      HError(15350, "OpenNGramFile: Empty file %s\n", fn);
   NGramExpand(N,ngRawBuf,ngExpBuf);
   if (!SameHGrams(N,ngExpBuf,ngs->firstGram)) {
      WriteTxtHGram(stdout,"Gram1",N,ngs->firstGram);
      WriteRawHGram(stdout,"gram1",N,ngExpBuf,wm);
      HError(15330, "OpenNGramFile: Header-specified 1st gram is not equal to the actual 1st gram in file %s\n", fn);
   }
   SetNext(ngs,ngRawBuf); /* This could well exhaust the file and reduce nItems to 0 */
   DeleteHeap(&mem);
}

/* EXPORT->CloseNGramFile: close given ngram file source */
void CloseNGramFile(NGSource *ngs)
{
   CloseSource(&(ngs->src));
}

/* EXPORT->ReadNGram: read the next ngram from given source.
   (The next ngram to read will already be in its buffer) */
void ReadNGram(NGSource *ngs, NGram ng)
{
   UInt a,oc,N,ng_size;
   Byte c,b[GSIZE];
   Boolean same;

   if (ngs->nItems <= 0)
      HError(15313,"ReadNGram: Gram file %s is empty",ngs->src.name);
   ngs->nItems--;
   oc = 0; a = 1; N = ngs->info.N;
   ng_size = ngs->info.ng_size;
   c = ngs->buf[ng_size-1];
   do {
      oc += a*c; a *= 256;
      if (fread(b, ng_size, 1, ngs->src.f)==1) {
         same = memcmp(ngs->buf, b, ng_size-1) == 0;
         c = b[ng_size-1];
      } else {
         same = FALSE;
      }
   } while (same);
   NGramExpand(N,ngs->buf,ng); ng[N] = oc;
   SetNext(ngs,b);
}

/* EXPORT -> WriteNGram: write compressed nGram to file f */
int WriteNGram(FILE *f, int N, NGram ng)
{
   Byte b;
   UInt a,c,bsize,count;
   static Byte buf[GSIZE];

   NGramSquash(N, ng,buf);
   bsize = N*SQUASH;
#ifdef LM_FLOAT_COUNT
   count = (UInt) *((float *)(ng + N))
#else
   count = ng[N];
#endif
   for (a=count,c=0; a != 0; a = a / 256, c++) {
      b = a % 256;
      fwrite(buf, bsize, 1, f);
      fwrite(&b, sizeof(Byte), 1, f);
   }
   return c;
}

/* --------------------- NGram Buffer Handling --------------- */

/* EXPORT->CreateNGBuffer: Create an N-gram buffer with size slots */
NGBuffer *CreateNGBuffer(MemHeap *mem, int N, int size, char *fn, WordMap *wm)
{
   NGBuffer *ngb;
   UInt poolbytes;

   ngb = (NGBuffer *)New(mem,sizeof(NGBuffer));
   ngb->info = SetNGInfo(N);
   ngb->poolsize = size; ngb->wm = wm;
   ngb->used = 0; ngb->fn = CopyString(mem,fn); ngb->fndx = 0;
   poolbytes = ngb->info.ng_full*size;
   ngb->next = ngb->pool = (UInt *) New(mem,poolbytes);
   return ngb;
}

/* EXPORT->StoreNGram: store ngram in buf into ngb, return TRUE if ngb is full */
Boolean StoreNGram(NGBuffer *ngb, NGram ng)
{
   memcpy(ngb->next, ng, ngb->info.ng_full);
   ngb->used++; ngb->next += ngb->info.N+1;
   return (ngb->used==ngb->poolsize);
}

/* CmpNGram: compare N-grams ng1 and ng2 using word map wm */
static int CmpNGram(WordMap *wm, int N, UInt *ng1, UInt *ng2)
{
   int i1,i2,j,s1,s2;

#ifdef SANITY
   if (wm == NULL)
      HError(15390,"WordLMCmp: Word map is NULL");
   if (!wm->isSorted)
      HError(15390,"WordLMCmp: Word map is not sorted");
#endif

    for (j=0; j<N; j++) {
      if ((i1 = GetMEIndex(wm,ng1[j])) < 0)
	 HError(15395,"WordLMCmp: Index %d not found in wordmap",ng1[j]);
      if ((i2 = GetMEIndex(wm,ng2[j])) < 0)
	 HError(15395,"WordLMCmp: Index %d not found in wordmap",ng2[j]);
      s1 = wm->me[i1].sort; s2 = wm->me[i2].sort;
      if (s1 < s2) return -1;
      if (s1 > s2) return +1;
   }
   return 0;
}

static int        qs_cmpSize;   /* must set before using this routine */
static WordMap    *qs_wmap;     /* word list to access mapentries */
static NGInputSet *qs_inset;    /* input set */

/* qs_CmpNGram: compare two N-grams, used in qsort */
static int qs_CmpNGram(const void *p1, const void *p2)
{
   return CmpNGram(qs_wmap,qs_cmpSize,(UInt *)p1,(UInt *)p2);
}

/* qs_CmpGFile: compare two NGSources on nxt field */
static int qs_CmpGFile(const void *p1, const void *p2)
{
   NGram p,q;
   int *i1, *i2;

   i1 = (int *)p1; i2 = (int *)p2;
   p = qs_inset->ngs[*i1].nxt;
   q = qs_inset->ngs[*i2].nxt;
   return CmpNGram(qs_inset->wm,qs_inset->N,p,q);
}

/* EXPORT->SortNGBuffer: sort+uniqe N-grams in ngb  */
void SortNGBuffer(NGBuffer *ngb)
{
   int i, count, isize, N;
   UInt *p, *q;
   char fn[256];

   if (trace&T_SRT) {
      sprintf(fn,"%s.%d",ngb->fn,ngb->fndx);
      printf(" Sorting %d N-grams (next write to %s)\n", ngb->used,fn);
   }
   SortWordMap(ngb->wm);
   qs_cmpSize = N = ngb->info.N; qs_wmap = ngb->wm;
   usort(ngb->pool,ngb->used,ngb->info.ng_full,qs_CmpNGram);
   p = ngb->pool; count = 1; isize = N + 1;
   for (q = ngb->pool + isize, i=1; i < ngb->used; i++, q += isize) {
      if (CmpNGram(ngb->wm,ngb->info.N,p,q)==0) {
#ifdef LM_FLOAT_COUNT
	pp = (float *) p+N;
	qq = (float *) p+N;
	*pp += *qq;
#else
	p[N] += q[N];
#endif
      } else {
         p += isize; count++;
         if (p != q) memcpy(p, q, ngb->info.ng_full);
      }
   }
   ngb->used = count; ngb->next = p+isize;
   if (trace&T_SRT) {
      printf(" N-grams sorted %d remaining\n", ngb->used);
      fflush(stdout);
   }
}

/* WriteNGHeader: write a header for given NG Buffer */
static void WriteNGHeader(FILE *f, NGBuffer *ngb, char *source)
{

   int N = ngb->info.N,chkndx;
   LabId chkid;

   if (ngb->used == 0)
      HError(15390,"WriteNGHeader: Ngram buffer is empty");
   fprintf(f,"NGram = %d\n",N);
   fprintf(f,"WMap  = %s\n",ngb->wm->name);
   fprintf(f,"SeqNo = %d\n",ngb->wm->seqno);
   fprintf(f,"Entries = %d\n",ngb->used);
   WriteRawHGram(f,"Gram1",N,ngb->pool,ngb->wm);
   WriteRawHGram(f,"GramN",N,ngb->next-(N+1),ngb->wm);
   chkid = ngb->wm->id[ngb->wm->used / 2];
   chkndx = WordLMIndex(chkid);
   fprintf(f,"WMCheck = %s %d\n",chkid->name,chkndx);
   if (source != NULL)
      fprintf(f,"Source = %s\n",source);
   fprintf(f,"\\Grams\\\n");
}

/* EXPORT->WriteNGBuffer: write ngb in compressed format to f */
void WriteNGBuffer(NGBuffer *ngb, char *source)
{
   int i, N;
   UInt *p;
   FILE *f;
   char fn[256];
   Boolean isPipe;

   N = ngb->info.N;
   sprintf(fn,"%s.%d",ngb->fn,ngb->fndx);
   f = FOpen(fn, LGramOFilter, &isPipe);
   WriteNGHeader(f,ngb,source);
   for (i=0,p = ngb->pool; i<ngb->used; i++, p += N + 1)
      WriteNGram(f, N, p);
   ngb->used = 0; ngb->next = ngb->pool; ++ngb->fndx;
   FClose(f,isPipe);
}

/* EXPORT->PrintNGBuffer: print given buffer */
void PrintNGBuffer(NGBuffer *ngb)
{
   UInt *p;
   int i,N;

   N = ngb->info.N;
   printf("NGram Buffer: out file %s.%d\n",ngb->fn,ngb->fndx);
   WriteNGHeader(stdout,ngb,NULL);
   for (i=0,p=ngb->pool; i<ngb->used; i++,p+=N+1)
      PrintNGram(N,p,ngb->wm);
   printf("%d entries\n",ngb->used);
}


/* ------------- Multiple N-Gram Input File Handling --------- */

/* ShowAbbrTxtGram: show ngram with fixed field width for each entry */
static void ShowAbbrTxtGram(int N, LabId *ng)
{
   int i;

   for (i=0; i<N; i++) {
      printf(" %-5.5s",(ng[i]==NULL)?"?????":ng[i]->name);
   }
}

/* ShowAbbrRawGram: show ngram with fixed field width for each entry */
static void ShowAbbrRawGram(int N, NGram ng, WordMap *wm)
{
   int i;
   LabId id;

   for (i=0; i<N; i++) {
      id = WordLMName(ng[i],wm);
      printf(" %-5.5s",(id==NULL)?"?????":id->name);
   }
}

#define LASTN(s,n) (strlen(s)>n) ? s+strlen(s)-n : s

/* ShowGFSons: print offspring gramfiles */
static void ShowGFSons(int N, GFLink gf, char * parent, WordMap *wm)
{
   GFLink p;

   if (gf==NULL)
      HError(15390,"ShowGFSons: Unexpected null gram file list");
   for (p=gf; p!=NULL; p=p->alt) {
      printf("%-12.12s %-12.12s  ",LASTN(p->fn,12),LASTN(parent,12));
      ShowAbbrTxtGram(N,p->firstGram); printf("-> ");
      ShowAbbrTxtGram(N,p->lastGram);  printf("\n");
      if (p->next != NULL)
         ShowGFSons(N, p->next,p->fn,wm);
   }
}

/* ShowInputSetTree: print input tree of ordering dependencies */
static void ShowInputSetTree(NGInputSet *inset)
{
   printf("Input Set Tree: %d files\n",inset->nFiles);
   printf("%-12s %-12s  %s\n","File","Parent",
           "   First N-gram     ->  Last N-gram");
   ShowGFSons(inset->N, inset->head.next,"Root",inset->wm);
}

/* EXPORT->CreateInputSet: create input file set */
void CreateInputSet(MemHeap *mem, WordMap *wm, NGInputSet *inset)
{
   int i;

   SortWordMap(wm);
   inset->mem = mem;
   inset->N = 0; inset->wm = wm;
   inset->nFiles = inset->nOpen = inset->maxNOpen = 0;
   inset->nextValid = FALSE;
   inset->head.next = inset->head.alt = inset->head.chain = NULL;
   for (i=0; i<MAXINF; i++) inset->gf[i] = NULL;
   /* make head gram file a sentinel */
   for (i=0; i<MAXNG; i++)
      inset->head.lastGram[i] = wm->id[wm->firstNdx];
}

/* EXPORT->AddInputGFile: add file fn to input set */
void AddInputGFile(NGInputSet *inset, char *fn, float weight)
{
   int i,N;
   GFLink p;
   NGSource ngs;

   OpenNGramFile(&ngs,fn,inset->wm);
   if (ngs.nItems > 0) {
      N = ngs.info.N;
      if (inset->N > 0) {
	 if (N != inset->N)
	    HError(15340,"AddInputGFile: File %s is %d-gram but inset is %d-gram",
		   fn,N,inset->N);
      } else {
	 inset->N = N;
      }
      p = (GFLink) New(inset->mem,sizeof(GramFile));
      p->alt = p->next = NULL; p->weight = weight;
      p->chain = inset->head.chain; inset->head.chain = p;
      strcpy(p->fn,fn);
      for (i=0; i<inset->N; i++) {
	 p->firstGram[i] = ngs.firstGram[i];
	 p->lastGram[i]  = ngs.lastGram[i];
      }
      ++inset->nFiles;
   }
   CloseNGramFile(&ngs);
}

/* ShowInputState: show current open input streams */
static void ShowInputState(char *mess, NGInputSet *inset)
{
   int i,j;

   printf("%s: %d files open\n",mess,inset->nOpen);
   for (i=0; i<inset->nOpen; i++) {
      j = inset->gfsort[i];
      printf("  %2d[%2d] %-10s",i,j,inset->gf[j]->fn);
      ShowAbbrRawGram(inset->N,inset->ngs[j].nxt,inset->wm);
      printf("\n");
   }
}

/* SortGFList: sort the list of open files into order of next
   available N-Gram. Sort order is defined by gfsort array */
static void SortGFList(NGInputSet *inset)
{
   qs_inset = inset;
   usort(inset->gfsort,inset->nOpen,sizeof(int),qs_CmpGFile);
   if (trace&T_SRT) ShowInputState("Full sort",inset);
}

/* ReSortGFList: resort after reading topmost N-Gram */
static void ReSortGFList(NGInputSet *inset)
{
   int i,j,n,this;
   NGram p,q;
   Boolean found = FALSE;

   n = inset->nOpen; this = inset->gfsort[0];
   p = inset->ngs[this].nxt;
   i = 1;
   while ( i<n && !found){
      q = inset->ngs[inset->gfsort[i]].nxt;
      found = (CmpNGram(inset->wm,inset->N,p,q) <= 0 );
      if (!found) ++i;
   }
   if (i==1) return;
   --i;
   /* i = index of new position for updated file */
   for (j=0; j<i; j++)
      inset->gfsort[j] = inset->gfsort[j+1];
   inset->gfsort[i] = this;
   if (trace&T_SRT) ShowInputState("Re-sorted",inset);
}

/* EXPORT->OpenInputSet: sort and open input streams */
void OpenInputSet(NGInputSet *inset)
{
   GFLink p,q,parent;

   for (p=inset->head.chain; p!=NULL; p=p->chain) {
      /* find parent of p */
      parent = &(inset->head);
      for (q=inset->head.chain; q!=NULL; q=q->chain) {
         if (CmpTxtNGram(inset->N,q->lastGram,parent->lastGram) > 0  &&
             CmpTxtNGram(inset->N,q->lastGram,p->firstGram) < 0 )
             parent = q;
      }
      /* link p to its parent */
      p->alt = parent->next; parent->next = p;
   }
   if (trace&T_ITR)
      ShowInputSetTree(inset);
   /* open all root file's offspring */
   for (p=inset->head.next; p!=NULL; p=p->alt,inset->nOpen++){
      if(inset->nOpen >= MAXINF)
         HError(15390,"OpenInputSet: max number of input gram files %d exceeded", MAXINF);
      OpenNGramFile(&(inset->ngs[inset->nOpen]),p->fn,inset->wm);
      inset->gf[inset->nOpen] = p;
      inset->gfsort[inset->nOpen] = inset->nOpen;
   }
   inset->maxNOpen = inset->nOpen;
   SortGFList(inset);
}

/* GetInsetGram: get next ngram and weight from the input set */
static void GetInsetGram(NGInputSet *inset, NGram ng, float *wt)
{
   NGSource *ngs;
   int i,cur;
   GFLink p,next,alt;
   UInt nextng[MAXNG];

   if (inset->nOpen == 0 )
      HError(15390,"GetInsetGram: No grams left");;
   cur = inset->gfsort[0];
   ngs = &(inset->ngs[cur]);
   ReadNGram(ngs,ng); *wt = inset->gf[cur]->weight;
   if (ngs->nItems == 0) { /* close this source and open all its successors */
      CloseNGramFile(ngs);
      if (trace&T_IST) printf(" closing file %s\n", inset->gf[cur]->fn);
      next = inset->gf[cur]->next;
      alt = (next == NULL) ? NULL : next->alt;
      if (next != NULL && alt == NULL) {  /* single successor */
         inset->gf[cur] = next;        /* so just replace it */
         OpenNGramFile(ngs,next->fn,inset->wm);
         if (trace&T_IST)
            printf(" replaced by file %s\n", next->fn);
      } else { /* zero or multiple successors */
         /* delete exhausted input stream */
         --inset->nOpen;
         for (i=cur; i<inset->nOpen; i++) {
            inset->ngs[i] = inset->ngs[i+1];
            inset->gf[i] = inset->gf[i+1];
         }
         for (i=0; i<inset->nOpen; i++) inset->gfsort[i] = i;
         if (next != NULL) {  /* add multiple successors */
            if (trace&T_IST)
               printf(" replaced by files ...\n");
            for (p=next; p!=NULL; p=p->alt,inset->nOpen++){
               OpenNGramFile(&(inset->ngs[inset->nOpen]),p->fn,inset->wm);
               inset->gf[inset->nOpen] = p;
               inset->gfsort[inset->nOpen] = inset->nOpen;
               if (trace&T_IST)
                  printf("   %s\n", p->fn);
            }
	    if (inset->maxNOpen < inset->nOpen)
	       inset->maxNOpen = inset->nOpen;
         }
      }
      if (inset->nOpen > 1) SortGFList(inset);
   } else         /* no change, just resort the parallel input streams */
      if (inset->nOpen > 1) ReSortGFList(inset);
   if (checkOrder && inset->nOpen > 0){ /* check ordering is consistent */
      cur = inset->gfsort[0];
      ngs = &(inset->ngs[cur]);
      NGramExpand(inset->N,ngs->buf,nextng);
      if (CmpNGram(inset->wm,inset->N,ng,nextng) > 0)
         HError(15345,"GetInsetGram: n-grams out of order");
   }
}

/* EXPORT->GetNextNGram: get next ngram from parallel input streams */
Boolean GetNextNGram(NGInputSet *inset, NGram ng, float *count, int N)
{
   float sum;
   UInt thisGram[MAXNG];
   int i;
   Boolean same;

   if (N > inset->N)
      HError(15341,"GetNextNGram: Requested N[%d] > gram size [%d]",N,inset->N);
   if(!inset->nextValid) {   /* either first or last */
      if (inset->nOpen == 0)
	 return FALSE;
      GetInsetGram(inset,inset->nextGram,&(inset->nextWt));
   }
   /* pick up the last read gram from this inset */
   for (i=0; i<N; i++)
      thisGram[i] = inset->nextGram[i];
   sum = inset->nextWt * inset->nextGram[inset->N];
   inset->nextValid = FALSE;

   /* read new grams whilst same as current */
   if (inset->nOpen > 0)
      do {
         GetInsetGram(inset,inset->nextGram,&(inset->nextWt));
         inset->nextValid = TRUE;
         same = TRUE;
         for (i=0; same && i<N; i++)
            same = thisGram[i] == inset->nextGram[i];
         if (same) {
            sum += inset->nextWt * inset->nextGram[inset->N];
            inset->nextValid = FALSE;
         }
      } while (same && inset->nOpen > 0);

   /* copy accumulated/truncated ngram to caller */
   for (i=0; i<N; i++) ng[i] = thisGram[i];
   *count = sum;
   return TRUE;
}

/* EXPORT->CloseInputSet: close input set */
void CloseInputSet(NGInputSet *inset)
{
   int i;

   if (trace&T_MOP)
      printf("Max parallel input streams = %d\n",inset->maxNOpen);
   for (i=0; i<inset->nOpen; i++)
      CloseNGramFile (&(inset->ngs[i]));
   inset->nOpen = 0;
}


/* ------------------- FoF Table Handling -------------- */

/* EXPORT-> Create a FoF table with size rows */
FoFTab *CreateFoFTab(MemHeap *mem, int size, int N)
{
   FoFTab *p;
   int i,j;
   UInt *u;

   p = (FoFTab *)New(mem,sizeof(FoFTab));
   p->size = size; p->N = N;
   p->fof = (UInt **)New(mem,sizeof(UInt *)*size);
   p->fof--;
   for (i=1; i<=size; i++){
      u = (UInt *)New(mem,sizeof(UInt)*N);
      --u; p->fof[i] = u;
   }
   for (i=1; i<=size; i++)
      for (j=1; j<=N; j++)
         p->fof[i][j] = 0;
   return p;
}


/* WriteFoFHeader: write a header for given NG Buffer */
static void WriteFoFHeader(FILE *f, FoFTab *tab, char *source)
{
   if (tab->size == 0)
      HError(15390,"WriteFoFHeader: FoF table is empty");
   fprintf(f,"NGram = %d\n",tab->N);
   fprintf(f,"Entries = %d\n",tab->size);
   if (source != NULL)
      fprintf(f,"Source = %s\n",source);
   fprintf(f,"\\Fofs\\\n");
}

/* EXPORT-> WriteFoFTab: Write given table to fn */
void WriteFoFTab(char *fn, FoFTab *tab, char *source)
{
   FILE *f;
   int i,j;

   f = (fn==NULL)?stdout:fopen(fn,"w");
   if (f==NULL)
      HError(15311,"WriteFoFTab: Can't create output file %s",
         (fn==NULL)?"stdout":fn);
   WriteFoFHeader(f,tab,source);
   for (i = 1; i<=tab->size; i++){
      for (j=1; j<=tab->N; j++)
         fprintf(f,"%10d ",tab->fof[i][j]);
      fprintf(f,"\n");
   }
   if (trace&T_FOF)
      printf("FoF written to file %s - N=%d,size=%d\n",
             (fn==NULL)?"stdout":fn,tab->N,tab->size);
   if (fn!=NULL)fclose(f);
}

/* EXPORT-> Create a FoF table holding contents of file fn */
FoFTab *ReadFoFTab(MemHeap *mem, char *fn)
{
   LMFileHdr hdr;
   MemHeap hmem;
   int ibuf[MAXNG],size,N,i,j;
   FoFTab *p;
   Source src;

   CreateHeap(&hmem,"FoFheader",MSTAK,1,0.0,1000,1000);
   if (InitSource(fn,&src,NoFilter)==FAIL) {
      HError(15310,"ReadFofTab: Require FoF file to continue");
   }
   if (ReadLMHeader(&hmem, &src, NoFilter, &hdr, &size) != LFOF_HDR)
      HError(15313,"ReadFoFTab: Bad header in file %s",fn);
   if (size<0 || size>=1000)
      HError(15313,"ReadFoFTab: Bad FoF table size %d in file %s",size,fn);
   if (!GetLMHdrInt("NGRAM",&N,hdr))
      HError(15313,"ReadFoFTab: No NGRAM field in %s",fn);
   if (N<0 || N>=MAXNG)
      HError(15313,"ReadFoFTab: Bad ngram size %d in %s",N, fn);
   p = CreateFoFTab(mem,size,N);
   for (i = 1; i<=size; i++) {
      if (!ReadInt(&src,ibuf,N,FALSE))
         HError(15313,"ReadFoFTab: Cannot read row %d from file %s",i,fn);
      for (j=1; j<=N; j++) p->fof[i][j] = ibuf[j-1];
   }
   if (trace&T_FOF)
      printf("FoF created from file %s - N=%d,size=%d\n",fn,N,size);
   CloseSource(&src);
   DeleteHeap(&hmem);
   return p;
}

/* -------------------- End of LGBase.c ---------------------- */
