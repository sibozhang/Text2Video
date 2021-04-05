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
/*      File: LWMap:  Word List and Word Map Routines          */
/* ----------------------------------------------------------- */

char *lwmap_version = "!HVER!LWMap:   3.4.1 [CUED 12/03/09]";
char *lwmap_vc_id = "$Id: LWMap.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"
#include "LWMap.h"

/* ------------------------ Trace Flags --------------------- */

static int trace = 0;
#define T_TOP   0001       /* top level tracing */
#define T_WMP   0002       /* word map loading */
#define T_SRT   0004       /* word map sorting */

/* --------------------- Global Variables ------------------- */

static ConfParam *cParm[MAXGLOBS];   /* config parameters */
static int nParm = 0;

static Boolean outWMapRaw = FALSE;   /* Output file in raw mode */
static Boolean inWMapRaw = FALSE;    /* Read input files in raw mode */

static LabId *sortTab;               /* global temp used by qsort */

/* --------------------- Initialisation --------------------- */

/* EXPORT -> InitWMap: initialise the module */
void InitWMap(void)
{
   int i;
   Boolean b;
   
   Register(lwmap_version,lwmap_vc_id);
   nParm = GetConfig("LWMAP", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"INWMAPRAW",&b)) inWMapRaw = b; 
      if (GetConfBool(cParm,nParm,"OUTWMAPRAW",&b)) outWMapRaw = b; 
   }
}

/* --------------------- Input Routines ------------------ */

/* CmpListEntry: qsort compare routine for word list sorting */
static int CmpListEntry(const void *p1, const void *p2)
{
  LabId *w1,*w2;
  w1 = (LabId *) p1;
  w2 = (LabId *) p2;
  return strcmp((*w1)->name,(*w2)->name);
}

/* EXPORT->GetSrcString: get next string from src in appropriate mode */
Boolean GetSrcString(Source *src, char *s, Boolean htkEsc)
{
   if (htkEsc) 
      return ReadString(src,s);
   else {
      src->wasQuoted=FALSE;
      return ReadRawString(src,s);
   }
}

/* ReadMapHeader: read header and store info in wm */
static void ReadMapHeader(Source *src, WordMap *wm)
{
   LMFileHdr hdr;
   LMHdrKind kind;
   int n;
   char *s;

   if (trace&T_TOP)
      printf("Reading header from %s\n",src->name);
   kind = ReadLMHeader(&(wm->mem),src,LWMapFilter,&hdr,&n);
   if (n<0 || n > 9999000)
      HError(15151,"ReadMapHeader: Unlikely num map entries[%d] in %s",n,src->name); 
   wm->used = n;
   wm->isMap = wm->hasCnts = FALSE;
   if (kind == NO_HDR) {
      wm->seqno = 0;
      wm->htkEsc = !inWMapRaw;
      wm->lang = NULL; wm->source = NULL;
   } else { 
      wm->name = GetLMHdrStr("NAME",hdr,FALSE);
      if (wm->name == NULL)
         HError(15152,"ReadMapHeader: No NAME header in %s",src->name);      
      if (!GetLMHdrInt("SEQNO",&n,hdr))
         HError(15153,"ReadMapHeader: No SEQNO header in %s",src->name);
      wm->seqno = n;
      wm->htkEsc = TRUE;
      if ((s=GetLMHdrStr("ESCMODE",hdr,TRUE)) != NULL) {
         if (strcmp(s,"HTK")==0) wm->htkEsc = TRUE; else
         if (strcmp(s,"RAW")==0) wm->htkEsc = FALSE; else
            HError(15155,"ReadMapHeader: Unknown escmode %s in %s",s,src->name);
      }
      wm->lang = GetLMHdrStr("LANGUAGE",hdr,FALSE);
      wm->source = GetLMHdrStr("SOURCE",hdr,FALSE);
      if ((s=GetLMHdrStr("FIELDS",hdr,TRUE)) != NULL) {
         if (strcmp(s,"ID")==0) wm->isMap = TRUE; else
         if (strcmp(s,"ID,WFC")==0 || strcmp(s,"WFC,ID")==0) 
            wm->isMap = wm->hasCnts = TRUE; 
         else
            HError(15150,"ReadMapHeader: Unknown fields tag %s in %s",s,src->name);
      }
   }
}

#define LTBNDX(ltb,i) (i - ltb->minId)

/* EXPORT->GetMEIndex: obtain index of ME corresponding to ndx */
int GetMEIndex(WordMap *wm, int ndx)
{
   LookupTable *ltb;

   ltb = (ndx < BASEWORDNDX) ? wm->clt : wm->wlt;
   return (ndx < ltb->minId || ndx > ltb->maxId) ? -1 : ltb->tlb[LTBNDX(ltb,ndx)];
}

/* EXPORT->BuildLookupTables: construct ndx -> ME lookup tables */
void BuildLookupTables(WordMap *wm)
{
   int i,j,freeSlots;
   MapEntry *me;
   LookupTable *wlt,*clt;

   if (!wm->isMap) {
      wm->wlt = NULL;
      wm->clt = NULL;
      return;
   }
   if (wm->me==NULL || wm->id==NULL)
      HError(15190,"BuildLookupTables: Map not initialised");

   freeSlots = wm->size - wm->used;
   if (wm->wlt!=NULL) {
      Dispose(&wm->mem,wm->wlt);
      wm->wlt = wm->clt = NULL;
   }

   /* build word lookup table first */
   wlt = (LookupTable *) New(&wm->mem,sizeof(LookupTable));
   wlt->minId = wlt->maxId = BASEWORDNDX; 
   for (me = wm->me,i=0; i<wm->used; i++,me++) {
      if (me->ndx >= BASEWORDNDX) {
	 wlt->minId = wlt->maxId = me->ndx; break;
      }
   }
   for (me=wm->me,i=0; i<wm->used; i++,me++) {
      if (me->ndx >= BASEWORDNDX) {
	 if (me->ndx < wlt->minId) wlt->minId = me->ndx;
	 if (me->ndx > wlt->maxId) wlt->maxId = me->ndx;
      }
   }
   wlt->maxId += freeSlots;
   wlt->size = wlt->maxId - wlt->minId + 1;
   wlt->tlb = (int *) New(&wm->mem,wlt->size*sizeof(int));
   for (i=0; i<wlt->size; i++) wlt->tlb[i]=-1;
   for (me=wm->me,i=0; i<wm->used; i++,me++) {
      if (me->ndx >= BASEWORDNDX) {
	 j = LTBNDX(wlt,me->ndx);
	 if (wlt->tlb[j]!=-1)
	    HError(15190,"BuildLookupTables: Duplicate word %s",wm->id[i]->name);
	 wlt->tlb[j] = i;
      }
   }
   wm->wlt = wlt;

   /* build class lookup table second */
   clt = (LookupTable *) New(&wm->mem,sizeof(LookupTable));
   clt->minId = clt->maxId = 0;
   for (me = wm->me,i=0; i<wm->used; i++,me++) {
      if (me->ndx < BASEWORDNDX) {
	 clt->minId = clt->maxId = me->ndx; break;
      }
   }
   for (me=wm->me,i=0; i<wm->used; i++,me++) {
      if (me->ndx < BASEWORDNDX) {
	 if (me->ndx < clt->minId) clt->minId = me->ndx;
	 if (me->ndx > clt->maxId) clt->maxId = me->ndx;
      }
   }
   clt->size = clt->maxId - clt->minId + 1;
   clt->tlb = (int *) New(&wm->mem,clt->size*sizeof(int));
   for (i=0; i<clt->size; i++) clt->tlb[i]=-1;
   for (me=wm->me,i=0; i<wm->used; i++,me++) {
      if (me->ndx < BASEWORDNDX) {
	 j = LTBNDX(clt,me->ndx);
	 if (clt->tlb[j]!=-1)
	    HError(15190,"BuildLookupTables: Duplicate class %s",wm->id[i]->name);
	 clt->tlb[j] = i;
      }
   }
   wm->clt = clt;
}

#define MAP_ENTRY(map,i) map.me + ((i<BASEWORDNDX) ? i : i-BASEWORDNDX+map.nClass)
   
/* LoadMapData: load contents of src into wordmap */
static void LoadMapData(Source *src, WordMap *wm)
{
   char buf[256],*fn;
   MapEntry *me;
   int ibuf[2],i,n,ndx;
   LabId id;

   fn = src->name;
   if (wm->isMap)
      for (i=0; i<wm->used; i++) wm->id[i] = NULL;
   for (me=wm->me,i=0; i<wm->used; i++,me++){
      if (!GetSrcString(src,buf,wm->htkEsc))
         HError(15113,"LoadMapData: Cannot read word %d from %s",i+1,fn);
      id = GetLabId(buf,TRUE);
      if (wm->isMap){
         if (id->aux != NULL)
            HError(15155,"LoadMapData: Word %s is duplicated in %s",buf,fn); 
         n = (wm->hasCnts) ? 2 : 1; ibuf[1] = 0;
         if (!ReadInt(src,ibuf,n,FALSE))
            HError(15113,"LoadMapData: Cannot read index/count of %s in %s",buf,fn);
	 if ((ndx = ibuf[0]) > wm->lastUsed)
	    wm->lastUsed = ndx;
	 id->aux = (Ptr) me; wm->id[i] = id;
         me->ndx = ndx; me->count = ibuf[1]; me->sort = 0;  
	 me->class = NULL;  me->aux.auxint = 0; 
      } else {
	wm->id[i] = id;
      }
   }
   CloseSource(src);
}

/* CreateListorMap: Create a word list or map structure */
static void CreateListorMap(char *fn, WordMap *wm, int freeSlots, Boolean isMap)
{
   Source src;
   char buf[MAXSTRLEN],buf2[MAXSTRLEN];
   
   /* Initialise configuration values */
   strcpy(buf,(isMap)?"WMap:":"WList:");
   if (fn != NULL) strcat(buf,NameOf(fn,buf2));
   CreateHeap(&(wm->mem),buf,MSTAK,1,0.0,100,10000000);
   wm->used = wm->seqno = 0;
   wm->name = wm->lang = wm->source = NULL;
   wm->htkEsc = TRUE; wm->isMap = wm->hasCnts = FALSE;
   wm->lastUsed = BASEWORDNDX-1;
   if (fn != NULL) {    /* read header */
      if (InitSource(fn,&src,LWMapFilter) == FAIL) {
         HError(15110, "File is required to continue");
      }
      ReadMapHeader(&src,wm);
      if (isMap != wm->isMap)
         HError(15150,"CreateListorMap: File %s is not a map",fn);                
      if (trace&T_WMP)
         printf("Creating map/list from %s with %d entries\n", fn, wm->used);
   } else {
      wm->isMap = isMap;
      if (trace&T_WMP) printf("Creating empty map/list\n");
   }
   /* Create Data Storage */
   wm->size = wm->used + freeSlots;
   wm->isSorted = FALSE;
   wm->firstNdx = -1;
   wm->id = (LabId *)New(&(wm->mem),wm->size*sizeof(LabId));
   wm->me = (wm->isMap)?(MapEntry *)New(&(wm->mem),wm->size*sizeof(MapEntry)):NULL;
   if (fn != NULL) {
      LoadMapData(&src,wm);
      if (trace&T_WMP) printf("%d words loaded\n",wm->used);
   }
   wm->wlt = wm->clt = NULL;
   BuildLookupTables(wm);
}

/* --------------------- Output Routines ---------------------- */

/* WriteMapHeader: write HLM header to f */
static void WriteMapHeader(FILE *f, WordMap *w)
{
   fprintf(f,"Name  = %s\n",w->name);
   fprintf(f,"SeqNo = %d\n",w->seqno);
   fprintf(f,"Entries = %d\n",w->used);
   fprintf(f,"EscMode  = %s\n",(w->htkEsc)?"HTK":"RAW");
   if (w->isMap)
      fprintf(f,"Fields  = ID%s\n",(w->hasCnts)?",WFC":"");
   if (w->lang != NULL)
      fprintf(f,"Language  = %s\n",w->lang);
   if (w->source != NULL)
      fprintf(f,"Source  = %s\n",w->source); 
   fprintf(f,"\\Words\\\n");
}

/* WriteWordMap: Write given word map to file f */
static void WriteWordMap(FILE *f, WordMap *w, Boolean noHeader, Boolean debug)
{
   int i;
   MapEntry *me;
   LabId id;
   
   if (w == NULL)
      HError(15190,"WriteWordMap: Word map is NULL");
   if (!noHeader) WriteMapHeader(f,w);
   for (i=0; i<w->used; i++){
      id = w->id[i];
      if (!w->htkEsc || outWMapRaw)
         fprintf(f, "%s", id->name);
      else
         WriteString(f,id->name,ESCAPE_CHAR);
      if (w->isMap) {
         me = &(w->me[i]);
         if (me == NULL || me != (MapEntry *)id->aux)
            HError(15190,"WriteWordMap: Map is broken");
         fprintf(f,"\t%d", me->ndx);
         if (w->hasCnts)
            fprintf(f,"\t%d", me->count);
         if (debug) 
            fprintf(f,"\t%d", me->sort);
      }
      fprintf(f,"\n");
   }
   if (debug) {
      fprintf(f,"Map is%s sorted\n", w->isSorted?"":" not");
      fprintf(f,"Used=%d, Size=%d\n", w->used, w->size);
      fprintf(f,"FirstNdx=%d, lastUsed=%d\n", w->firstNdx, w->lastUsed);
   }
}

/* ------------------- Interface Routines ------------------- */

/* EXPORT->CreateWordList: Create a word list */
void CreateWordList(char *fn, WordMap *w, int freeSlots)
{
   CreateListorMap(fn,w,freeSlots,FALSE);
   if (w->used>0) SortWordMap(w);
}

/* EXPORT->CreateWordMap: Create a word map */
void CreateWordMap(char *fn, WordMap *w, int freeSlots)
{
   CreateListorMap(fn,w,freeSlots,TRUE);
}

/* EXPORT->SaveWordMap: Write given word map to file fn */
void SaveWordMap(char *fn, WordMap *w, Boolean noHeader)
{
   FILE *f;
   Boolean isPipe;
   
   f = FOpen(fn,LWMapOFilter,&isPipe);
   if (f==NULL)
      HError(15111,"SaveWordMap: Cannot create %s",fn);
   WriteWordMap(f,w,noHeader,FALSE);
   FClose(f,isPipe);
}

/* EXPORT->ShowWordMap: Show given word map */
void ShowWordMap(WordMap *w)
{
   WriteWordMap(stdout,w,FALSE,TRUE);
}

/* EXPORT->AddWordToMap: add word to wm and return its id */
void AddWordToMap(WordMap *wm, LabId word)
{
   int i,ndx;
   MapEntry *me;
   
   if (wm==NULL)
      HError(15190,"AddWordToMap: Word map is NULL");
   if (!wm->isMap)
      HError(15190,"AddWordToMap: Cannot add word to word list");
   me = (MapEntry *) word->aux;
   if (me == NULL) {             /* new word */
      if (wm->used == wm->size)
         HError(15190,"AddWordToMap: Word map is full");
      i = wm->used++;
      wm->id[i] = word; word->aux = me = &(wm->me[i]);
      me->ndx = ndx = ++wm->lastUsed; me->sort = 0; me->count = 1; 
      me->class = NULL; me->aux.auxint = 0;
      wm->isSorted = FALSE;
      /* update the lookup table */
      wm->wlt->tlb[LTBNDX(wm->wlt,ndx)]=i;
   } else                        
      ++me->count;               /* seen this before */
}

/* EXPORT->MarkWordList: mark words in wl by setting me->aux.auxint to 1 */
void MarkWordList(WordMap *wl)
{
   int i;
   MapEntry *me;
   
   for (i=0; i<wl->used; i++) {
      me = (MapEntry *)wl->id[i]->aux;
      if (me == NULL) 
         HError(15190,"MarkWordList: Word %s is not in map",wl->id[i]->name);
      me->aux.auxint = 1;
   }
}

/* CmpMapEntry: qsort compare routine for map sorting */
static int CmpMapEntry(const void *p1, const void *p2)
{
  int b, *w1, *w2;

  w1 = (int *) p1;
  w2 = (int *) p2;
  b = strcmp(sortTab[*w1]->name, sortTab[*w2]->name);
  return b;
}

/* EXPORT->SortWordMap: sort the map */
void SortWordMap(WordMap *wm)
{
   int i,*smap;

   if (wm==NULL)
      HError(15190,"SortWordMap: Word map is NULL");
   if (wm->isSorted || wm->used==0) 
      return;
   if (trace&T_SRT)
      printf(" sorting %d entries in map\n", wm->used);
   if (!wm->isMap) {
      usort(wm->id,wm->used,sizeof(LabId),CmpListEntry);
   } else {
      sortTab = wm->id;
      smap = (int *) New(&gstack,wm->size*sizeof(int));
      for (i=0; i<wm->used; i++) smap[i] = i;
      usort(smap, wm->used, sizeof(int), CmpMapEntry);
      wm->firstNdx = smap[0];
      for (i=0; i<wm->used; i++) {
	 wm->me[smap[i]].sort = i;
	 if (trace&T_SRT) {
	    printf("%-12s %d\n",wm->id[i]->name,smap[i]);
	 }
      }
      Dispose(&gstack,smap);
   }
   wm->isSorted = TRUE;
}

/* -------------------  Access Routines ------------------- */

/* EXPORT->WordLMIndex: Returns word map index, -1 if not in map */
int WordLMIndex(LabId id)
{
   MapEntry *me;

   if ((id==NULL) || ((me = (MapEntry *)id->aux)==NULL))
      return -1;
   return me->ndx;
}

/* EXPORT->WordLMCount: Returns word map count */
int WordLMCount(LabId id)
{
   MapEntry *me;

   if (id == NULL)
      HError(15190,"WordLMCount: null id");
   if ((me = (MapEntry *)id->aux) == NULL)
      HError(15190,"WordLMCount: Word %s not in map",id->name);
   return me->count;
}

/* EXPORT->WordLMName: returns word given index */
LabId WordLMName(int ndx, WordMap *wm)
{
   int i;
   
   if (wm == NULL)
      HError(15190,"WordLMName: Word map is NULL");
   if ((i=GetMEIndex(wm,ndx)) < 0) 
      return NULL;
   return wm->id[i];
}

/*  EXPORT->WordLMCmp: true if word with ndx1 is < ndx2 */
int WordLMCmp(int ndx1, int ndx2, WordMap *wm)
{
   int i1,i2,s1,s2;
   
   if (wm == NULL)
      HError(15190,"WordLMCmp: Word map is NULL");
   if (!wm->isSorted)      
      HError(15190,"WordLMCmp: Map is not sorted");
   if ((i1 = GetMEIndex(wm,ndx1)) < 0)
      HError(15190,"WordLMCmp: Index %d not in map",ndx1);
   if ((i2 = GetMEIndex(wm,ndx2)) < 0)
      HError(15190,"WordLMCmp: Index %d not in map",ndx2);
   s1 = wm->me[i1].sort; s2 = wm->me[i2].sort;
   if (s1 < s2) return -1;
   if (s1 > s2) return +1;
   return 0;
}

/* -------------------- End of LWMap.c ---------------------- */
