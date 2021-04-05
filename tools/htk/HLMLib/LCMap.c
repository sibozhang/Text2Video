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
/*      File: LCMap:  Class Map Routines                       */
/* ----------------------------------------------------------- */

char *lcmap_version = "!HVER!LCMap:   3.4.1 [CUED 12/03/09]";
char *lcmap_vc_id = "$Id: LCMap.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"
#include "LWMap.h"
#include "LCMap.h"

/* ------------------------ Trace Flags --------------------- */

static int trace = 0;
#define T_TOP   0001       /* top level tracing */
#define T_CML   0002       /* class map loading */

/* --------------------- Global Variables ------------------- */

static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

static Boolean outCMapRaw = FALSE;       /* Output file in raw mode */
static Boolean inCMapRaw = FALSE;        /* Read input files in raw mode */

static int  unkndx  = DEF_UNKNOWNID;         /* ndx of (hdrless) unk class */
static LabId unkid  = NULL;	             /* name of (hdrless) unk class */
static char  unkStr[256] = DEF_UNKNOWNNAME;  /* name of (hdrless) unk class */

/* --------------------- Initialisation --------------------- */

/* EXPORT -> InitCMap: initialise the module */
void InitCMap(void)
{
   int i;
   Boolean b;
   char buf[100];

   Register(lcmap_version,lcmap_vc_id);
   nParm = GetConfig("LCMAP", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"INCMAPRAW",&b)) inCMapRaw = b; 
      if (GetConfBool(cParm,nParm,"OUTCMAPRAW",&b)) outCMapRaw = b; 
      if (GetConfInt(cParm,nParm,"UNKNOWNID",&i)) unkndx = i;
      if (GetConfStr(cParm,nParm,"UNKNOWNNAME",buf)) strcpy(unkStr,buf);
   }
   unkid = GetLabId(unkStr,TRUE);
}

/* --------------------- Input Routines ------------------ */

/* ReadMapHeader: read header and store info in cm */
static void ReadMapHeader(Source *src, ClassMap *cm, int *entries)
{
   LMFileHdr hdr;
   LMHdrKind kind;
   int n;
   char *s;

   if (trace&T_TOP)
      printf("Reading header from %s\n",src->name);
   kind = ReadLMHeader(&(cm->mem), src, LCMapFilter, &hdr, &n);
   if (n<0 || n > 999000)
      HError(15050,"ReadMapHeader: Unlikely num map entries[%d] in %s",n,src->name); 
   *entries = n;
   cm->entries = 0;  /* will be incremented as classes are read */
   if (kind == NO_HDR) {
      if (unkndx == -1 || unkid == NULL)
         HError(15051,"ReadMapHeader: UNKxxx configs must be set for hdrless map");
      cm->hdrless = TRUE;
      cm->htkEsc = !inCMapRaw; cm->lang = NULL;
   } else { 
      cm->name = GetLMHdrStr("NAME",hdr,FALSE);
      if (cm->name == NULL)
         HError(15052,"ReadMapHeader: No name in %s",src->name);      
      cm->htkEsc = TRUE;
      if ((s=GetLMHdrStr("ESCMODE",hdr,TRUE)) != NULL) {
         if (strcmp(s,"HTK")==0) cm->htkEsc = TRUE; else
         if (strcmp(s,"RAW")==0) cm->htkEsc = FALSE; else
            HError(15053,"ReadMapHeader: Unknown escmode %s in %s",s,src->name);
      }
      cm->lang = GetLMHdrStr("LANGUAGE",hdr,FALSE);
   }
}
   
/* SetClassWords: add given word to specified class */
static void SetClassWords(ClassEntry *ce, WordMap *wmap, int nItem, LabId *cword)
{
   int i;
   LabId id=0;
   MapEntry *me;
   ClassEntry *tce;

   ce->size = 0;
   if (ce->inClass) {
      for (i=0; i<nItem; i++) {
	 id = cword[i];
	 me = (MapEntry *)id->aux;
	 if (!me) {
	    if (trace&T_TOP) {
	       printf("  Adding word %-12s from class %s to map\n",
		      id->name,ce->id->name);
	    }
	    AddWordToMap(wmap,id);
	    me = (MapEntry *)id->aux;
	 }
	 tce = (ClassEntry *)me->class;
	 if (tce)
	    HError(15090,"SetClassWords: word %s already in class %s",
		   id->name,ce->id->name);
	 ++ce->size;
	 me->class = ce;
      }
   } else {
      for (me=wmap->me,i=0; i<wmap->used; i++,me++) 
	 me->aux.auxint = 0;
      for (i=0; i<nItem; i++) {
	 id = cword[i];
	 me = (MapEntry *)id->aux;
         if((void*)ce == (void*)me){
            HError(15058, "SetClassWords: word %s cannot be in the list of words excluded from class %s",
                   id->name, ce->id->name);
         }
	 if (!me) {
	    if (trace&T_TOP) {
	       printf("  Adding word %-12s from class %s to map\n",
		      id->name,ce->id->name);
	    }
	    AddWordToMap(wmap,id); me = (MapEntry *)id->aux;
	 }
	 me->aux.auxint = 1;
      }
      for (me=wmap->me,i=0; i<wmap->used; i++,me++) {
	 if (me->aux.auxint == 0) {
	    tce = (ClassEntry *) me->class;
	    if (tce)
	       HError(15090,"SetClassWords: word %s already in class %s",
		      id->name,ce->id->name);
	    me->class = ce;
	    ce->size++;
	 } else {
	    me->aux.auxint=0;
	 }
      }
   }
}


/* LoadClass: load and store a single class from src */
static void LoadClass(ClassMap *cm, Source *src, LabId clname, int clndx, 
                 int clsize, Boolean inClass)
{
   int i,nItem;
   LabId *cword;
   ClassEntry *ce;
   char buf[MAXSTRLEN];

   if (trace&T_CML) 
      printf("  loading class %s[%d] from %s\n",clname->name,clsize,src->name);
   ce = MakeNewClass(cm,clname,clndx,inClass);
   nItem = 0;
   cword = (LabId *) New(&gstack,clsize*sizeof(LabId));
   for (i=0; i<clsize; i++) {
      if (!GetSrcString(src,buf,cm->htkEsc))
	 HError(15013,"LoadClass: Can't read word %d from %s",i+1,src->name);
      cword[nItem++] = GetLabId(buf,TRUE); 
   }
   SetClassWords(ce,cm->wmap,nItem,cword);
   if (trace&T_CML) 
      printf("   class %s loaded\n",clname->name);
   Dispose(&gstack,cword);
}

/* LoadMapData: load contents of src into classmap */
static void LoadMapData(Source *src, ClassMap *cm, int entries)
{
   char buf[MAXSTRLEN],*fn;
   LabId id;
   int i,ibuf[2],ndx,nents;
   Boolean inClass=FALSE;

   fn = src->name;
   if (cm->hdrless) {
      cm->name = CopyString(&cm->mem,fn);
      if ((ClassEntry *)unkid->aux)
         HError(-15092, "LoadMapData: %s already in map (index=%d)", unkid->name, ((ClassEntry*)unkid->aux)->ndx);
      else
         LoadClass(cm,src,unkid,unkndx,entries,FALSE);
   } else {
      for (i=0; i<entries; i++){
	 if (!GetSrcString(src,buf,cm->htkEsc))
	    HError(15013,"LoadMapData: Can't read class name %d from %s",i+1,fn);
	 id = GetLabId(buf,TRUE);
	 if (id->aux != NULL)
	    HError(15054,"LoadMapData: Class name %s duplicate in %s",buf,fn); 
	 if (!ReadInt(src,ibuf,2,FALSE))
	    HError(15013,"LoadMapData: Can't read index for class %s in %s",buf,fn);
	 ndx = ibuf[0];
	 if (ndx > 0  && ndx < BASEWORDNDX) {
	    if (cm->maxClndx < ndx) cm->maxClndx = ndx;
	 } else
	    HError(15055,"LoadMapData: Bad index %d for class %s in %s",ndx,buf,fn);
         nents = ibuf[1];
	 if (nents<1)
	    HError(15056,"LoadMapData: Number of entries = %d for class %s in %s",nents,buf,fn);
	 if (!GetSrcString(src,buf,cm->htkEsc))
	    HError(15013,"LoadMapData: Can't read type for class %s in %s",id->name,fn);
         if (strcmp(buf,"IN") == 0 )
	    inClass = TRUE;
         else if (strcmp(buf,"NOTIN") == 0 )
            inClass = FALSE;
         else
	    HError(15057,"LoadMapData: Bad type %s for class %s in %s",buf,id->name,fn);
	 if (nents>=1)
	   LoadClass(cm,src,id,ndx,nents,inClass);
      }
   }
   CloseSource(src);
}


/* --------------------- Output Routines ---------------------- */

/* WriteMapHeader: write class header to f */
static void WriteMapHeader(FILE *f, ClassMap *cm)
{
   fprintf(f,"Name  = %s\n",cm->name);
   fprintf(f,"Entries = %d\n",cm->entries);
   fprintf(f,"EscMode  = %s\n",(cm->htkEsc)?"HTK":"RAW");
   if (cm->lang != NULL)
      fprintf(f,"Language  = %s\n",cm->lang);
   fprintf(f,"\\Classes\\\n");
}

/* OutString: output given id in appropriate esc mode to f */
static void OutString(FILE *f, Boolean htkEsc, LabId id)
{
   if (!htkEsc || outCMapRaw)
      fprintf(f, "%s", id->name);
   else 
      WriteString(f,id->name,ESCAPE_CHAR);
}

/* WriteClassMap: Write given class map to file f */
static void WriteClassMap(FILE *f, ClassMap *cm, Boolean debug)
{
   ClassEntry *ce;
   int i,h;
   int *ndxlist,len,maxlen=0,avelen=0;


   if (cm == NULL)
      HError(15092,"WriteClassMap: Class map is NULL");
   WriteMapHeader(f,cm);
   for (h=0; h<CLMHASHSIZE; h++) {
      len = 0;
      for (ce = cm->htab[h]; ce != NULL; ce = ce->next ){
         ++len;
	 OutString(f,cm->htkEsc,ce->id);
	 fprintf(f," %d %d %s\n", ce->ndx, ce->size, (ce->inClass)?"IN":"NOTIN");
         if (ce->size > 0) {
	    ndxlist = (int *)New(&gstack,sizeof(int)*ce->size);
	    GetClassMembers(cm,ce->ndx, ndxlist);
	    for (i=0; i<ce->size; i++) {
	       OutString(f,cm->htkEsc,WordLMName(ndxlist[i], cm->wmap));
	       fprintf(f,"\n");
	    }
	    Dispose(&gstack,ndxlist);
	 }
      }
      if (len>maxlen) maxlen = len;
      avelen += len;
   }
   if (debug) {
      avelen /= CLMHASHSIZE;
      fprintf(f,"Entries=%d, maxClndx=%d, nfree=%d\n", 
                 cm->entries, cm->maxClndx, cm->nfree);
      fprintf(f,"Max Hash length=%d, Average Hash length=%d\n", 
                 maxlen,avelen);
   }
}


/* ------------------- Interface Routines ------------------- */

/* EXPORT->CreateClassMap: create class map and load fn if non-null */
void CreateClassMap(char *fn, ClassMap *c, WordMap *w)
{
   Source src;
   char buf[MAXSTRLEN],buf2[MAXSTRLEN];
   int h,n;
   
   /* Initialise configuration values */
   strcpy(buf,"CMap:");
   if (fn != NULL) strcat(buf,NameOf(fn,buf2));
   CreateHeap(&(c->mem),buf,MSTAK,1,0.5,1000,100000);
   c->entries = 0; c->hdrless = FALSE;
   c->name = c->lang = NULL;
   c->maxClndx = 0; c->wmap = w;
   c->flist = NULL; c->nfree = 0;
   for (h=0; h<CLMHASHSIZE; h++) c->htab[h] = NULL;
   if (fn != NULL) {    /* read header */
      if (InitSource(fn,&src,LCMapFilter)!=SUCCESS) {
	 HError(15010,"CreateClassMap: Cannot open file %s", fn);
      }
      ReadMapHeader(&src,c,&n);
      if (trace&T_CML)
         printf("Creating class map from %s with %d entries\n", fn, n);
   } else {
      if (trace&T_CML) printf("Creating empty class map\n");
   }
   if (fn != NULL) {
      LoadMapData(&src,c,n);
      if (trace&T_CML) printf("%d classes loaded\n",c->entries);
   }
}

/* EXPORT->SaveClassMap: Write given class map to file fn. */
void SaveClassMap(char *fn, ClassMap *c)
{
   FILE *f;
   Boolean isPipe;
   
   f = FOpen(fn,LCMapOFilter,&isPipe);
   if (f==NULL)
      HError(15011,"SaveClassMap: Cannot create %s",fn);
   WriteClassMap(f, c, FALSE);
   FClose(f,isPipe);
}


/* EXPORT->ShowClassMap: Write given class map to file fn. */
void ShowClassMap(ClassMap *c)
{
   WriteClassMap(stdout,c,TRUE);
}


/* EXPORT->MakeNewClass: create new class and return index */
ClassEntry *MakeNewClass(ClassMap *c, LabId id, int clndx, Boolean inClass)
{  
   int h;
   ClassEntry *ce;

   ce = (ClassEntry *)id->aux;
   if (ce != NULL) 
      HError(15092,"MakeNewClass: %s already in map (index=%d)",id->name,ce->ndx);
   /* determine ndx of new class */
   if (clndx < 0)
      clndx = ++c->maxClndx;
   else {
      if (clndx > c->maxClndx) c->maxClndx = clndx;
   }
   /* create the class record */
   if (c->nfree == 0)  /* freelist empty so make new one */
      ce = (ClassEntry *)New(&(c->mem),sizeof(ClassEntry));
   else { 	       /* reuse deleted class entry */
      ce = c->flist; c->flist = ce->next; --c->nfree;
   }
   ce->id = id; ce->ndx = clndx; ce->size = 0; ce->inClass = inClass;

   /* link it to hashtable and the class name */
   id->aux = ce;  /* NB. This is overwritten in LGCopy.c - beware! */

   h = clndx % CLMHASHSIZE;
   ce->next = c->htab[h]; c->htab[h] = ce;
   ++c->entries;
   return ce;
}

/* EXPORT->DeleteClass: delete the given class */
void DeleteClass(ClassMap *c, int clndx)
{
   int h,i,nwords=0;
   ClassEntry *pce, *ce, *delce;
   MapEntry *me;

   if (c->entries == 0) 
      HError(15092,"DeleteClass: Class map %s is empty",c->name);
   /* find class record */
   h = clndx % CLMHASHSIZE;
   if (c->htab[h]->ndx == clndx) {
      delce = c->htab[h]; c->htab[h] = delce->next;
   } else {
      pce = c->htab[h]; delce = NULL;
      for (ce = pce->next; ce != NULL; ce = ce->next) {
	if (ce->ndx == clndx) {delce = ce; break;}
        pce = ce;
      }
      if (delce == NULL)
         HError(15092,"DeleteClass: Class %d not in map %s",clndx,c->name);
      pce->next =  delce->next;
   }
   /* delete it and add to freelist */
   ++c->nfree; --c->entries; delce->next = c->flist; c->flist = delce;
   delce->id->aux = NULL;
   /* make sure no words reference it */
   for (me=c->wmap->me,i=0; i<c->wmap->used; i++,me++) 
      if ((ClassEntry *)me->class == delce)
	 { me->class = NULL; ++nwords;} 
   if (nwords != delce->size)
      HError(15092,"DeleteClass: Found %d words in class of %d words",nwords,delce->size);
}

/* EXPORT->AddWordToClass: add given word to specified class */
void AddWordToClass(ClassMap *c, int clndx, int wdndx)
{
   LabId id;
   MapEntry *me;
   ClassEntry *ce;

   id = WordLMName(wdndx,c->wmap);
   if ((me = (MapEntry *)id->aux) == NULL)
      HError(15090,"AddWordToClass: Word %d has no map entry",wdndx);
   if ((ce = (ClassEntry *)me->class) != NULL)
      HError(15090,"AddWordToClass: Word %d already in class %d",wdndx,ce->ndx);
   ce = GetClassEntry(c,clndx);
   ++ce->size;
   me->class = ce;
}

/* EXPORT->RemWordFromClass: delete given word from class */
void RemWordFromClass(ClassMap *c, int clndx, int wdndx)
{
   LabId id;
   MapEntry *me;
   ClassEntry *ce;

   id = WordLMName(wdndx,c->wmap);
   me = (MapEntry *)id->aux;
   if (me == NULL)
      HError(15090,"RemWordFromClass: Word %d has no map entry",wdndx);
   ce = GetWordClassEntry(c,wdndx);
   if (ce == NULL)
      HError(15090,"RemWordFromClass: Word %d has no class",wdndx);
   if  ((ClassEntry *)me->class != ce)
      HError(15090,"RemWordFromClass: Word %d not in class %d",wdndx,clndx);;
   if (--ce->size < 0) 
      HError(15092,"RemWordFromClass: Class %d has negative size",clndx);
   me->class = NULL;
}

/* EXPORT->WordClass: return class of  word, -1 if no class found */
int WordClass(ClassMap *c, int wdndx)
{
   ClassEntry *ce;

   ce = GetWordClassEntry(c,wdndx);
   if (ce == NULL) return -1;
   return ce->ndx;
}

/* EXPORT->ClassSize: return size of class, -1 if class not found */
int ClassSize(ClassMap *c, int clndx)
{
   ClassEntry *ce;

   ce = GetClassEntry(c,clndx);
   if (ce == NULL)
      HError(15092,"ClassSize: %d is not a class index",clndx);
   return ce->size;
}

/* EXPORT->GetClassMembers: copy class members into words.
   Indexing is [0..Classsize(c,clndx)-1] */
void GetClassMembers(ClassMap *c, int clndx, int *words)
{
   ClassEntry *ce;
   MapEntry *me;
   int i,j=0,nfound=0;

   ce = GetClassEntry(c,clndx);
   for (me=c->wmap->me,i=0; i<c->wmap->used; i++,me++) 
      if ((ClassEntry *)me->class == ce){
	  ++nfound; words[j++] = me->ndx;
      }
   if (ce->size != nfound) 
      HError(15092,"GetClassMembers: %d words found in class of size %d",nfound,ce->size);
}

/* EXPORT->IsAnInClass: true if class is a regular 'inClass' set */
Boolean IsAnInClass(ClassMap *c, int clndx)
{
   ClassEntry *ce;

   ce = GetClassEntry(c,clndx);
   if (ce == NULL)
      HError(15092,"IsAnInClass: %d is not a class index",clndx);
   return ce->inClass;
}

/* EXPORT->IsClassMember: true if word is in class */
Boolean IsClassMember(ClassMap *c, int clndx, int wdndx)
{   
   ClassEntry *ce;

   ce = GetWordClassEntry(c,wdndx);
   if (ce == NULL)
      HError(15090,"IsClassMember: Word %d has no class",wdndx);
   return ce->ndx == clndx;
}


/* ------------------- Low Level Access ---------------- */

/* EXPORT->GetClassEntry: find class entry via cache */
ClassEntry *GetClassEntry(ClassMap *c, int clndx)
{
   int h;
   ClassEntry *ce;

   h = clndx % CLMHASHSIZE;
   for (ce = c->htab[h]; ce != NULL; ce = ce->next)
      if (ce->ndx == clndx) return ce;
   return NULL;
}

/* EXPORT->GetWordClassEntry: find class entry via word ndx */
ClassEntry *GetWordClassEntry(ClassMap *c, int wdndx)
{
   LabId id;
   MapEntry *me;

   id = WordLMName(wdndx,c->wmap);
   me = (MapEntry *)id->aux;
   if (me == NULL)
      HError(15090,"GetWordClassEntry: Word %d has no map entry",wdndx);
   return (ClassEntry *)me->class;
}

/* -------------------- End of LCMap.c ---------------------- */
