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
/*      File: LUtil:    General Utility Routines               */
/* ----------------------------------------------------------- */

char *lutil_version = "!HVER!LUtil:   3.4.1 [CUED 12/03/09]";
char *lutil_vc_id = "$Id: LUtil.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"

/* ------------------------ Trace Flags --------------------- */

static int trace = 0;
#define T_TOP   0001       /* top level tracing */
#define T_HDR   0002       /* show header processing */
#define T_HASH  0004       /* hash table tracing */

/* --------------------- Global Variables ------------------- */

static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;
static MemHeap hashTableHeap;           /* heap for hash tables */

/* --------------------- Initialisation --------------------- */

/* EXPORT -> InitWMap: initialise the module */
void InitLUtil(void)
{
   int i;

   Register(lutil_version,lutil_vc_id);
   nParm = GetConfig("LUTIL", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
   CreateHeap(&hashTableHeap,"hashTableHeap",MSTAK,1,0.5,5000,20000);
}

/* ----------- HLM Header Type and Operations -------------- */

typedef struct hdrField{
   char * name;
   char * value;
   struct hdrField * next;
} HdrField;

typedef struct lmFileHdrRec {
   MemHeap *mem;
   int numFields;
   struct hdrField *list;
}LMFileHdrRec;

/* NumLines: return num lines remaining in source and rewind it */
static int NumLines(Source *src, IOFilter filter)
{
   int count = 0;

   while (SkipLine(src)) ++count;
   CloseSource(src);
   if (InitSource(src->name,src,filter)==FAIL) {
      HError(15299, "NumLines: Unable to reopen file '%s'", src->name);
   };
   return count;
}

/* HeaderKind: checks end of header tag and returns kind */
static LMHdrKind HeaderKind(char *s,int l)
{
   char *p;
   int len;

   len = strlen(s);
   if (s[len-1] == '\r') len--; /* also work if terminated with CR+LF */
   if (len>3 && s[0] == '\\' && s[len-1] == '\\') {
      s[len-1] = '\0'; p = s+1; len -=2;
      UpperCase(p);
      if (strcmp(p,"WORDS") == 0) return WMAP_HDR;
      if (strcmp(p,"CLASSES") == 0) return CMAP_HDR;
      if (strcmp(p,"GRAMS") == 0) return GRAM_HDR;
      if (strcmp(p,"FOFS") == 0) return LFOF_HDR;
   }
   HError(15250,"HeaderKind: Expected data tag at line %d",l);
   return 0;
}

/* GetHeaderStr: extract string from s into buf */
static void GetHeaderStr(char *s, char *buf)
{
   int i=0,j=0,k,l=0,len;

   len = strlen(s); j = len-1;
   while (isspace((int) s[i]) && i<len) i++;
   while (isspace((int) s[j]) && j>i) j--;
   for (k=i; k<=j; k++)
      buf[l++] = s[k];
   buf[l] = '\0';
}

/* EXPORT->ReadLMHeader: read a std HLM file header */
LMHdrKind ReadLMHeader(MemHeap *mem, Source *src, IOFilter filter,
                       LMFileHdr *hdr, int *n)
{
   HdrField *h;
   char line[1024],*esign;
   char name[MAXSTRLEN],value[MAXSTRLEN];

   if (trace&T_HDR)
      printf("Reading header:\n");
   *hdr = (LMFileHdrRec *)New(mem,sizeof(LMFileHdrRec));
   (*hdr)->numFields = 0;
   (*hdr)->list = NULL;
   if (!ReadLine(src,line))
      HError(15213,"ReadLMHeader: File is empty");
   while ((esign = strchr(line,'=')) != NULL) {
      GetHeaderStr(esign+1,value); *esign = '\0';
      GetHeaderStr(line,name);  UpperCase(name);
      (*hdr)->numFields ++;
      h = (HdrField *)New(mem,sizeof(HdrField));
      h->name = CopyString(mem,name);
      if (strlen(name) == 0)
         HError(15213,"ReadLMHeader: Field %d name is empty", (*hdr)->numFields);
      h->value = CopyString(mem,value);
      h->next = (*hdr)->list; (*hdr)->list = h;
      if (trace&T_HDR)
         printf("  %d %s = %s\n",(*hdr)->numFields,name,value);
      if (!ReadLine(src,line))
         HError(15213,"ReadLMHeader: Unexpected EOF");
      if (BlankString(line))
         HError(15213,"ReadLMHeader: Line %d is blank", (*hdr)->numFields+1);
   }
   if ((*hdr)->numFields == 0) {  /* must be headerless */
      *n = NumLines(src,filter) + 1;   /* already read 1st line */
      return NO_HDR;
   } else {
      if (!GetLMHdrInt("ENTRIES", n, *hdr))
         HError(15213,"ReadLMHeader: No ENTRIES field in header");
      return HeaderKind(line,(*hdr)->numFields+1);
   }
}

/* EXPORT->NumLMHdrFields: return number of fields in hdr */
int NumLMHdrFields(LMFileHdr hdr)
{
   return hdr->numFields;
}

/* EXPORT->GetLMHdrStr: get string valued field */
char * GetLMHdrStr(char *name, LMFileHdr hdr, Boolean ucase)
{
   HdrField *h;

   if (hdr->numFields == 0) return NULL;
   for (h=hdr->list; h != NULL; h = h->next) {
      if (strcmp(h->name,name) == 0) {
         if (ucase) UpperCase(h->value);
         return h->value;
      }
   }
   return NULL;
}

/* EXPORT->GetLMHdrInt: get int valued field */
Boolean GetLMHdrInt(char *name, int *value, LMFileHdr hdr)
{
   char *s;

   s = GetLMHdrStr(name,hdr,FALSE);
   if (s==NULL) return FALSE;
   if (strlen(s) == 0) return FALSE;
   if (! ( isdigit((int) s[0]) || s[0] == '-' || s[0] == '+') ) return FALSE;
   *value = atoi(s);
   return TRUE;
}


/* ------------------ Miscellaneous Operations ----------------- */

/* EXPORT->UpperCase: convert s to upper case */
void UpperCase(char *s)
{
   int i;
   for (i=0; i<strlen(s); i++) s[i] = toupper(s[i]);
}

/* EXPORT->BlankString: return true if s is blank */
Boolean BlankString(char *s)
{
   int i;
   for (i=0; i<strlen(s); i++)
      if (!isspace((int) s[i])) return FALSE;
   return TRUE;
}

/* ------------------ Hash table Operations ----------------- */

/* Hash: return a hash value for given label name */
static unsigned Hash(char *name, int hashSize)
{
   unsigned hashval;

   for (hashval=0; *name != '\0'; name++)
      hashval = *name + 31*hashval;
   return hashval%hashSize;
}

/* NewHolder: return a pointer to a new NameHolder */
static NameHolder *NewHolder(char *name)
{
   char *s;
   NameHolder *p;
   int len;

   len = strlen(name);
   s = (char *)New(&hashTableHeap,len+1);
   strcpy(s,name);
   p = (NameHolder *) New(&hashTableHeap,sizeof(NameHolder));
   p->name = s; p->next = NULL; p->aux = 0; p->wptr = NULL;
   return p;
}

/* EXPORT->Create and initialise a hash table */
HashTab *CreateHashTable(int hashSize, char *description)
{
   int i;
   HashTab *htab;

   htab = (HashTab *) New(&hashTableHeap,sizeof(HashTab));
   htab->nameTab =
     (NameHolder **) New(&hashTableHeap,hashSize*sizeof(NameHolder *));
   for (i=0;i<hashSize;i++) htab->nameTab[i] = NULL;
   htab->hashSize = hashSize;
   htab->description = description;
   htab->numAccesses = 0;
   htab->numTests = 0;
   return htab;
}

/* EXPORT->GetNameId: return id of given name */
NameId GetNameId(HashTab *htab, char *name, Boolean insert)
{
   int h;
   NameHolder *p;

   htab->numAccesses++; htab->numTests++;
   if ((trace&T_HASH) && htab->numAccesses%100 == 0)
      PrintHashTabStats(htab);
   h = Hash(name,htab->hashSize); p = htab->nameTab[h];
   if (p==NULL) {  /* special case - this slot empty */
      if (insert)
         p=htab->nameTab[h] = NewHolder(name);
      return p;
   }
   do{             /* general case - look for name */
      if (strcmp(name,p->name) == 0)
         return p; /* found it */
      htab->numTests++;
      p = p->next;
   } while (p != NULL);
   if (insert){    /* name not stored */
      p = NewHolder(name);
      p->next = htab->nameTab[h];
      htab->nameTab[h] = p;
   }
   return p;
}

/* EXPORT->PrintHashTabStats: print out statistics on hash table usage */
void PrintHashTabStats(HashTab *htab)
{
   printf("Hash Table Statistics for %s:\n", htab->description);
   printf("Total Accesses: %ld\n",htab->numAccesses);
   printf("Avg Search Len: %f\n",(float)(htab->numTests)/(float)(htab->numAccesses));
   PrintHeapStats(&hashTableHeap);
   printf("\n"); fflush(stdout);
}

/* -------------------- Sorting Routines  -------------------- */

const static int ci[28]={ 28,0,0,2,1,0,0,1,2,3,0,0,3,0,
			   2,0,3,0,0,3,2,1,1,0,0,2,0,0 };
static int sign(int r)
{
   int s;
   s=(r>0)-(r<0);
   return(s);
}

static void u4srt(int32 *data, int l, int r,
		  int (*compar)(const void *, const void *))
{
   int32 t,p,*p1,*p2,*p3;
   int c,o,x,i,j,n;
   Boolean sorted;

   if (l>=r) return;
   n=r-l+1;
   switch(n) {
    case 2:
       p1=data+l; p2=data+r;
       if (compar(p1,p2)>0) t=*p1,*p1=*p2,*p2=t;
       break;
    case 3:
       c=((l+r)>>1);
       p1=data+l; p2=data+c; p3=data+r;
       if (compar(p1,p3)>0)      t=*p1,*p1=*p3,*p3=t; /* p1 < p3 */
       if (compar(p2,p1)<0)      t=*p1,*p1=*p2,*p2=t; /* p2 < p1 p3 */
       else if (compar(p3,p2)<0) t=*p2,*p2=*p3,*p3=t; /* p1 p3 < p2 */
       /* else p1 < p2 < p3 */
       break;
    default:
       c=((l+r)>>1); o=(n>>2);
       p1=data+(c-o); p2=data+c; p3=data+(c+o);
       x=ci[(14+9*sign(compar(p1,p2))+
	     3*sign(compar(p2,p3))+sign(compar(p3,p1)))];
       switch(x) {
	case 3:  p=*p1; break;
	case 1:  p=*p2; break;
	default: p=*p3; break;
       }
       i=l-1;j=r+1; sorted=TRUE;
       do {
	  j--;
	  while (compar(&p,(p1=(data+j)))<0) {
	     if (sorted && compar(p1,p1-1)<0) sorted=FALSE;
	     j--;
	  }
	  i++;
	  while (compar(&p,(p3=(data+i)))>0) {
	     if (sorted && compar(p3+1,p3)<0) sorted=FALSE;
	     i++;
	  }
	  if (i<j) {
	     t=data[i]; data[i]=data[j]; data[j]=t; sorted=FALSE;
	  }
       }
       while(i<j);
       if (!sorted) {
	  u4srt(data,l,j,compar);
	  u4srt(data,j+1,r,compar);
       }
       break;
   }
}


static void usrt(void *base, int l, int r, size_t size,
		 int (*compar)(const void *, const void *))
{
   static char tmp[1024],p[1024];
   Boolean sorted;
   char *first=base,*p1,*p2,*p3;
   int c,o,x,i,j,n;

   if (l>=r) return;
   n=r-l+1;
   switch(n) {
    case 2:
       p1=first+l*size; p2=first+r*size;
       if (compar(p1,p2)>0) {
	  memcpy(tmp,p1,size);
	  memcpy(p1,p2,size);
	  memcpy(p2,tmp,size);
       }
       break;
    case 3:
       c=((l+r)>>1);
       p1=first+l*size; p2=first+c*size; p3=first+r*size;
       if (compar(p1,p3)>0) {
	  memcpy(tmp,p1,size);
	  memcpy(p1,p3,size);
	  memcpy(p3,tmp,size); /* p1 > p3 */
       }
       if (compar(p2,p1)<0) {
	  memcpy(tmp,p1,size);
	  memcpy(p1,p2,size);
	  memcpy(p2,tmp,size); /* p2 < p1 p3 */
       }
       else if (compar(p3,p2)<0) {
	  memcpy(tmp,p2,size);
	  memcpy(p2,p3,size);
	  memcpy(p3,tmp,size); /* p1 p3 < p2 */
       }
       /* else p1 < p2 < p3 */
       break;
    default:
       c=((l+r)>>1)*size; o=(n>>2)*size;
       p1=first+(c-o); p2=first+c; p3=first+(c+o);
       x=ci[(14+9*sign(compar(p1,p2))+
	     3*sign(compar(p2,p3))+sign(compar(p3,p1)))];
       switch(x) {
	case 3:  memcpy(&p,p3,size); break;
	case 1:  memcpy(&p,p1,size); break;
	default: memcpy(&p,p2,size); break;
       }
       i=l-1;j=r+1; sorted=TRUE;
       do {
	  j--;
	  while (compar(&p,(p1=(first+size*j)))<0) {
	     if (sorted && compar(p1,p1-size)<0) sorted=FALSE;
	     j--;
	  }
	  i++;
	  while (compar(&p,(p3=(first+size*i)))>0) {
	     if (sorted && compar(p3+size,p3)<0) sorted=FALSE;
	     i++;
	  }
	  if (i<j) {
	     memcpy(tmp,first+size*i,size);
	     memcpy(first+size*i,first+size*j,size);
	     memcpy(first+size*j,tmp,size); sorted=FALSE;
	  }
       }
       while(i<j);
       if (!sorted) {
	  usrt(first,l,j,size,compar);
	  usrt(first,j+1,r,size,compar);
       }
       break;
   }
}

void usort(void *base, int n, size_t size,
	   int (*compar)(const void *, const void *))
{
   if (size==sizeof(int32)) u4srt(base,0,n-1,compar);
   else usrt(base,0,n-1,size,compar);
}

/* -------------------- End of LUtil.c ---------------------- */
