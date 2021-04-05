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
/*         File: HDict.c  Dictionary Storage                   */
/* ----------------------------------------------------------- */

char *hdict_version = "!HVER!HDict:   3.4.1 [CUED 12/03/09]";
char *hdict_vc_id = "$Id: HDict.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "HDict.h"

/* --------------------------- Trace Flags ------------------------- */

static int trace=0;

#define T_TOP  0001       /* Top Level tracing */
#define T_DIC  0002       /* Show Dict on loading */

/* --------------------------- Initialisation ---------------------- */

static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

/* EXPORT->InitDict: register module & set configuration parameters */
void InitDict(void)
{
   int i;

   Register(hdict_version,hdict_vc_id);
   nParm = GetConfig("HDICT", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

/* ----------- Vocab Hash Table Routines  ---------------------- */

/* VocabHash: return a hash value for given Word LabId */
static int VocabHash(LabId name)
{
   return (int) (((unsigned) name)%VHASHSIZE);
}

/* NewWord: Add a new word wordName to voc */
static Word NewWord(Vocab *voc, LabId wordName)
{
   Word wid; 

   wid = (Word) New(&voc->wordHeap, sizeof(DictEntry)); 
   wid->wordName = wordName;
   wid->pron = NULL;
   wid->nprons = 0;
   wid->aux = NULL;
   voc->nwords++;
   return wid;
}

/* EXPORT->DelWord: Delete word and its pronunciations from voc */
void DelWord(Vocab *voc, Word word)
{
   int h;
   Pron p;
   Word *v;

   /* Remove from hash table (if present) */
   h = VocabHash(word->wordName);
   for (v=&(voc->wtab[h]); *v!=NULL; v=&((*v)->next))
      if (*v==word) break;
   if (*v!=NULL) {
      *v = (*v)->next;
      /* Free memory and remove from voc */
      for (p=word->pron; p!=NULL; p=p->next) {
         voc->nprons--;
         Dispose(&voc->pronHeap,p);
      }
      voc->nwords--;
      Dispose(&voc->wordHeap,word);
   }
}

/* EXPORT->GetWord: return word of given wordName */
Word GetWord(Vocab *voc, LabId wordName, Boolean insert)
{
   int h;
   Word p;

   h = VocabHash(wordName); p = voc->wtab[h];
   if (p==NULL) {  /* special case - this slot empty */
      if (insert) {
         p=voc->wtab[h]=NewWord(voc,wordName);
         p->next=NULL;
      }
      return p;
   }
   do{             /* general case - look for name */
      if (wordName == p->wordName)
         return p; /* found it */
      p = p->next;
   } while (p != NULL);
   if (insert){    /* name not stored */
      p = NewWord(voc,wordName);
      p->next = voc->wtab[h];
      voc->wtab[h] = p; 
   }
   return p;
}

/* EXPORT->NewPron: add a pron to a given word - pron stored in phones */
void NewPron(Vocab *voc, Word wid, int nphones, LabId *phones, 
             LabId outSym, float prob)
{
   WordPron *pron, **p;
   int i;

   pron = (WordPron *) New(&voc->pronHeap, sizeof(WordPron));
   if (nphones>0)
      pron->phones = (LabId *) New(&voc->phonesHeap,nphones*sizeof(LabId));
   else
      pron->phones = NULL;
   pron->nphones = nphones;
   for (i=0; i<nphones; i++)
      pron->phones[i] = phones[i];
   pron->outSym = outSym;
   pron->word = wid;
   if (prob>=MINPRONPROB && prob<=1.0)
      pron->prob = log(prob);
   else if (prob>=0.0 && prob<MINPRONPROB)
      pron->prob=LZERO;
   else
      pron->prob = 0.0;
   for (p=&(wid->pron), i=0; *p!=NULL; p=&((*p)->next), i++);
   pron->next = NULL;
   pron->pnum = i+1;
   *p=pron; 
   wid->nprons++;
   voc->nprons++;
}

/* EXPORT->DelPron: delete a specific pronunciation */
void DelPron(Vocab *voc, Word word, Pron pron)
{
   int i;
   Pron *p, q;

   for (p=&(word->pron); *p!=NULL; p=&((*p)->next))
      if ((*p)==pron) break;
   if (*p!=NULL) {
      *p = (*p)->next;
      for (q=word->pron,i=1; q!=NULL; q=q->next,i++) q->pnum=i;
      Dispose(&voc->pronHeap, pron);
      word->nprons--; voc->nprons--;
   }
}

/* EXPORT->ShowDict: print out the pronunciation dictionary */
void ShowDict(Vocab *voc)
{
   Word wid;
   Pron thisPron;
   int i,j;

   for (i = 0; i< VHASHSIZE; i++)
      for ( wid = voc->wtab[i]; wid != NULL; wid = wid->next ) 
         for (thisPron = wid->pron; thisPron != NULL; thisPron = thisPron->next) {
            printf("%4d: %-20s",i,wid->wordName->name);
            if (thisPron->outSym)
               printf(" [%s]",thisPron->outSym->name);
            else
               printf(" []");
            for (j=0; j < thisPron->nphones; j++)
               printf(" %s",thisPron->phones[j]->name);
            printf("\n");
         }     

}

/* EXPORT->InitVocab: Initialise voc data structure */
void InitVocab(Vocab *voc)
{
   int i;

   CreateHeap(&voc->wordHeap,"Word Heap",MHEAP,sizeof(DictEntry),
              0.4,200,2000);
   CreateHeap(&voc->pronHeap,"Pron Heap",MHEAP,sizeof(WordPron),
              0.4,200,2000);
   CreateHeap(&voc->phonesHeap,"Phones Heap",MSTAK,1,0.4,400,4000);
   voc->wtab = (Word*) New(&voc->phonesHeap,sizeof(Word)*VHASHSIZE);
   for (i=0; i<VHASHSIZE; i++)
      voc->wtab[i] = NULL;
   voc->nullWord = GetWord(voc, GetLabId("!NULL",TRUE), TRUE);
   voc->subLatWord = GetWord(voc, GetLabId("!SUBLATID",TRUE), TRUE);
   voc->nwords = voc->nprons = 0;
}

/* EXPORT-> ClearVocab: Clears vocabulary datastructure */
void ClearVocab(Vocab *voc)
{
   DeleteHeap(&voc->wordHeap);
   DeleteHeap(&voc->pronHeap);
   DeleteHeap(&voc->phonesHeap);
}

/* EXPORT->ReadDictWord: Read word and pron from src */
ReturnStatus ReadDictWord(Source *src,LabId *labels,float *prob, int *num)
{
   char buf[MAXSTRLEN];
   int len,nphones;
   char *ptr;
   float p=-1.0,v;

   if(!ReadString(src,buf)){
      *num=-1;
      return(SUCCESS);
   }
   if (prob!=NULL)
      *prob=1.0;
   labels[0]=GetLabId(buf,TRUE);
   labels[1]=NULL;nphones=0;
   SkipWhiteSpace(src);
   while (!src->wasNewline) {
      if (!ReadString(src,buf)){
         HRError(8050,"ReadDict: Phone or outsym expected in word %s",
                 labels[0]->name);
         return(FAIL);
      }
      len = strlen(buf);
      if (buf[0] == '[' && buf[len-1] == ']') {    /* outsym */
         if (labels[1]!=NULL || nphones!=0){ 
            HRError(8050,"ReadDict: Only single outsym allowed for word %s",
                    labels[0]->name);
            return(FAIL);
         }
         buf[len-1] = '\0';
         labels[1] = GetLabId(buf+1,TRUE);
      } 
      else {
         if (nphones==0 && p<0)
            v=strtod(buf,&ptr);
         else
            v=0.0,ptr=buf;
         if (ptr!=buf) {
            if (v<=0.0 || v>1.0 || *ptr!=0) {
               HRError(8050,"ReadDict: Probability malformed %s",buf);
               return(FAIL);
            }
            p=v;
            if (prob!=NULL) *prob=v;
         }
         else {
            if (nphones==MAXPHONES){
               HRError(8050,"ReadDict: Too many phones in word %s",
                       labels[0]->name);
               return(FAIL);
            }
            labels[2+nphones++] = GetLabId(buf,TRUE);
         }
      }
      SkipWhiteSpace(src);
   }
   labels[nphones+2] = NULL;
   *num=nphones;
   return(SUCCESS);
}


/* EXPORT->ReadDict: read and store a dictionary definition */
ReturnStatus ReadDict(char *dictFn, Vocab *voc)
{
   LabId labels[MAXPHONES+4];
   Source src;
   Word word;
   float prob;
   int nphones;
   ReturnStatus ret;

   if(InitSource(dictFn,&src,DictFilter)<SUCCESS){
      HRError(8010,"ReadDict: Can't open file %s", dictFn);
      return(FAIL);
   }
   if (trace&T_TOP)
      printf("\nLoading Dictionary from %s\n",dictFn);
   if((ret=ReadDictWord(&src,labels,&prob, &nphones))<SUCCESS){
      CloseSource(&src);
      HRError(8013,"ReadDict: Dict format error in first entry");
      return(FAIL);
   }
   while(nphones>=0){
      word = GetWord(voc,labels[0],TRUE);
      if (labels[1]==NULL) labels[1]=labels[0];
      if (labels[1]->name[0]==0) labels[1]=NULL;
      if (voc->nullWord->wordName == word->wordName)
         HRError(-8013,"ReadDict: !NULL entry contains pronunciation");
      NewPron(voc,word,nphones,labels+2,labels[1],prob);
      if((ret=ReadDictWord(&src,labels,&prob, &nphones))<SUCCESS){
         HRError(8013,"ReadDict: Dict format error");
         return(FAIL);
      }
   }
   CloseSource(&src);

   if (trace&T_DIC)
      ShowDict(voc);
   if (trace&T_TOP)
      printf("Dictionary loaded from %s with %d words and %d prons\n\n",
             dictFn,voc->nwords,voc->nprons);
   return(SUCCESS);
}


/* Wd_Cmp: word order relation used to sort dictionary output */
static int Wd_Cmp(const void *v1,const void *v2)
{
   Word w1,w2;
   
   w1=*(Word*)v1;  w2=*(Word*)v2;
   return(strcmp(w1->wordName->name,w2->wordName->name));
}

/* Pad: pad output with nSp spaces subject to min of minSp */
static void Pad(FILE *f, int nSp, int minSp)
{
   int n;

   n = (nSp < minSp)?minSp:nSp;
   while (n-- > 0) fprintf(f," ");
}

#define WORDFIELDWIDTH 16

/* EXPORT->WriteDict: Write the given Vocab structure to the file dictFn */
ReturnStatus WriteDict(char *dictFn, Vocab *voc)
{
   FILE *df;
   Boolean isPipe,withOut,withProbs;
   Word wid, *wlist;
   Pron thisPron;
   float prob;
   char buf[MAXSTRLEN];
   int i,j,nw;

   nw = voc->nwords;
   if (trace&T_TOP)
      printf("WriteDict: %d words/%d prons to %s\n",
             nw,voc->nprons,dictFn);  
   if ( (df = FOpen(dictFn,DictOFilter,&isPipe)) == NULL){
      HRError(8011,"WriteDict: Cannot create dictionary file %s",dictFn);
      return(FAIL);
   }
   /* Create array of words */
   j = 0;
   wlist = (Word *)New(&gstack,sizeof(Word)*(nw+1));
   for (i=0,withOut=withProbs=FALSE; i< VHASHSIZE; i++)
      for ( wid = voc->wtab[i]; wid != NULL; wid = wid->next ) {
         if (wid==voc->nullWord || wid==voc->subLatWord)
            continue;
         if (j>=nw){
            FClose(df, isPipe);
            HRError(8090,"WriteDict: wlist full [%d]",j);
            return(FAIL);
         }
         wlist[j++] = wid;
         for (thisPron = wid->pron; thisPron != NULL; thisPron = thisPron->next) {
            if (thisPron->outSym==NULL || thisPron->outSym != wid->wordName) 
               withOut=TRUE;
            if (thisPron->prob!=0.0)
               withProbs=TRUE;
         }
      }
   if (j!=nw){
      HRError(-8090,"WriteDict: only %d of %d words found",j,nw);
   }
   /* sort list */
   qsort(wlist,nw,sizeof(Word),Wd_Cmp);

   /* print list of prons */
   for (i=0; i<nw; i++){
      wid = wlist[i];
      for (thisPron = wid->pron; thisPron != NULL; thisPron = thisPron->next) {
         ReWriteString(wid->wordName->name,buf,ESCAPE_CHAR);
         fprintf(df,"%s",buf); 
         Pad(df,WORDFIELDWIDTH-strlen(buf),1);
         if (thisPron->outSym==NULL) {
            fprintf(df,"[]");
            Pad(df,WORDFIELDWIDTH-2,0);
         } else if (thisPron->outSym != wid->wordName) {
            ReWriteString(thisPron->outSym->name,buf,ESCAPE_CHAR);
            fprintf(df,"[%s]",buf);
            Pad(df,WORDFIELDWIDTH-strlen(buf)-2,0);
         } else if (withOut)
            Pad(df,WORDFIELDWIDTH,0);
         if (withProbs) {
            prob=(thisPron->prob>LSMALL && thisPron->prob<=0.0)?exp(thisPron->prob):1.0;
            if (prob<1.0) fprintf(df," %8.6f",prob);
            /* 1.0 is just skipped */
            else Pad(df,9,0);
         }

         for (j=0; j < thisPron->nphones; j++) {
            fputc(' ',df);
            WriteString(df,thisPron->phones[j]->name,ESCAPE_CHAR);
         }
         fprintf(df,"\n");
      }
   }
   FClose(df,isPipe);
   return(SUCCESS);
}

/* ------------------------ End of HDict.c ----------------------- */
