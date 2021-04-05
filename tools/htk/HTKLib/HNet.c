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
/*         File: HNet.c  Network and Lattice Functions         */
/* ----------------------------------------------------------- */

char *hnet_version = "!HVER!HNet:   3.4.1 [CUED 12/03/09]";
char *hnet_vc_id = "$Id: HNet.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HAudio.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HUtil.h"
#include "HDict.h"
#include "HNet.h"

/* ----------------------------- Trace Flags ------------------------- */

#define T_CXT 0001         /* Trace context definitions */
#define T_CST 0002         /* Trace network construction */
#define T_MOD 0004         /* Show models making up each word */
#define T_ALL 0010         /* Show whole network */

static int trace=0;
static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

/* --------------------------- Global Flags -------------------------- */

Boolean forceCxtExp=FALSE;
/*
  force triphone context exp to get model names
  ie. don't use model names direct from dict 
  without expansion (is overridden by allowCxtExp)
*/
Boolean forceLeftBiphones=FALSE;
Boolean forceRightBiphones=FALSE;
/*
  force biphone context exp to get model names
  ie. don't try triphone names
*/
Boolean allowCxtExp=TRUE;
/*
   allow context exp to get model names #
*/
Boolean allowXWrdExp=FALSE;
/*
   allow context exp across words
*/
Boolean cfWordBoundary=TRUE;
/*
   In word internal systems treat context free phones as word boundaries.
*/
Boolean factorLM=FALSE;
/*
   factor lm likelihoods throughout words
*/

char *frcSil=NULL,frcSilBuf[MAXSTRLEN];
/* 
   Automagically add these sil models to the end of words.
*/
Boolean remDupPron=TRUE;
/*
   Remove duplicate pronunciations
*/

Boolean sublatmarkers=FALSE;
/*
   Add sublatstart and sublatend markers to the lattice
*/
char *subLatStart="!SUBLAT_(",subLatStartBuf[MAXSTRLEN];
char *subLatEnd="!)_SUBLAT",subLatEndBuf[MAXSTRLEN];
/* 
   Set these strings as the start and end sublattice markers 
*/

/* --------------------------- Initialisation ---------------------- */

/* EXPORT->InitNet: register module & set configuration parameters */
void InitNet(void)
{
   Boolean b;
   int i;

   Register(hnet_version,hnet_vc_id);
   nParm = GetConfig("HNET", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfBool(cParm,nParm,"FORCECXTEXP",&b)) forceCxtExp = b;
      if (GetConfBool(cParm,nParm,"FORCELEFTBI",&b)) forceLeftBiphones = b;
      if (GetConfBool(cParm,nParm,"FORCERIGHTBI",&b)) forceRightBiphones = b;
      if (GetConfBool(cParm,nParm,"ALLOWCXTEXP",&b)) allowCxtExp = b;
      if (GetConfBool(cParm,nParm,"ALLOWXWRDEXP",&b)) allowXWrdExp = b;
      if (GetConfBool(cParm,nParm,"CFWORDBOUNDARY",&b)) cfWordBoundary = b;
      if (GetConfBool(cParm,nParm,"FACTORLM",&b)) factorLM = b;
      if (GetConfStr(cParm,nParm,"ADDSILPHONES",frcSilBuf)) frcSil=frcSilBuf;
      if (GetConfStr(cParm,nParm,"STARTSUBLAT",subLatStartBuf)) 
         subLatStart=subLatStartBuf;
      if (GetConfStr(cParm,nParm,"ENDSUBLAT",subLatEndBuf)) 
         subLatEnd=subLatEndBuf;
      if (GetConfBool(cParm,nParm,"REMDUPPRON",&b)) remDupPron = b;
      if (GetConfBool(cParm,nParm,"MARKSUBLAT",&b)) sublatmarkers = b;
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

/* ------------------------ Lattice Creation ------------------------- */

#define SafeCopyString(heap,str) ((str)==NULL?NULL:CopyString((heap),(str)))

/* EXPORT->NewLattice: Allocate and initialise a new lattice structure */
Lattice *NewLattice(MemHeap *heap,int nn,int na)
{
   Lattice *lat;
   LNode *ln;
   LArc *la;
   int i;
  
   lat=(Lattice *) New(heap,sizeof(Lattice));
   lat->heap=heap;
   lat->nn=nn;
   lat->na=na;

   lat->format=0;
   lat->utterance=lat->vocab=NULL;
   lat->net=lat->hmms=NULL;
   lat->acscale=1.0;
   lat->lmscale=1.0; lat->wdpenalty=0.0;
   lat->prscale=0.0; lat->framedur=0.0;
   lat->logbase = 1.0;
   lat->tscale = 1.0;
   lat->subList = NULL;
   lat->refList = NULL;
   lat->subLatId = NULL;
   lat->chain = NULL;

   if (nn>0) lat->lnodes=(LNode *) New(heap, sizeof(LNode)*nn);
   else lat->lnodes=NULL;
     
   if (na>0) lat->larcs=(LArc *) New(heap, sizeof(LArc)*na);
   else lat->larcs=NULL;

   for(i=0,ln=lat->lnodes;i<nn;i++,ln++) {
      ln->time=0.0;ln->word=NULL;ln->tag=NULL;
      ln->score=0.0;
      ln->foll=ln->pred=NARC;
      ln->hook=NULL;
      ln->sublat=NULL;
   }
   for(i=0,la=lat->larcs;i<na;i++,la++) {
      la->aclike=la->lmlike=la->prlike=0.0;
      la->start=la->end=NNODE;
      la->farc=la->parc=NARC;
      la->nAlign=0;la->lAlign=NULL;
   }
   return(lat);
}

/* EXPORT->NewILattice: Allocate and initialise a new lattice structure */
Lattice *NewILattice(MemHeap *heap,int nn,int na,Lattice *info)
{
   Lattice *lat;
   LNode *ln,*in;
   LArc *la,*ia;
   int i,j;
  
   lat=(Lattice *) New(heap,sizeof(Lattice));
   lat->heap=heap;
   lat->nn=nn;
   lat->na=na;

   lat->format = info->format;

   lat->voc = info->voc;
   lat->subLatId = info->subLatId;
   lat->subList=NULL; lat->refList=NULL; lat->chain=NULL;
   lat->utterance = SafeCopyString(heap,info->utterance);
   lat->vocab = SafeCopyString(heap,info->vocab);
   lat->hmms = SafeCopyString(heap,info->hmms);
   lat->net = SafeCopyString(heap,info->net);
   lat->acscale = info->acscale;
   lat->lmscale = info->lmscale;
   lat->wdpenalty = info->wdpenalty;
   lat->prscale = info->prscale;
   lat->framedur = info->framedur;
   lat->logbase = info->logbase;
   lat->tscale = info->tscale;

   if (nn==-1) {
      lat->lnodes=(LNode *) New(heap, sizeof(LNode)*info->nn);
      lat->nn=info->nn;
   }
   else if (nn>0) {
      lat->lnodes=(LNode *) New(heap, sizeof(LNode)*nn);
      lat->nn=nn;
   }
   else {
      lat->lnodes=NULL;
      lat->nn=0;
   }

   if (info->format&HLAT_SHARC) i=sizeof(LArc_S);
   else i=sizeof(LArc);
   
   if (na==-1) {
      lat->larcs=(LArc *) New(heap, i*info->na);
      lat->na=info->na;
   }
   else if (na>0) {
      lat->larcs=(LArc *) New(heap, i*na);
      lat->na=na;
   }
   else {
      lat->larcs=NULL;
      lat->na=0;
   }

   if (nn==-1)
      for(i=0,ln=lat->lnodes,in=info->lnodes;i<lat->nn;i++,ln++,in++) {
         *ln=*in;
         if (in->word==lat->voc->subLatWord){
            ln->sublat=AdjSubList(lat,in->sublat->lat->subLatId,
                                  in->sublat->lat,+1);
            if(ln->sublat==NULL){
               HError(8253, "NewILattice: AdjSubList failed");
            }
         }
         if (in->foll!=NULL) ln->foll=NumbLArc(lat,LArcNumb(in->foll,info));
         if (in->pred!=NULL) ln->pred=NumbLArc(lat,LArcNumb(in->pred,info));
      }
   else
      for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++) {
         ln->time=0.0;ln->word=NULL;ln->tag=NULL;
         ln->score=0.0;
         ln->foll=ln->pred=NARC;
         ln->hook=NULL;
         ln->sublat=NULL;
      }
   if (na==-1)
      for(i=0;i<lat->na;i++) {
         la=NumbLArc(lat,i);
         ia=NumbLArc(info,i);
         if (info->format&HLAT_SHARC) *(LArc_S*)la=*(LArc_S*)ia;
         else *la=*ia;
         la->start=ia->start-info->lnodes+lat->lnodes;
         la->end=ia->end-info->lnodes+lat->lnodes;
         if (ia->farc!=NULL) la->farc=NumbLArc(lat,LArcNumb(ia->farc,info));
         if (ia->parc!=NULL) la->parc=NumbLArc(lat,LArcNumb(ia->parc,info));
         if (!(info->format&HLAT_SHARC) &&
             ia->nAlign>0 && ia->lAlign!=NULL) {
            la->lAlign=(LAlign *) New(heap,ia->nAlign*sizeof(LAlign));
            for (j=0;j<ia->nAlign;j++) la->lAlign[j]=ia->lAlign[j];
         }
      }
   else
      for(i=0;i<lat->na;i++) {
         la=NumbLArc(lat,i);
         la->lmlike=0.0;
         la->start=la->end=NNODE;
         la->farc=la->parc=NARC;
         if (!(info->format&HLAT_SHARC))
            la->aclike=0.0,la->nAlign=0,la->lAlign=NULL;
      }
   return(lat);
}

/* EXPORT->FreeLattice: free memory used by a lattice structure */
void FreeLattice(Lattice *lat)
{
   Dispose(lat->heap,lat);
}

/* ------------------------ Lattice Output ------------------------- */

#define SUBLATHASHSIZE 101

/* Search hash table for lattice matching subLatId.  If subLat!=NULL */
/* add (if not found) or check (if found) the new subLat definition. */
static Lattice *GetSubLat(LabId subLatId,Lattice *subLat)
{
   int h;
   Lattice *cur,*nxt;
   static Lattice **subLatHashTab = NULL;

   if (subLatHashTab==NULL) {
      /* Need to allocate and initialise table */
      subLatHashTab=(Lattice **) New(&gcheap,SUBLATHASHSIZE*sizeof(Lattice *));
      for (h=0;h<SUBLATHASHSIZE;h++) subLatHashTab[h]=NULL;
   }
   if (subLatId==NULL) {
      for (h=0;h<SUBLATHASHSIZE;h++)
         for (cur=subLatHashTab[h];cur!=NULL;cur=nxt) {
            nxt=cur->chain;
            cur->chain=NULL;
         }
      Dispose(&gcheap,subLatHashTab);
      subLatHashTab=NULL;
      return(NULL);
   }
   h=(((unsigned) subLatId)%SUBLATHASHSIZE);
   for (cur=subLatHashTab[h];cur!=NULL;cur=cur->chain)
      if (cur->subLatId==subLatId) break;
   if (subLat!=NULL) {
      if (cur==NULL) {
         /* Add this to table */
         cur=subLat;
         cur->chain=subLatHashTab[h];
         subLatHashTab[h]=cur;
      }
      if (cur!=subLat)
         HError(8253,"GetSubLat: All sublats must have unique names");
   }
      
   return(cur);
}

/* Search list of sublats for current lattice for a match */
/*  When add>0 either add new sublat or increase usage of match */
/*  when add<0 either decrease usage of match and possibly remove */
/*  If adding new sublat get actual lattice definition from hash */
/*  table containing all loaded so far unless supplied */
/* When add>0 if can't find sublat returns NULL - error condition */
SubLatDef *AdjSubList(Lattice *lat,LabId subLatId,Lattice *subLat,int adj)
{
   SubLatDef *p,*q,*r,*s;

   /* Inefficient linear search may be good enough */
   for (p=lat->subList,q=NULL;p!=NULL;q=p,p=p->next) 
      if (p->lat->subLatId==subLatId) break;
   if (adj<0) {
      if (p==NULL)
         HError(8253,"AdjSubList: Decreasing non-existent sublat",
                subLatId->name);
      p->usage+=adj;
      if (p->usage<=0) {
         /* First remove from refList */
         for (r=p->lat->refList,s=NULL;r!=NULL;s=r,r=r->chain) 
            if (r==p) break;
         if (r!=p || r==NULL)
            HError(8253,"AdjSubList: Could not find SubLatDef in refList");
         if (s==NULL) p->lat->refList=p->chain;
         else s->chain=r->chain;
         /* Then remove from subList */
         if (q==NULL) lat->subList=p->next;
         else q->next=p->next;
         p=NULL;
      }
   }
   else if (adj>0) {
      if (p==NULL) {
         p=(SubLatDef *) New(lat->heap,sizeof(SubLatDef));
         /* p->subLatId=subLatId; */
         if (subLat!=NULL) p->lat=subLat;
         else if ((p->lat=GetSubLat(subLatId,NULL))==NULL){
            HRError(8253,"AdjSubList: SUBLAT %s not found",subLatId->name);
            return NULL;
         }
         p->next=lat->subList;
         lat->subList=p;
         if (p->lat==lat){
            HRError(8253,"AdjSubList: Circular subLat reference to %s",
                    subLatId->name);
            return NULL;
         }
         p->chain=p->lat->refList;
         p->lat->refList=p;
         p->usage=0;
      }
      p->usage+=adj;
   }
   return(p);
}

#define MAXLATDEPTH 32

Lattice *SubLatList(Lattice *lat, Lattice *tail, int depth)
{
   SubLatDef *sub;

   if (depth>MAXLATDEPTH)
      HError(8253,"SubLatList: Nesting too deep (%d == recursive ?)",depth);

   for (sub=lat->subList;sub!=NULL;sub=sub->next) {
      if (tail==NULL) sub->lat->chain=NULL;
      if (sub->lat->chain==NULL)
         tail=SubLatList(sub->lat,tail,depth+1);
   }
   if (tail!=NULL)
      for (sub=lat->subList;sub!=NULL;sub=sub->next) {
         if (sub->lat->chain!=NULL) continue;  /* Done it already */
#if 0 /* Algorithm sanity check only needed once */
         {
            Lattice *cur;
         /* Quick sanity check */
         cur=tail->chain; /* Actual lattice == last in list */
         for (cur=cur->chain;cur!=tail->chain;cur=cur->chain)
            if (sub->lat==cur || sub->lat->subLatId==cur->subLatId || 
                sub->lat->subLatId==NULL) break;
         if (cur!=tail->chain) /* Match */
            HError(8253,"Match");
         }
#endif
         sub->lat->chain=tail->chain;
         tail->chain=sub->lat;
         tail=sub->lat;
      }
   return(tail);
}

/* Lattices often need to be sorted before output */

static Lattice *slat;   /* Used by qsort cmp routines */

/* QSCmpNodes: order on time, then score if times equal */
static int QSCmpNodes(const void *v1,const void *v2)
{
   int s1,s2;
   double tdiff,sdiff;
  
   s1=*((int*)v1);s2=*((int*)v2);
   tdiff=slat->lnodes[s1].time-slat->lnodes[s2].time;
   sdiff=slat->lnodes[s1].score-slat->lnodes[s2].score;
   if (tdiff==0.0) {
      if (sdiff==0.0) return(s1-s2);
      else if (sdiff>0.0) return(1);
      else return(-1);
   }
   else if (tdiff>0.0) return(1);
   else return(-1);
}

/* QSCmpArcs: order on end node order, then start node order */
static int QSCmpArcs(const void *v1,const void *v2)
{
   int s1,s2,j,k;
  
   s1=*((int*)v1);s2=*((int*)v2);
   j=slat->larcs[s1].end->n-slat->larcs[s2].end->n;
   k=slat->larcs[s1].start->n-slat->larcs[s2].start->n;
   if (k==0 && j==0) return(s1-s2);
   else if (j==0) return(k);
   else return(j);
}

/* OutputIntField: output integer as text or binary */
static void OutputIntField(char field,int val,Boolean bin,
                           char *form,FILE *file)
{
   fprintf(file,"%c%c",field,bin?'~':'=');
   if (bin)
      WriteInt(file,&val,1,TRUE);
   else
      fprintf(file,form,val);
   fprintf(file," ");
}

/* OutputFloatField: output float as text or binary */
static void OutputFloatField(char field,float val,Boolean bin,
                             char *form,FILE *file)
{
   fprintf(file,"%c%c",field,bin?'~':'=');
   if (bin)
      WriteFloat(file,&val,1,TRUE);
   else
      fprintf(file,form,val);
   fprintf(file," ");
}

/* OutputAlign: output models aligned with this arc */
static void OutputAlign(LArc *la,int format,FILE *file)
{
   int i;
   LAlign *lal;

   fprintf(file,"d=:");
   for(i=0,lal=la->lAlign;i<la->nAlign;i++,lal++) {
      fprintf(file,"%s",lal->label->name);
      if (format&HLAT_ALDUR)
         fprintf(file,",%.2f",lal->dur);
      if (format&HLAT_ALLIKE)
         fprintf(file,",%.2f",lal->like);
      fprintf(file,":");
   }
}

/* convert log likelihoods from/to external represnation with specified base.
   internal representation is ALWAYS natural logs, i.e. base e 
   base = 0.0   no logs
   base = 1.0   natural logs (no conversion required)
*/
#define ConvLogLikeFromBase(base, ll) ((base) == 0.0 ? log(ll) : \
                                       (base) == 1.0 ? (ll) : (ll) * log(base))
#define ConvLogLikeToBase(base, ll)  ((base) == 0.0 ? exp(ll) : \
                                      ((base) == 1.0 ? (ll) : (ll) / log(base)))

/* WriteOneLattice: Write a single lattice to file */
ReturnStatus WriteOneLattice(Lattice *lat,FILE *file,LatFormat format)
{
   int i, *order, *rorder, st, en;
   LNode *ln = NULL;
   LArc *la;

   /* Rather than return an error assume labels on nodes !! */
   order=(int *) New(&gstack, sizeof(int)*(lat->nn<lat->na ? lat->na+1 : lat->nn+1));
   rorder=(int *) New(&gstack, sizeof(int)*lat->nn);
   
   if (lat->subLatId) fprintf(file,"SUBLAT=%s\n",lat->subLatId->name);
   
   fprintf(file,"N=%-4d L=%-5d\n",lat->nn,lat->na);
   
   for (i=0;i<lat->nn;i++)
      order[i]=i;
   if (!(lat->format&HLAT_SHARC) && !(format&HLAT_NOSORT)) {
      slat=lat;
      qsort(order,lat->nn,sizeof(int),QSCmpNodes);
   }
   
   if ((format&HLAT_TIMES) || !(format&HLAT_ALABS)) {
      for (i=0;i<lat->nn;i++) {
         ln=lat->lnodes+order[i];
         rorder[order[i]]=i;
         ln->n = i;
         OutputIntField('I',i,format&HLAT_LBIN,"%-4d",file);
         if (format&HLAT_TIMES)
            OutputFloatField('t',ln->time / lat->tscale,format&HLAT_LBIN,"%-5.2f",file);
         if (!(format&HLAT_ALABS)) {
            if (ln->word==lat->voc->subLatWord && ln->sublat!=NULL)
               fprintf(file,"L=%-19s ",
                       ReWriteString(ln->sublat->lat->subLatId->name,
                                     NULL,ESCAPE_CHAR));
            else if (ln->word!=NULL) {
               fprintf(file,"W=%-19s ",
                       ReWriteString(ln->word->wordName->name,
                                     NULL,ESCAPE_CHAR));
               if ((format&HLAT_PRON) && ln->v>=0)
                  OutputIntField('v',ln->v,format&HLAT_LBIN,"%-2d",file);
               if ((format&HLAT_TAGS) && ln->tag!=NULL)
                  fprintf(file,"s=%-19s ",
                          ReWriteString(ln->tag,NULL,ESCAPE_CHAR));
            }
            else
               fprintf(file,"W=%-19s ","!NULL");
         }
         fprintf(file,"\n");
      }
   }
   else
      for (i=0; i < lat->nn; i++)
         rorder[order[i]]=i;

   for (i=0;i<lat->na;i++)
      order[i]=i;
   if (!(lat->format&HLAT_SHARC) && !(format&HLAT_NOSORT)) {
      slat=lat;
      qsort(order,lat->na,sizeof(int),QSCmpArcs);
   }
   for (i=0;i<lat->na;i++) {
      la=NumbLArc(lat,order[i]);
      OutputIntField('J',i,format&HLAT_LBIN,"%-5d",file);
      st=rorder[la->start-lat->lnodes];
      en=rorder[la->end-lat->lnodes];
      OutputIntField('S',st,format&HLAT_LBIN,"%-4d",file);
      OutputIntField('E',en,format&HLAT_LBIN,"%-4d",file);
      if (format&HLAT_ALABS) {
         if (la->end->word!=NULL) 
            fprintf(file,"W=%-19s ",
                    ReWriteString(la->end->word->wordName->name,
                                  NULL,ESCAPE_CHAR));
         else
            fprintf(file,"W=%-19s ","!NULL");
         if ((format&HLAT_PRON) && ln->v>=0)
            OutputIntField('v',la->end->v,format&HLAT_LBIN,"%-2d",file);
      }
      if (!(lat->format&HLAT_SHARC) && (format&HLAT_ACLIKE))
         OutputFloatField ('a', ConvLogLikeToBase(lat->logbase, la->aclike), format&HLAT_LBIN, "%-9.2f", file);
      if (format&HLAT_LMLIKE) {
         if (lat->net==NULL)
            OutputFloatField('l', ConvLogLikeToBase(lat->logbase, la->lmlike*lat->lmscale+lat->wdpenalty),
                             format&HLAT_LBIN,"%-8.2f",file);
         else
            OutputFloatField('l',ConvLogLikeToBase(lat->logbase, la->lmlike), format&HLAT_LBIN, "%-7.3f", file);
      }
      if (!(lat->format&HLAT_SHARC) && (format&HLAT_PRLIKE))
         OutputFloatField('r', ConvLogLikeToBase(lat->logbase, la->prlike), format&HLAT_LBIN, "%-6.2f", file);
      if (!(lat->format&HLAT_SHARC) && (format&HLAT_ALIGN) && la->nAlign>0)
         OutputAlign(la,format,file);
      fprintf(file,"\n");
   }
   
   if (lat->subLatId) fprintf(file,".\n\n");

   Dispose(&gstack,order);
   slat=NULL;
   return(SUCCESS);
}

/* EXPORT->WriteLattice: Write lattice to file */
ReturnStatus WriteLattice(Lattice *lat,FILE *file,LatFormat format)
{
   LabId id;
   Lattice *list;
   
   fprintf(file,"VERSION=%s\n",L_VERSION);
   if (lat->utterance!=NULL)
      fprintf(file,"UTTERANCE=%s\n",lat->utterance);
   if (lat->net!=NULL) {
      fprintf(file,"lmname=%s\nlmscale=%-6.2f wdpenalty=%-6.2f\n",
              lat->net,lat->lmscale,lat->wdpenalty);
   }
   if (format&HLAT_PRLIKE)
      fprintf(file,"prscale=%-6.2f\n",lat->prscale);
   if (format&HLAT_ACLIKE)
      fprintf(file,"acscale=%-6.2f\n",lat->acscale);
   if (lat->vocab!=NULL) fprintf(file,"vocab=%s\n",lat->vocab);
   if (lat->hmms!=NULL) fprintf(file,"hmms=%s\n",lat->hmms);
   if (lat->logbase != 1.0) fprintf(file,"base=%f\n",lat->logbase);
   if (lat->tscale != 1.0) fprintf(file,"tscale=%f\n",lat->tscale);

   /* First write all subsidiary sublattices */
   if (lat->subList!=NULL && !(format&HLAT_NOSUBS)) {
      /* Set all chain fields to NULL */
      lat->chain=NULL; SubLatList(lat,NULL,1);
      /* Set chain fields to make linked list */
      lat->chain=lat; list=SubLatList(lat,lat,1);
      /* Disconnect loop */
      list->chain=NULL;
      for (list=lat->chain;list!=NULL;list=list->chain) {
         if (list->subLatId==NULL){ 
            HRError(8253,"WriteLattice: Sublats must be labelled");
            return(FAIL);
         }
         if(WriteOneLattice(list,file,format)<SUCCESS){
            return(FAIL);
         }
      }
   }
   id=lat->subLatId;
   lat->subLatId=NULL;
   if(WriteOneLattice(lat,file,format)<SUCCESS){
      return(FAIL);
   }
   lat->subLatId=id;
   return(SUCCESS);
}


/* ------------------------ Lattice Input ------------------------- */

typedef enum {UNK_FIELD, STR_FIELD, INT_FIELD, FLT_FIELD} LatFieldType;

/* CheckStEndNodes: lattice must only have one start and one end node */
static ReturnStatus CheckStEndNodes(Lattice *lat)
{
   int i,st,en;
   NodeId ln;

   st=en=0;
   for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++){
      if (ln->pred==NARC) ++st;
      if (ln->foll==NARC) ++en;
   }
   if (st != 1){
      HRError(-8252,"CheckStEndNodes: lattice has %d start nodes",st);
      return(FAIL);
   }
   if (en != 1){
      HRError(-8252,"CheckStEndNodes: lattice has %d end nodes",en);
      return(FAIL);
   }
   return(SUCCESS);
}

/* GetNextFieldName: put field name into buf and delimiter into del */
static char *GetNextFieldName(char *buf, char *del, Source *src)
{
   int ch,i;
   char *ptr;

   buf[0]=0;
   ch=GetCh(src);
   while (isspace(ch) && ch!='\n')
      ch=GetCh(src);
   if (ch==EOF) {
      ptr=NULL;
   }
   else if (ch=='\n') {
      buf[0]='\n';buf[1]=0;ptr=buf;
   }
   else if (!isalnum(ch) && ch != '.') {
      if (ch!='#') 
         HError(8250,"GetNextFieldName: Field name expected");
      while (ch!='\n' && ch!=EOF) ch=GetCh(src);
      buf[0]='\n';buf[1]=0;ptr=buf;
   }
   else if (ch != '.') {
      i=0;
      while(isalnum(ch)) {
         buf[i++]=ch;
         ch=GetCh(src);
         if (ch==EOF)
            HError(8250,"GetNextFieldName: EOF whilst reading field name");
      }
      buf[i]=0;
      if (ch!='=' && ch!='~')
         HError(8250,"GetNextFieldName: Field delimiter expected %s|%c|",buf,ch);
      *del=ch;
      ptr=buf;
   }
   else {
      buf[0]='.';buf[1]=0;ptr=buf;
   }
   return(ptr);
}  

/* GetFieldValue: into buf and return type */
static LatFieldType GetFieldValue(char *buf, Source *src, int buflen)
{
   static char tmp[MAXSTRLEN*10];
   int ch;
  
   ch=GetCh(src);
   if (isspace(ch) || ch==EOF)
      HError(8250,"GetFieldValue: Field value expected");
   UnGetCh(ch,src);
   if (buf==NULL)
      ReadStringWithLen(src,tmp,MAXSTRLEN*10);
   else
     if(!buflen)  ReadString(src,buf);
     else ReadStringWithLen(src,buf,buflen);
   if (src->wasQuoted)
      return(STR_FIELD);
   else
      return(UNK_FIELD);
}

/* ParseNumber: if buf is number convert it and return in rval */
static LatFieldType ParseNumber(double *rval,char *buf)
{
   char *ptr;
   double val;
   LatFieldType type;

   type=STR_FIELD;
   if (isdigit((int) buf[0]) || 
       ((buf[0]=='-' || buf[0]=='+') && 
        isdigit((int) buf[1]))) {
      val=strtod(buf,&ptr);
      if (ptr != buf) {
         type=INT_FIELD;
         if (strchr(buf,'.') != NULL) {
            type=FLT_FIELD;
            *rval=val;
         }
         else
            *rval=val;
      }
   }
   return(type);
}

/* GetIntField: return integer field */
static int GetIntField(char ntype,char del,char *vbuf,Source *src)
{
   int vtype,iv;
   double rv;
   int fldtype;
  
   if (del=='=') {
      if ((vtype=GetFieldValue(vbuf,src,0))==STR_FIELD)
         HError(8250,"GetIntField: %c field expects numeric value (%s)",
                ntype,vbuf);
      if ((fldtype = ParseNumber(&rv,vbuf))!=INT_FIELD)
         HError(8250,"GetIntField: %c field expects integer value (%s==%d)",
                ntype,vbuf,fldtype);
   }
   else {
      if (!ReadInt(src,&iv,1,TRUE))
         HError(8250,"GetIntField: Could not read integer for %c field",ntype);
      rv=iv;
   }
   return((int)rv);
}

/* GetFltField: return float field */
static double GetFltField(char ntype,char del,char *vbuf,Source *src)
{
   int vtype;
   double rv;
   float fv;
  
   if (del=='=') {
      if ((vtype=GetFieldValue(vbuf,src,0))==STR_FIELD)
         HError(8250,"GetFltField: %c field expects numeric value (%s)",ntype,vbuf);
      if (ParseNumber(&rv,vbuf)==STR_FIELD)
         HError(8250,"GetFltField: %c field expects numeric value (%s)",ntype,vbuf);
   }
   else {
      if (!(ReadFloat(src,&fv,1,TRUE)))
         HError(8250,"GetFltField: Could not read float for %c field",ntype);
      rv=fv;
   }
   return(rv);
}

static int ReadAlign(Lattice *lat,LArc *la,char *buf)
{
   LAlign *lal;
   char *p,*str;
   int i,n,c;

   for(n=-1,p=buf;*p;p++) if (*p==':') n++;
   if (n<1) return(0);

   la->lAlign=(LAlign *) New(lat->heap,sizeof(LAlign)*n);
   
   for(i=0,lal=la->lAlign,p=buf;i<n;i++,lal++) {
      if (*p!=':')
         HError(8250,"ReadAlign: ':' Expected at start of field %d in %s",
                i,buf);
      for (str=++p;*p!=':' && *p!=',';p++)
         if (*p==0) HError(8250,"ReadAlign: Unexpected end of field %s",buf);
      c=*p;*p=0;lal->label=GetLabId(str,TRUE);
      if ((str=strchr(lal->label->name,'['))!=NULL)
         lal->state=atoi(str+1);
      else lal->state=-1;
      *p=c;
      if (*p==',') {
         str=p+1;
         lal->dur=strtod(str,&p);
         if (p==str || (*p!=':' && *p!=',')) 
            HError(8250,"ReadAlign: Cannot read duration %d from field %s",
                   i,buf);
      }
      if (*p==',') {
         str=p+1;
         lal->like=strtod(str,&p);
         if (p==str || *p!=':') 
            HError(8250,"ReadAlign: Cannot read like %d from field %s",i,buf);
      }
   }
   return(n);
}

/* ReadOneLattice: Read (one level) of lattice from file */
Lattice *ReadOneLattice(Source *src, MemHeap *heap, Vocab *voc, 
                               Boolean shortArc, Boolean add2Dict)
{
   int i,s,e,n,v=0,nn,na;
   Lattice *lat;
   LNode *ln;
   LArc *la;
   Word wordId;
   double time,aclike,lmlike;
   double prlike;
   char nbuf[132],vbuf[132],*ptr,ntype,del;
#define DBUFLEN 4096
   char dbuf[DBUFLEN];
   double lmscl=1.0, lmpen=0.0, acscl=1.0, prscl=1.0;
   float logbase = 1.0, tscale = 1.0;

   char *uttstr,*lmnstr,*vocstr,*hmmstr,*sublatstr,*tag;
   SubLatDef *subLatId = NULL;

   lat = (Lattice *) New(heap,sizeof(Lattice));
   lat->heap=heap; lat->subLatId=NULL; lat->chain=NULL;
   lat->voc=voc; lat->refList=NULL; lat->subList=NULL;

   /* Initialise default header values */
   nn=0;na=0; uttstr=lmnstr=vocstr=hmmstr=sublatstr=NULL;
   /* Process lattice header */
   while((ptr=GetNextFieldName(nbuf,&del,src))) {
      if (nbuf[0]=='\n') {
         if (na != 0 && nn != 0) break;
      }
      else if (strlen(ptr)==1) {
         ntype=*ptr;
         switch(ntype) {
         case 'N':
            nn=GetIntField('N',del,vbuf,src);
            break;
         case 'L':
            na=GetIntField('L',del,vbuf,src);
            break;
         default:
            GetFieldValue(0,src,0);
            break;
         }
      }
      else {
         if (!strcmp(ptr,"UTTERANCE"))
            GetFieldValue(vbuf,src,0),uttstr=CopyString(heap,vbuf);
         else if (!strcmp(ptr,"SUBLAT"))
            GetFieldValue(vbuf,src,0),sublatstr=CopyString(heap,vbuf);
         else if (!strcmp(ptr,"vocab"))
            GetFieldValue(vbuf,src,0),vocstr=CopyString(heap,vbuf);
         else if (!strcmp(ptr,"hmms"))
            GetFieldValue(vbuf,src,0),hmmstr=CopyString(heap,vbuf);
         else if (!strcmp(ptr,"lmname"))
            GetFieldValue(vbuf,src,0),lmnstr=CopyString(heap,vbuf);
         else if (!strcmp(ptr,"wdpenalty"))
            lmpen=GetFltField('p',del,vbuf,src);
         else if (!strcmp(ptr,"lmscale"))
            lmscl=GetFltField('s',del,vbuf,src);
         else if (!strcmp(ptr,"prscale"))
            prscl=GetFltField('s',del,vbuf,src);
         else if (!strcmp(ptr,"acscale"))
            acscl=GetFltField('a',del,vbuf,src);
         else if (!strcmp(ptr,"base"))
            logbase=GetFltField('b',del,vbuf,src);
         else if (!strcmp(ptr,"tscale"))
            tscale=GetFltField('t',del,vbuf,src);
         else
            GetFieldValue(NULL,src,0);
      }
   }

   if(ptr == NULL){
      /* generic memory clearing routine */
      Dispose(heap, lat);
      HRError(8250,"ReadLattice: Premature end of lattice file before header");
      return(NULL);
   }

   /* Initialise lattice based on header information */
   lat->nn=nn;
   lat->na=na;
   lat->utterance=uttstr;lat->vocab=vocstr;lat->hmms=hmmstr;
   lat->net=lmnstr;lat->lmscale=lmscl;lat->wdpenalty=lmpen;
   lat->acscale = acscl;
   lat->logbase = logbase;
   lat->tscale = tscale;
   lat->framedur=0;
   lat->prscale=prscl;

   if (logbase < 0.0)
      HError (8251, "ReadLattice: Illegal log base in lattice");

   /* Set format to indicate type and default word label position */
   lat->format=(shortArc?HLAT_SHARC|HLAT_ALABS:HLAT_ALABS);

   /* Presence of SUBLAT=id string indicates more to come */
   lat->subList=NULL; lat->chain=NULL;
   if (sublatstr!=NULL) lat->subLatId = GetLabId(sublatstr,TRUE);
   else lat->subLatId = NULL;

   /* Allocate and initiailise nodes/arcs */
   lat->lnodes=(LNode *) New(heap, sizeof(LNode)*nn);
   if (shortArc) 
      lat->larcs=(LArc *) New(heap, sizeof(LArc_S)*na);
   else 
      lat->larcs=(LArc *) New(heap, sizeof(LArc)*na);

   for(i=0, ln=lat->lnodes; i<nn; i++, ln++) {
      ln->hook=NULL;
      ln->pred=NULL;
      ln->foll=NULL;
      ln->score=0.0;
   }
   for(i=0, la=lat->larcs; i<na; i++, la=NextLArc(lat,la)) {
      la->lmlike=0.0;
      la->start=la->end=NNODE;
      la->farc=la->parc=NARC;
   }
   if (!shortArc)
      for(i=0, la=lat->larcs; i<na; i++, la=NextLArc(lat,la)) {
         la->aclike=la->prlike=la->score=0.0;
         la->nAlign=0;
         la->lAlign=NULL;
      }
   
   do {
      if ((ptr=GetNextFieldName(nbuf,&del,src)) == NULL)
         break;
      /* Recognised line types have only one character names */
      if (strlen(ptr)==1) 
         ntype=*ptr;
      else 
         ntype=0;
      if (ntype == '.') {
         ptr = NULL;
         break;
      }
      switch(ntype) {
      case '\n': break;
      case 'I':
         n=GetIntField('I',del,vbuf,src);
         if (n < 0 || n >= lat->nn){
            Dispose(heap, lat);
            HRError(8251,"ReadLattice: Lattice does not contain node %d",n);
            return(NULL);
         }
         ln=lat->lnodes+n;
         if (ln->hook!=NULL){
            Dispose(heap, lat);
            HRError(8251,"ReadLattice: Duplicate info info for node %d",n);
            return(NULL);
         }
         time=0.0;wordId=voc->nullWord;tag=NULL;v=-1;
         while((ptr=GetNextFieldName(nbuf,&del,src)) != NULL) {
            if (nbuf[0]=='\n') break;
            else {
               if (strlen(ptr)>=1) 
                  ntype=*ptr;
               else 
                  ntype=0;
               switch(ntype) {
               case 't':
                  time=GetFltField('t',del,vbuf,src);
                  time *= tscale;
                  lat->format |= HLAT_TIMES;
                  break;
               case 'W':
                  GetFieldValue(vbuf,src,0);
                  wordId=GetWord(voc,GetLabId(vbuf,add2Dict),add2Dict);
                  if (wordId==NULL){
                     Dispose(heap, lat);
                     HRError(8251,"ReadLattice: Word %s not in dict",vbuf);
                     return(NULL);
                  }
                  break;
               case 's':
                  GetFieldValue(vbuf,src,0);
                  tag=CopyString(heap,vbuf);
                  lat->format |= HLAT_TAGS;
                  break;
               case 'L':
                  GetFieldValue(vbuf,src,0);
                  wordId=voc->subLatWord;
                  if((subLatId=AdjSubList(lat,GetLabId(vbuf,TRUE),NULL,+1))==NULL) {
                     HRError(8251,"ReadLattice: AdjSubLat failed");
                     return(NULL);
                  }

                  break;
               case 'v':
                  lat->format |= HLAT_PRON;
                  v=GetIntField('v',del,vbuf,src);
                  break;
               default:
                  GetFieldValue(0,src,0);
                  break;
               }
            }
         }
         if (wordId != voc->nullWord)
            lat->format &= ~HLAT_ALABS;
         ln->time=time;
         ln->word=wordId;
         ln->tag=tag;
         ln->v=v;
         if (wordId == voc->subLatWord)
            ln->sublat = subLatId;
         else
            ln->sublat = NULL;
         ln->hook=ln;
         nn--;
         break;
      case 'J':
         n=GetIntField('I',del,vbuf,src);
         if (n<0 || n>=lat->na){
            Dispose(heap, lat);
            HRError(8251,"ReadLattice: Lattice does not contain arc %d",n);
            return(NULL);
         }
         la=NumbLArc(lat,n);
         if (la->start!=NULL){
            Dispose(heap, lat);
            HRError(8251,"ReadLattice: Duplicate info for arc %d",n);
            return(NULL);
         }
         s=e=v=-1; wordId=NULL; aclike=lmlike=0.0;
         prlike=0.0;
         while ((ptr=GetNextFieldName(nbuf,&del,src))) {
            if (nbuf[0]=='\n') break;
            else {
               if (strlen(ptr)>=1) ntype=*ptr;
               else ntype=0;
               switch(ntype)
                  {
                  case 'S':
                     s=GetIntField('S',del,vbuf,src);
                     if (s<0 || s>=lat->nn){
                        Dispose(heap, lat);
                        HRError(8251,"ReadLattice: Lattice does not contain start node %d",s);
                        return(NULL);
                     }
                     break;
                  case 'E':
                     e=GetIntField('E',del,vbuf,src);
                     if (e<0 || e>=lat->nn){
                        Dispose(heap, lat);
                        HRError(8251,"ReadLattice: Lattice does not contain end node %d",e);
                        return(NULL);
                     }
                     break;
                  case 'W':
                     GetFieldValue(vbuf,src,0);
                     wordId=GetWord(voc,GetLabId(vbuf,add2Dict),add2Dict);
                     if (wordId==NULL || wordId==voc->subLatWord){
                        Dispose(heap, lat);
                        HRError(8251,"ReadLattice: Word %s not in dict",
                                vbuf);
                        return(NULL);
                     }
                     break;
                  case 'v':
                     lat->format |= HLAT_PRON;
                     v=GetIntField('v',del,vbuf,src);
                     break;
                  case 'a':
                     lat->format |= HLAT_ACLIKE;
                     aclike=GetFltField('a',del,vbuf,src);
                     aclike = ConvLogLikeFromBase(logbase, aclike);
                     break;
                  case 'l':
                     lat->format |= HLAT_LMLIKE;
                     lmlike=GetFltField('l',del,vbuf,src);
                     lmlike = ConvLogLikeFromBase(logbase, lmlike);
                     break;
                  case 'r':
                     lat->format |= HLAT_PRLIKE;
                     prlike=GetFltField('r',del,vbuf,src);
                     prlike = ConvLogLikeFromBase(logbase, prlike);
                     break;
                  case 'd':
                     lat->format |= HLAT_ALIGN;
                     GetFieldValue(dbuf,src,DBUFLEN);
                     if (!shortArc)
                        la->nAlign=ReadAlign(lat,la,dbuf);
                     break;
                  default:
                     GetFieldValue(0,src,0);
                     break;
                  }
            }
         }
         if (s<0 || e<0 ||(wordId==NULL && (lat->format&HLAT_ALABS))){
            Dispose(heap, lat);
            HRError(8250,"ReadLattice: Need to know S,E [and W] for arc %d",n);
            return(NULL);
         }
         la->start=lat->lnodes+s;
         la->end=lat->lnodes+e;
         la->lmlike=lmlike;
           
         if ((lat->format&HLAT_ALABS) && la->end->word == voc->nullWord){
            la->end->word=wordId;
	    la->end->v = v;
	 }
         if (wordId != NULL && la->end->word != wordId){
            Dispose(heap, lat);
            HRError(8251,"ReadLattice: Lattice arc (%d) W field (%s) different from node (%s)",  n,wordId->wordName->name,la->end->word->wordName->name);
            return(NULL);
         }

         la->farc=la->start->foll;
         la->parc=la->end->pred;
         la->start->foll=la;
         la->end->pred=la;
         if (!shortArc) {
            la->aclike=aclike;
            la->prlike=prlike;
         }
         na--;
         break;
      default:
         GetFieldValue(0,src,0);
         while ((ptr=GetNextFieldName(nbuf,&del,src))) {
            if (nbuf[0]=='\n') break;
            else GetFieldValue(0,src,0);
         }
         break;
      }
   }
   while(ptr != NULL);
   if (na!=0 || (nn!=0 && nn!=lat->nn)){
      Dispose(heap, lat);
      HRError(8250,"ReadLattice: %d Arcs unseen and %d Nodes unseen",na,nn);
      return(NULL);
   }

   if(CheckStEndNodes(lat)<SUCCESS){
      Dispose(heap, lat);
      HRError(8250,"ReadLattice: Start/End nodes incorrect",na,nn);
      return(NULL);
   }

   for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      ln->hook=NULL;
   if (shortArc) lat->format&=~(HLAT_ACLIKE|HLAT_PRLIKE|HLAT_ALIGN);
   return(lat);
}


/* EXPORT->ReadLattice: Read lattice from file - calls ReadOneLattice */
/*                      for each level of a multi-level lattice file  */
Lattice *ReadLattice(FILE *file, MemHeap *heap, Vocab *voc, 
                     Boolean shortArc, Boolean add2Dict)
{
   Lattice *lat,*list,*fLat;
   Source source;
 
   AttachSource(file,&source);

   if((lat=ReadOneLattice(&source,heap,voc,shortArc,add2Dict))==NULL)
      return NULL;

   if (lat->subLatId!=NULL) {
      /* Need to preserve first lattice to return */
      fLat=lat; lat = (Lattice *) New(heap,sizeof(Lattice)); *lat=*fLat;
      do {
         /* Add SUBLAT to hash table for later lookup */
         GetSubLat(lat->subLatId,lat);
         if((lat=ReadOneLattice(&source,heap,voc,shortArc,add2Dict))==NULL){
            Dispose(heap, fLat); /*fLat points to 1st thing on heap*/
            return NULL;
         }               

      }
      while(lat->subLatId!=NULL);

      /* Clear hash table */
      GetSubLat(NULL,NULL); 
      /* Set all chain fields to NULL */
      lat->chain=NULL; 
      SubLatList(lat,NULL,1);
      /* Set chain fields to make linked list */
      lat->chain=lat; 
      list=SubLatList(lat,lat,1);
      /* Disconnect loop */
      list->chain=NULL;
      /* Copy last to first Lattices to ensure lat is first thing on stack */
      *fLat=*lat; lat=fLat;
   }
   return(lat);
}

/* ----------------------- ExpandLattice -------------------------- */

static DictEntry specialNull;

/* ExpandedLatticeSize: Calculate the size of the new lattice */
static void ExpandedLatticeSize(Lattice *lat, int *nNodes,int *nArcs)
{
   int i; 
   NodeId thisNode;

   for (i=0; i<lat->nn; i++) {
      thisNode = lat->lnodes+i;
      if (thisNode->word == lat->voc->subLatWord && thisNode->sublat != NULL) {
         if (thisNode->tag != NULL) {
            *nNodes += 1;  
            *nArcs  += 1;
            if (sublatmarkers) {
               *nNodes += 1;  
               *nArcs  += 1;
            }
         }
         ExpandedLatticeSize(thisNode->sublat->lat,nNodes,nArcs);   
      }
   }
   *nNodes += lat->nn;  
   *nArcs  += lat->na;
}

/* CopyLattice: copy lattice from lat to newlat starting at offsets         */
/*              *newNodes and *newArcs - ignore NULL id words if ignoreNull */
void CopyLattice(Lattice *lat, Lattice *newlat, 
                 int *newNodes, int *newArcs, Boolean ignoreNull)
{
   int i,j;
   LNode *oldNode,*newNode;
   LArc *oldArc,*newArc;

   for (i=0,j=0; i< lat->nn; i++) {
      oldNode = lat->lnodes+i;
      if ((oldNode->word != &specialNull) || !ignoreNull) {
         newNode = newlat->lnodes+j+*newNodes;
         newNode->word = oldNode->word;
         newNode->tag = NULL;
         if (oldNode->tag != NULL)
            newNode->tag = oldNode->tag;
         newNode->sublat = oldNode->sublat;
         if (oldNode->foll != NULL) {
            newNode->foll=NumbLArc(newlat,
                                   *newArcs+LArcNumb(oldNode->foll,lat));
            for (oldArc=oldNode->foll; oldArc != NARC; oldArc = oldArc->farc) {
               newArc = NumbLArc(newlat,*newArcs+LArcNumb(oldArc,lat));
               newArc->start = newNode;
               newArc->lmlike = oldArc->lmlike;
               newArc->prlike = oldArc->prlike;
            }
         }
         else
            newNode->foll = NULL;
         if (oldNode->pred != NULL) {
            newNode->pred = NumbLArc(newlat,
                                     *newArcs+LArcNumb(oldNode->pred,lat));
            for (oldArc = oldNode->pred; oldArc != NARC; oldArc = oldArc->parc) {
               newArc = NumbLArc(newlat,*newArcs+LArcNumb(oldArc,lat));
               newArc->end = newNode;
               newArc->lmlike = oldArc->lmlike;
               newArc->prlike = oldArc->prlike;
            }
         }
         else
            newNode->pred = NULL;
         j++;
      }
   }
   for (i=0; i< lat->na; i++) {
      oldArc = NumbLArc(lat,i);
      newArc = NumbLArc(newlat,i+*newArcs);
      if (oldArc->farc != NULL)
         newArc->farc = NumbLArc(newlat,*newArcs+LArcNumb(oldArc->farc,lat));
      else
         newArc->farc = NULL;
      if (oldArc->parc != NULL)
         newArc->parc = NumbLArc(newlat,*newArcs+LArcNumb(oldArc->parc,lat));
      else
         newArc->parc = NULL;
   }
   *newNodes += j;
   *newArcs += lat->na;
}

/*  SubLattice: sub latStart/latEnd in place of thisNode in newlat */
void SubLattice(Lattice *newlat,NodeId thisNode, NodeId latStart,NodeId latEnd)
{
   ArcId thisArc;

   for (thisArc = thisNode->foll; thisArc != NULL; thisArc=thisArc->farc)
      thisArc->start = latEnd;
   for (thisArc = thisNode->pred; thisArc != NULL; thisArc=thisArc->parc)
      thisArc->end = latStart;
   latStart->pred = thisNode->pred;
   latEnd->foll = thisNode->foll;
   thisNode->pred = thisNode->foll = NARC;
   thisNode->word = &specialNull;
}


/* ExpandLattice: Expand all the subLats in newlat (recursively) */
static void ExpandLattice(Lattice *newlat, int nNodes, int nArcs)
{
   int i;
   NodeId thisNode;
   NodeId latStart,latEnd,node;
   ArcId arc;
   int newNodes, newArcs;
   int len;
   Lattice *subLat;

   for (i=0; i< nNodes; i++) {
      thisNode = newlat->lnodes+i;
      if (thisNode->word == newlat->voc->subLatWord) {
         newNodes = nNodes; newArcs = nArcs;
         subLat = thisNode->sublat->lat;
         CopyLattice(subLat,newlat,&newNodes,&newArcs,FALSE);
         if (thisNode->tag != NULL) {
            latStart = newlat->lnodes + nNodes +
               (FindLatStart(subLat)-subLat->lnodes);
            latEnd = newlat->lnodes + nNodes +
               +(FindLatEnd(subLat)-subLat->lnodes);
           
            if (sublatmarkers) {
               /* add sublat start marker */
               node = newlat->lnodes + newNodes;
               arc = NumbLArc(newlat,newArcs);
               node->word=GetWord(newlat->voc,GetLabId("!NULL", TRUE),TRUE);
               node->tag=SafeCopyString(newlat->heap,subLatStart);
               /* node->word=GetWord(newlat->voc,
                  GetLabId(subLatStart, TRUE),TRUE);
                  if (node->word->pron==NULL)
                  NewPron(newlat->voc,node->word,0,NULL,
                  node->word->wordName,1.0); */

               arc->start = node;
               arc->end = latStart;
               node->foll = arc;
               latStart->pred = node->foll;
               latStart = node;
             
               newNodes++; newArcs++;
             
               /* add sublat end marker */
               node = newlat->lnodes + newNodes;
               arc = NumbLArc(newlat,newArcs);
               node->word=GetWord(newlat->voc,GetLabId("!NULL", TRUE),TRUE);
               len = strlen(subLatEnd) + strlen(thisNode->tag) + 4;
               node->tag=(char *) New(newlat->heap,sizeof(char)*len);
               strcpy(node->tag, subLatEnd);
               strcat(node->tag, "-");
               strcat(node->tag, thisNode->tag);
               /* node->word=GetWord(newlat->voc,
                  GetLabId(subLatEnd, TRUE),TRUE);
                  if (node->word->pron==NULL)
                  NewPron(newlat->voc,node->word,0,NULL,
                  node->word->wordName,1.0);*/
             
               arc->start = latEnd;
               arc->end = node;
               latEnd->foll = arc;
               node->pred = latEnd->foll;
               node->foll = NARC;
               latEnd = node;
             
               newNodes++; newArcs++;
            }
            else {
               /* add a tagged !NULL node holding the sublat name tag */
               node = newlat->lnodes + newNodes;
               arc = NumbLArc(newlat,newArcs);
               arc->start = latEnd;
               arc->end = node;
               latEnd->foll = arc;
               node->foll=NARC;
               node->pred=arc;
               node->word=GetWord(newlat->voc,GetLabId("!NULL", TRUE),TRUE);
               node->tag=SafeCopyString(newlat->heap,thisNode->tag);
               latEnd = node; 
             
               newNodes++; newArcs++;
            }
         }
         else {
            latStart = newlat->lnodes + nNodes +
               (FindLatStart(subLat)-subLat->lnodes);
            latEnd = newlat->lnodes + nNodes +
               +(FindLatEnd(subLat)-subLat->lnodes);
         }
         SubLattice(newlat,thisNode,latStart,latEnd);
         nNodes = newNodes; nArcs = newArcs;
      }
   }
}

/* CountNonNullNodes: count the nodes with a non-NULL word id */
static int CountNonNullNodes(Lattice *lat)
{
   int i,count=0;
   NodeId thisNode;
   
   for (i=0; i< lat->nn; i++) {
      thisNode = lat->lnodes+i;
      if (thisNode->word != &specialNull) 
         count++;
   }
   return count;
}

/* EXPORT->ExpandMultiLevelLattice: Expand multi-level lattice lat  */
/*                                  into a single-level lattice.    */ 
Lattice *ExpandMultiLevelLattice(MemHeap *heap, Lattice *lat, Vocab *voc)
{
   Lattice *newlat;
   Lattice *final;
   int  nArcs, nNodes;
   int newArcs, newNodes;   

   nNodes = nArcs = 0;
   ExpandedLatticeSize(lat,&nNodes,&nArcs);        
   newlat = NewLattice(&gstack,nNodes,nArcs);
   newlat->voc = lat->voc;
   newArcs = newNodes = 0;
   CopyLattice(lat,newlat,&newNodes,&newArcs,FALSE);  /* copy the top level  */
   ExpandLattice(newlat,newNodes,newArcs);        /* expand all sub-lats */
   nNodes = CountNonNullNodes(newlat);
   final = NewILattice(heap,nNodes,nArcs,lat);              
   newArcs = newNodes = 0;
   CopyLattice(newlat,final,&newNodes,&newArcs,TRUE); /* remove NULL id nodes */
   Dispose(&gstack,newlat);

   final->subList=NULL;  /* Actually unnecessary */

   return final;
}


/* ------------------------ Misc Lattice Ops ------------------------- */

/* EXPORT->LatticeFromLabels: Create lattice from label list for alignment */
Lattice *LatticeFromLabels(LabList *ll,LabId bnd,Vocab *voc,MemHeap *heap)
{
   Lattice *lat;
   LNode *ln;
   LArc *la;
   LLink l;
   LabId labid;
   int i,n,N;

   N=CountLabs(ll);  /* total number of symbols */
   n=1;              /* index of first word in list */
   if (bnd!=NULL) N+=2,n--;
   lat=NewLattice(heap,N,N-1);
   lat->voc=voc;
   for (i=1,ln=lat->lnodes;i<=N;i++,ln++,n++) {
      if (bnd!=NULL && (i==1 || i==N)) {
         labid=bnd;
      }
      else {
         l=GetLabN(ll,n);
         labid=l->labid;
      }
      /* Node */
      ln->n=i-1;
      if ((ln->word=GetWord(voc,labid,FALSE))==NULL)
         HError(8220,"LatticeFromLabels: Word %s not defined in dictionary",
                labid->name);
      ln->v=-1;
      ln->time=0.0;
      ln->foll=ln->pred=NULL;
      ln->score=0.0;
      ln->hook=NULL;
      if (i!=1) {
         /* Arc */
         la=NumbLArc(lat,i-2);
         la->start=ln-1;
         la->end=ln;
         la->lmlike=0.0;
         la->prlike=0.0;
         la->farc=la->parc=NULL;
         la->end->pred=la->start->foll=la;
      }
   }
   return(lat);
}


/* EXPORT->NumNodeFoll: return num outgoing arcs from node n */
int NumNodeFoll(NodeId n)
{
   int c = 0;
   ArcId a;
   
   for (a = n->foll; a != NARC; a=a->farc) ++c;
   return c;
}

/* EXPORT->NumNodePred: return num outgoing arcs from node n */
int NumNodePred(NodeId n)
{
   int c = 0;
   ArcId a;
   
   for (a = n->pred; a != NARC; a=a->parc) ++c;
   return c;
}


/* EXPORT->FindLatStart: find and return the lattice start node */
NodeId FindLatStart(Lattice *lat)
{
   int i;
   NodeId ln;

   for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      if (ln->pred==NARC)
         return ln;
   HError(8252,"FindLatStart: lattice has no start node");
   return(NARC);
}

/* EXPORT->FindLatEnd: find and return the lattice end node */
NodeId FindLatEnd(Lattice *lat)
{
   int i;
   NodeId ln;

   for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      if (ln->foll==NARC)
         return ln;
   HError(8252,"FindLatEnd: lattice has no end node");
   return(NARC);
}



/* --------------------------- Network Routines ------------------------ */

static void PrintNode(NetNode *node,HMMSet *hset)
{
   printf("Node[%05d] ",(((unsigned) node)/sizeof(NetNode))%100000);
   if (node->type & n_hmm)
      printf("{%s}\n",HMMPhysName(hset,node->info.hmm));
   else if (node->type == n_word && node->info.pron==NULL) {
      printf("NULL");
      if (node->tag != NULL)
         printf(" ... tag=%s\n",node->tag);
      else
         printf("\n");
   }
   else if (node->type == n_word) {
      printf("%s",node->info.pron->word->wordName->name);
      if (node->tag != NULL)
         printf(" ... tag=%s\n",node->tag);
      else
         printf("\n");
   }
   else
      printf("{%d}\n",node->type);
   fflush(stdout);

}

static void PrintLinks(NetLink *links,int nlinks)
{
   int i;

   for (i=0; i<nlinks; i++) {
      printf("    %-2d: -> [%05d] == %7.3f\n",i,
             (((unsigned) links[i].node)/sizeof(NetNode)%100000),
             links[i].like);
      fflush(stdout);
   }
}

static void PrintChain(Network *wnet,HMMSet *hset)
{
   NetNode *thisNode;
   
   thisNode = wnet->chain;
   while (thisNode != NULL) {
      PrintNode(thisNode,hset);
      PrintLinks(thisNode->links,thisNode->nlinks);
      thisNode = thisNode->chain;
   }
}

static Boolean IsWd0Link(NetLink *link)
{
   int i;
   NetNode *nextNode;

   nextNode = link->node;
   if ((nextNode->type&n_nocontext) == n_word)
      return TRUE;
   if (nextNode->type & n_tr0) {
      for (i = 0; i < nextNode->nlinks; i++)
         if (IsWd0Link(&nextNode->links[i]))
            return TRUE;
      return FALSE;
   }
   else
      return FALSE;
}

static void AddChain(Network*net, NetNode *hd) 
{
   NetNode *tl;

   if (hd == NULL) 
      return;
   tl = hd;
   while (tl->chain != NULL)
      tl = tl->chain;
   tl->chain = net->chain;
   net->chain = hd;
}

/* 
   Networks are created directly from lattices (which may be
   of type shortArc to minimise lattice storage requirements).
   
   Cross word context dependent networks are created in an
   automagic manner from the hmmlist and monophone dictionary.

   Contexts
   -1 == context free - skip this phone when determining context
    0 == context independent - matches any context - or undefined
    n == unique context identifier

   Many labids may map to a single context (generalised contexts)
   or there may be single context per phone (triphone contexts)

   There is currently no way to determine automatically which
   context a model belongs to for generalised contexts.
*/

typedef struct pronholder
{
   LNode *ln;       /* Node that created this instance */
   Pron pron;       /* Actual pronunciation */
   short nphones;   /* Number of phones for this instance */
   LabId *phones;   /* Phone sequence for the instance */
   LogFloat fct;    /* LM likelihood to be factored into each phone */
   int ic;          /* Initial context - cache saves finding for all links */
   int fc;          /* Final context - cache saves finding for all links */
   Boolean fci;     /* Final phone context independent */
   Boolean tee;     /* TRUE if word consists solely of tee models */
   int clen;        /* Number of non-cf phones in pronunciation */
   NetNode **lc;    /* Left contexts - linked to word initial models */
   NetNode **rc;    /* Right contexts - linked to word end nodes */
   int nstart;      /* Number of models in starts chain */
   int nend;        /* Number of models in ends chain */
   NetNode *starts; /* Chain of initial models */
   NetNode *ends;   /* Chain of final models */
   NetNode *chain;  /* Chain of other nodes in word */
   struct pronholder *next;
}
PronHolder;

#define HCI_CXT_BLOCKSIZE 256

/* Word end nodes are accessed through a hash table for fast access */
#define WNHASHSIZE 5701
static NetNode *wnHashTab[WNHASHSIZE];

static HMMSetCxtInfo *NewHMMSetCxtInfo(HMMSet *hset, Boolean frcCxtInd)
{
   HMMSetCxtInfo *hci;

   hci=(HMMSetCxtInfo *) New(&gcheap,sizeof(HMMSetCxtInfo));
   hci->hset=hset;
   hci->nc=hci->xc=hci->nci=hci->ncf=0;
   hci->sLeft=hci->sRight=FALSE;
   if (frcCxtInd) {
      hci->cxs=hci->cis=hci->cfs=NULL;
   }
   else {
      hci->cxs=(LabId *) New(&gcheap,sizeof(LabId)*HCI_CXT_BLOCKSIZE);
      hci->cis=(LabId *) New(&gcheap,sizeof(LabId)*HCI_CXT_BLOCKSIZE);
      hci->cfs=(LabId *) New(&gcheap,sizeof(LabId)*HCI_CXT_BLOCKSIZE);      
      hci->cxs[0]=hci->cis[0]=hci->cfs[0]=GetLabId("<undef>",TRUE);
   }

   return(hci);
}

/* Binary search to find labid in n element array */
static int BSearch(LabId labid, int n,LabId *array)
{
   int l,u,c;

   l=1;u=n;
   while(l<=u) {
      c=(l+u)/2;
      if (array[c]==labid) return(c);
      else if (array[c]<labid) l=c+1; 
      else u=c-1;
   }
   return(0);
}

/* Binary search to find labid in n element array */
static int BAddSearch(HMMSetCxtInfo *hci,LabId labid, int *np,LabId **ap)
{
   LabId *array;
   int l,u,c;

   array=*ap;
   l=1;u=*np;
   while(l<=u) {
      c=(l+u)/2;
      if (array[c]==labid) return(c);
      else if (array[c]<labid) l=c+1; 
      else u=c-1;
   }
   if (((*np+1)%HCI_CXT_BLOCKSIZE)==0) {
      LabId *newId;

      newId=(LabId *) New(&gcheap,sizeof(LabId)*(*np+1+HCI_CXT_BLOCKSIZE));
      for (c=1;c<=*np;c++)
         newId[c]=array[c];
      Dispose(&gcheap,array);
      *ap=array=newId;
   }
   for (c=1;c<=*np;c++)
      if (labid<array[c]) break;
   for (u=(*np)++;u>=c;u--)
      array[u+1]=array[u];
   array[c]=labid;
   return(c);
}

int AddHCIContext(HMMSetCxtInfo *hci,LabId labid)
{
   int c;
   c=BAddSearch(hci,labid,&hci->nc,&hci->cxs);
   return(c);
}

/* Return context defined by given labid (after triphone context stripping) */
int GetHCIContext(HMMSetCxtInfo *hci,LabId labid)
{
   LabId cxt;
   char buf[80];
   int c;

   if (hci->nc==0) return(0);
   strcpy(buf,labid->name);
   TriStrip(buf);
   cxt=GetLabId(buf,FALSE);
   
   c=BSearch(cxt,hci->nc,hci->cxs);
   if (c>0) return(c);
   c=BSearch(cxt,hci->ncf,hci->cfs);
   if (c>0) {
      if (hci->xc>0 || !cfWordBoundary) return(-1);
      else return(0); /* Context free are word boundaries */
   }
   return(0);
}

/* Check to see if CI model exists for labid */
Boolean IsHCIContextInd(HMMSetCxtInfo *hci,LabId labid)
{
   int c;
   if (hci->nc==0) return(TRUE);
   c=BSearch(labid,hci->nci,hci->cis);
   if (c>0) return(TRUE);
   else return(FALSE);
}

/* Search through pron for left context for phone in position pos */
/*  If beginning of word is reached return lc */
static int FindLContext(HMMSetCxtInfo *hci,PronHolder *p,int pos,int lc)
{
   int i,c;

   for (i=pos-1,c=-1;i>=0;i--)
      if ((c=GetHCIContext(hci,p->phones[i]))>=0)
         break;
   if (c<0) c=lc;
   return(c);
}

/* Search through pron for right context for phone in position pos */
/*  If end of word is reached return rc */
static int FindRContext(HMMSetCxtInfo *hci,PronHolder *p,int pos,int rc)
{
   int i,c;

   for (i=pos+1,c=-1;i<p->nphones;i++)
      if ((c=GetHCIContext(hci,p->phones[i]))>=0)
         break;
   if (c<0) c=rc;
   return(c);
}

/* Determine if dictionary contains any models not in list */
/* Notionally checks whether dictionary is phone or model based */
static Boolean ClosedDict(Vocab *voc,HMMSet *hset)
{
   Word word;
   Pron pron;
   int i,h;

   for (h=0; h<VHASHSIZE; h++)
      for (word=voc->wtab[h]; word!=NULL; word=word->next)
         for (pron=word->pron; pron!=NULL; pron=pron->next)
            for (i=0;i<pron->nphones;i++)
               if (FindMacroName(hset,'l',pron->phones[i])==NULL)
                  return(FALSE);
   return(TRUE);
}

/* Defines contexts from vocabulary and hmmset */
static int DefineContexts(HMMSetCxtInfo *hci)
{
   MLink ml,il;
   LabId labid;
   char buf[80],*ptr;
   int h,c,*temp;

   hci->nc=0; hci->sLeft=hci->sRight=FALSE;
   /* Scan for all contexts that appear */
   for (h=0; h<MACHASHSIZE; h++)
      for (ml=hci->hset->mtab[h]; ml!=NULL; ml=ml->next)
         if (ml->type=='l') {
            strcpy(buf,ml->id->name);
            TriStrip(buf);
            labid=GetLabId(buf,FALSE);
            il=FindMacroName(hci->hset,'l',labid);
            
            /* Start by adding all models then check later if CI */
            BAddSearch(hci,labid,&hci->nci,&hci->cis);
            
            /* Check for left context */
            if (strchr(ml->id->name,'-')!=NULL) {
               strcpy(buf,ml->id->name);
               strchr(buf,'-')[0]=0;
               labid=GetLabId(buf,TRUE);
               AddHCIContext(hci,labid);
               hci->sLeft=TRUE;
            }
            
            /* Check for right context */
            if (strchr(ml->id->name,'+')!=NULL) {
               strcpy(buf,ml->id->name);
               ptr=strchr(buf,'+');
               labid=GetLabId(ptr+1,TRUE);
               AddHCIContext(hci,labid);
               hci->sRight=TRUE;
            }
         }
   temp=(int *) New(&gstack,sizeof(int)*hci->nci);temp--;
   for (c=1;c<=hci->nci;c++) temp[c]=0;

   /* Otherwise scan for all contexts that appear */
   for (h=0; h<MACHASHSIZE; h++)
      for (ml=hci->hset->mtab[h]; ml!=NULL; ml=ml->next)
         if (ml->type=='l') {
            strcpy(buf,ml->id->name);
            TriStrip(buf);
            labid=GetLabId(buf,FALSE);
            il=FindMacroName(hci->hset,'l',labid);
            c=BSearch(labid,hci->nci,hci->cis);
            if (c!=0 && il!=ml)
               temp[c]++;
         }

   for (c=1;c<=hci->nci;c++)
      if (temp[c]!=0) hci->cis[c]=NULL;
   Dispose(&gstack,temp+1);
   
   for (c=1,h=1;c<=hci->nci;c++,h++) {
      for (;h<=hci->nci;h++) if (hci->cis[h]!=NULL) break;
      if (h>hci->nci) break;
      hci->cis[c]=hci->cis[h];
   }
   hci->nci=c-1;

   /* Now define which models represent which contexts */
   /*  Do nothing for assumed one-to-one corespondance */
   /* And finally set all other to context free */
      
   for (h=0; h<MACHASHSIZE; h++)
      for (ml=hci->hset->mtab[h]; ml!=NULL; ml=ml->next)
         if (ml->type=='l') {
            c=GetHCIContext(hci,ml->id);
            if (c==0) {
               if (!IsHCIContextInd(hci,ml->id)) 
                  HError(8230,"DefineContexts: Context free models must be context independent (%s)",ml->id->name);
               BAddSearch(hci,ml->id,&hci->ncf,&hci->cfs);
            }
         }
   return(hci->nc);
}

/* Return labid for given context */
static LabId ContextName(HMMSetCxtInfo *hci,int c)
{
   if (c<0 || c>hci->nc)
      HError(8290,"ContextName: Context %d not defined (-1..%d)",c,hci->nc);

   return(hci->cxs[c]);
}

/* Counting variables:  Number of */
/* Word-internal nodes, Word-initial nodes, Word final-nodes, chainNodes */
/*  Null nodes, context-free nodes, word end nodes */
/* Word-internal links, Cross-word links and null-word links */
int nwi=0,nin=0,nfi=0,ncn=0,nll=0,ncf=0,nwe=0,nil=0,nxl=0,nnl=0;

/* Use hash table to lookup word end node */
static NetNode *FindWordNode(MemHeap *heap,Pron pron,
                             PronHolder *pInst,NetNodeType type)
{
   union {
      Ptr ptrs[3];
      unsigned char chars[12];
   }
   un;
   unsigned int hash,i;
   NetNode *node;

   hash=0;
   un.ptrs[0]=pron;un.ptrs[1]=pInst;un.ptrs[2]=(Ptr)type;
   for (i=0;i<12;i++)
      hash=((hash<<8)+un.chars[i])%WNHASHSIZE;

   for (node=wnHashTab[hash];node!=NULL;node=node->chain)
      if (node->info.pron==pron && node->inst==(NetInst*)pInst &&
          node->type==type) break;

   if (node==NULL) {
      nwe++;
      if (heap==NULL)
         HError(8291,"FindWordNode: Node %s[%d] %d not created",
                pron->word->wordName->name,pron->pnum,type);
      node=(NetNode *) New(heap,sizeof(NetNode));
      node->info.pron=pron;
      node->type=type;
      node->inst=(NetInst*)pInst;
      node->nlinks=0;
      node->links=NULL;
      node->tag=NULL;
      node->aux=0;
      node->chain=wnHashTab[hash];
      wnHashTab[hash]=node;
   }
   return(node);
}



/* Create NetNode (and optionally NetLinks as well) */
static NetNode *NewNode(MemHeap *heap,HLink hmm,int nlinks)
{
   NetNode *node;

   node=(NetNode *)New(heap,sizeof(NetNode));
   node->type=(hmm->transP[1][hmm->numStates]>LSMALL?
               (n_hmm|n_tr0) : n_hmm );
   node->info.hmm=hmm;
   node->inst=NULL;node->chain=NULL;
   node->nlinks=nlinks;
   node->tag=NULL;
   node->aux=0;
   if (nlinks==0)
      node->links=NULL;
   else
      node->links=(NetLink*) New(heap,sizeof(NetLink)*node->nlinks);
   return(node);
}

/* Find model matching context */
static HLink FindModel(HMMSetCxtInfo *hci,int lc,LabId name,int rc)
{
   LabId labid;
   MLink ml;
   char buf[80];

   /* Word internal hack */
   /* Cross word will need proper specification of context */
   /* as well as knowledge of which models are cd/ci.      */

   /* First try constructing the cd name */
   if (!allowCxtExp || (lc<=0 && rc<=0) || IsHCIContextInd(hci,name)) {
      strcpy(buf,name->name);
      labid=name;
   }
   else if ((lc==0 || forceRightBiphones || !hci->sLeft) && 
            rc>0 && !forceLeftBiphones) {
      sprintf(buf,"%s+%s",name->name,ContextName(hci,rc)->name);
      labid=GetLabId(buf,FALSE);
   }
   else if ((rc==0 || forceLeftBiphones || !hci->sRight) && 
            lc>0 && !forceRightBiphones) {
      sprintf(buf,"%s-%s",ContextName(hci,lc)->name,name->name);
      labid=GetLabId(buf,FALSE);
   }
   else if (!forceLeftBiphones && !forceRightBiphones) {
      sprintf(buf,"%s-%s+%s",ContextName(hci,lc)->name,
              name->name,ContextName(hci,rc)->name);
      labid=GetLabId(buf,FALSE);
   }
   else{
      strcpy(buf, name->name);
      labid=name;
   }
   ml=FindMacroName(hci->hset,'l',labid);
   
   /* Then try the name itself */
   if (ml==NULL && (((lc==0 && rc==0) || !forceCxtExp) || 
                    (lc==0 || !forceLeftBiphones) || 
                    (rc==0 || !forceRightBiphones))) {
      ml=FindMacroName(hci->hset,'l',name);
   }
   if (ml==NULL) 
      return(NULL);
   else 
      return((HLink) ml->structure);
}

/* Determine if phone in position pos is independent of right context */
static Boolean IsRContextInd(HMMSetCxtInfo *hci,PronHolder *p,int pos,int xlc)
{
   LabId labid;
   HLink hmm,cmp;
   int i,j,lc;
   
   for (labid=NULL,i=pos-1;i>=0;i--)
      if (GetHCIContext(hci,p->phones[i])>=0) {
         labid=p->phones[i];
         break;
      }
   if (labid!=NULL) {
      lc=FindLContext(hci,p,i,xlc);
      if (lc==-1) {
         return(IsHCIContextInd(hci,labid));
      }
      else {
         hmm=NULL;
         for (j=1;j<hci->nc;j++) {
            cmp=FindModel(hci,lc,labid,j);
            if (hmm==NULL) hmm=cmp;
            else if (cmp!=hmm) 
               return(FALSE);
         }
         return(TRUE);
      }
   }
   else {
      for (i=pos-1;i>=0;i--)
         if (!IsHCIContextInd(hci,p->phones[i])) break;
      if (i<0) return(TRUE);
   }
      
   HError(8290,"IsRContextInd: Context check not possible for %s",
          p->pron->outSym==NULL?p->pron->word->wordName->name:
          p->pron->outSym->name);
   return(FALSE);
}

/* Determine if dictionary voc be constructed solely from word internal */
/* models.  Notionally checks for cross word/word internal differences */
static Boolean InternalDict(Vocab *voc,HMMSetCxtInfo *hci)
{
   Word word;
   Pron pron;
   int i,j,h,lc,rc;

   for (h=0; h<VHASHSIZE; h++)
      for (word=voc->wtab[h]; word!=NULL; word=word->next)
         for (pron=word->pron; pron!=NULL; pron=pron->next) {
            for (i=0;i<pron->nphones;i++) {
               for (j=i-1,lc=-1;j>=0;j--)
                  if ((lc=GetHCIContext(hci,pron->phones[j]))>=0)
                     break;
               for (j=i+1,rc=-1;j<pron->nphones;j++)
                  if ((rc=GetHCIContext(hci,pron->phones[j]))>=0)
                     break;
               if (lc<0) lc=0; if (rc<0) rc=0;
               if (FindModel(hci,lc,pron->phones[i],rc)==NULL)
                  return(FALSE);
            }
         }
   return(TRUE);
}

/* Find model matching context */
HLink GetHCIModel(HMMSetCxtInfo *hci,int lc,LabId name,int rc)
{
   HLink hmm;
   
   hmm=FindModel(hci,lc,name,rc);
   if (hmm==NULL)
      HError(8231,"GetHCIModel: Cannot find hmm [%s-]%s[+%s]",
             (lc>0?ContextName(hci,lc)->name:"???"),name->name,
             (rc>0?ContextName(hci,rc)->name:"???"));
   return(hmm);
}

/* AddInitialFinal: Add links to/from initial/final net nodes */
static void AddInitialFinal(Lattice *wnet, Network *net,int xc)
{
   PronHolder *pInst;
   NetNode *node;
   LNode *thisLNode;
   int ninitial = 0;
   int i,type;

   /* Simple case */
   for (i=0; i < wnet->nn; i++)
      if (wnet->lnodes[i].pred == NULL) 
         for (pInst=(PronHolder*)wnet->lnodes[i].sublat;
              pInst!=NULL;pInst=pInst->next)
            ninitial++;
   if (ninitial==0) 
      HError(8232,"AddInitialFinal: No initial links found");
   net->initial.type = n_word;
   net->initial.tag = NULL;
   net->initial.info.pron = NULL;
   net->initial.nlinks = 0;
   net->initial.links = (NetLink *) New(net->heap,ninitial*sizeof(NetLink));
   for (i=0,thisLNode=wnet->lnodes; i<wnet->nn; i++,thisLNode++) {
      if (thisLNode->pred != NULL) continue;
      for (pInst=(PronHolder*)thisLNode->sublat;
           pInst!=NULL;pInst=pInst->next) {
         if (xc==0) node=pInst->starts;
         else if (pInst->nphones!=0) node=pInst->lc[0];
         else node=FindWordNode(NULL,pInst->pron,pInst,n_word);

         net->initial.links[net->initial.nlinks].node = node;
         net->initial.links[net->initial.nlinks++].like = 0.0;
      }
   }
   
   net->final.type = n_word;
   net->final.info.pron = NULL;
   net->final.tag = NULL;
   net->final.nlinks = 0;
   net->final.links = NULL;
   for (i=0; i < wnet->nn; i++) {
      thisLNode = wnet->lnodes+i;
      if (thisLNode->foll != NULL) continue;
      for (pInst=(PronHolder*)thisLNode->sublat;
           pInst!=NULL;pInst=pInst->next) {
         if (xc==0 || pInst->nphones==0)
            node=FindWordNode(NULL,pInst->pron,pInst,n_word);
         else {
            type = n_word + pInst->fc*n_lcontext; /* rc==0 */
            node=FindWordNode(NULL,pInst->pron,pInst,type);
         }

         if (node->nlinks>0)
            HError(8232,"AddInitialFinal: End node already connected");
         node->nlinks = 1;
         node->links = (NetLink *)New(net->heap,sizeof(NetLink));
         node->links[0].node = &net->final;
         node->links[0].like = 0;
      }
   }
}

/* Allocate and initialise a PronHolder */
static PronHolder *NewPronHolder(MemHeap *heap,HMMSetCxtInfo *hci,
                                 Pron thisPron,int np,LabId *phones)
{ 
   PronHolder *pInst;
   int n;

   pInst = (PronHolder *) New(heap,sizeof(PronHolder));
   pInst->pron = thisPron; 
   pInst->nphones=np; pInst->phones=phones;
   pInst->clen=0;
   pInst->nstart=pInst->nend=0;
   pInst->starts=pInst->ends=NULL;
   pInst->chain=NULL;
   pInst->tee=FALSE;  /* Empty words are !NULL */
   if (hci->xc>0) {
      pInst->lc = (NetNode**) New(heap,sizeof(NetNode*)*hci->xc);
      pInst->rc = (NetNode**) New(heap,sizeof(NetNode*)*hci->xc);
      for (n=0;n<hci->xc;n++) pInst->lc[n]=pInst->rc[n]=NULL;
   }
   else
      pInst->lc=pInst->rc=NULL;

   if (np==0) {
      pInst->fc=-1;
      pInst->ic=-1;
      pInst->fci=FALSE;
   }
   else if (hci->xc>0) {
      pInst->fc=FindLContext(hci,pInst,pInst->nphones,-1);
      pInst->ic=FindRContext(hci,pInst,-1,-1);
      for (n=0;n<pInst->nphones;n++)
         if (GetHCIContext(hci,pInst->phones[n])>=0)
            pInst->clen++;
      if (pInst->clen==0 || pInst->fc==-1 || pInst->ic==-1)
         HError(8230,"NewPronHolder: Every word must define some context [%s=%d/%d/%d]",
                thisPron->outSym ? thisPron->outSym->name :thisPron->word->wordName->name,
                pInst->ic, pInst->clen, pInst->fc);
      pInst->fci=IsRContextInd(hci,pInst,pInst->nphones,-1);
   }
   return(pInst);
}

typedef struct pinstinfo {
   Pron pron;
   int silId;
   int n;
   int t;
   LabId *phones;
}
PInstInfo;

static int InitPronHolders(Network *net,Lattice *lat,HMMSetCxtInfo *hci,
                           Vocab *voc,MemHeap *heap,char *frcSil)
{
   PronHolder *pInst;
   NetNode *wordNode;
   Pron thisPron;
   Word thisWord;
   LNode *thisLNode;
   PInstInfo *pii;
   LabId silPhones[MAXPHONES],addPhones[MAXPHONES],labid;
   int i,j,k,l,n,t,lc,type,nNull,npii,nSil,nAdd;
   char *ptr,*p,*nxt,name[MAXSTRLEN],st;

   /* Reset hash table prior to processing lattice */
   for (i=0; i<WNHASHSIZE; i++)
      wnHashTab[i]=NULL;

   /* Determine if we have a real !NULL word */
   net->nullWord = GetWord(voc,GetLabId("!NULL", TRUE),TRUE);
   for (thisPron=net->nullWord->pron;thisPron!=NULL;thisPron=thisPron->next)
      if (thisPron->nphones!=0) {
         net->nullWord=NULL;
         break;
      }
   if (net->nullWord!=NULL) {
      if (net->nullWord->pron==NULL)
         NewPron(voc,net->nullWord,0,NULL,net->nullWord->wordName,1.0);
   }
   if (frcSil!=NULL && strlen(frcSil)>0) {
      for(nSil=nAdd=0,ptr=frcSil;ptr!=NULL;ptr=nxt) {
         if ((nxt=ParseString(ptr,name))==NULL) break;
         if (name[0]=='+' || name[0]=='-') st=name[0],p=name+1;
         else st=0,p=name;
         if (strlen(p)==0) labid=NULL;
         else labid=GetLabId(p,TRUE);
         if (st=='+' || st==0) addPhones[++nAdd]=labid;
         if (st=='-' || st==0) silPhones[++nSil]=labid;
      }
   }
   else
      nSil=nAdd=0;

   for (i=0; i<lat->nn; i++) {
      float fct;
      LArc *la;

      thisLNode = lat->lnodes+i;
      fct = 0.0;
      if (factorLM && thisLNode->pred!=NULL)
         for (la=thisLNode->pred,fct=LZERO;la!=NULL;la=la->parc) 
            if (la->lmlike>fct) fct=la->lmlike;
      thisLNode->score = fct;
   }
   if (factorLM)
      for (i=0; i<lat->na; i++) {
         LArc *la;
         la=NumbLArc(lat,i);
         la->lmlike-=la->end->score;
      }

   /* Create instance for each pronunciation in lattice */
   for (i=0,nNull=0,t=0; i < lat->nn; i++) {
      thisLNode = lat->lnodes+i;
      thisWord = thisLNode->word;
      if (thisWord==NULL) thisWord=voc->nullWord;
      if (thisWord==voc->subLatWord)
         HError(8220,"InitPronHolders: Expand lattice before making network");
      thisLNode->sublat=NULL;
      if (thisWord->nprons<=0)
         HError(8220,"InitPronHolders: Word %s not defined in dictionary",
                thisWord->wordName->name);

      pii=(PInstInfo *) New(&gstack,(thisWord->nprons+1)*(nAdd+1)*sizeof(PInstInfo));
      pii--;
      /* Scan current pronunciations and make modified ones */
      for (j=1,thisPron=thisWord->pron,npii=0; thisPron!=NULL;
           j++,thisPron=thisPron->next) {
         if (thisPron->nphones==0) n=0;
         else
            for (k=1,n=thisPron->nphones;k<=nSil;k++) 
               if (thisPron->phones[thisPron->nphones-1]==silPhones[k]) {
                  /* Strip it */
                  n--;break;
               }
         if (thisPron->nphones==0 || nAdd==0 || n==0) {
            /* Just need one pronunciation */
            if (thisPron->nphones==0) {
               if (thisWord!=net->nullWord && (trace&T_CXT)) 
                  printf("InitPronHolders: Word %s has !NULL pronunciation\n",
                         thisWord->wordName->name);
               nNull++;
            }
            if (n==0) n=thisPron->nphones;
            pii[++npii].pron=thisPron; pii[npii].silId=-1;
            pii[npii].n=n;pii[npii].t=n;
            pii[npii].phones=thisPron->phones;
         }
         else {
            /* Make one instance per silence label */
            for (k=1;k<=nAdd;k++) {
               pii[++npii].pron=thisPron; 
               pii[npii].silId=k;
               pii[npii].n=pii[npii].t=n;
               if (addPhones[k]!=NULL) 
                  pii[npii].t++;
               pii[npii].phones=(LabId *) New(heap,sizeof(LabId)*pii[npii].t);
               for(l=0;l<pii[npii].n;l++) 
                  pii[npii].phones[l]=pii[npii].pron->phones[l];
               if (addPhones[k]!=NULL) 
                  pii[npii].phones[pii[npii].n]=addPhones[k];
            }
         }
      }
      /* Scan new pronunciations and remove duplicates */
      if (remDupPron)
         for (j=2; j<=npii; j++) {
            n=pii[j].t;
            if (pii[j].pron==NULL) continue;
            for (k=1; k<j; k++) {
               if (pii[j].pron==NULL || pii[k].pron==NULL ||
                   pii[k].t!=n || pii[j].pron->prob!=pii[k].pron->prob) 
                  continue;
               for(l=0;l<n;l++) 
                  if (pii[j].phones[l]!=pii[k].phones[l]) break;
               if (l==n) pii[j].pron=NULL,t++;
            }
         }
      /* Now make the PronHolders */
      for (j=1; j<=npii; j++) {
         /* Don't add duplicates */
         if (pii[j].pron==NULL) continue;
         /* Build inst for each pron */
         pInst=NewPronHolder(heap,hci,pii[j].pron,pii[j].t,pii[j].phones);
         pInst->ln = thisLNode;
         pInst->next = (PronHolder*)thisLNode->sublat;
         thisLNode->sublat = (SubLatDef*) pInst;
         if (pInst->nphones<=0) pInst->fct = 0.0;
         else pInst->fct = thisLNode->score/pInst->nphones;

         /* Fake connections from SENT_[START/END] */
         if (hci->xc>0) {
            if (thisLNode->pred==NULL) 
               pInst->lc[0]=(NetNode*)lat;
            if (thisLNode->foll==NULL) {
               if (pInst->nphones==0) lc=0;
               else lc = pInst->fc;
               type = n_word + lc*n_lcontext; /* rc==0 */
               wordNode=FindWordNode(net->heap,pInst->pron,pInst,type);
               wordNode->tag=SafeCopyString(net->heap,thisLNode->tag);
               wordNode->nlinks = 0;
               pInst->rc[0]=wordNode;
            }
         }
         else if (thisLNode->foll==NULL) {
            wordNode = FindWordNode(net->heap,pInst->pron,pInst,n_word);
            wordNode->tag=SafeCopyString(net->heap,thisLNode->tag);
            wordNode->nlinks = 0;
         }
      }
      Dispose(&gstack,++pii);
   }
   if (t!=0) 
      HError(-8221,"InitPronHolders: Total of %d duplicate pronunciations removed",t);
   return(nNull);
}

#define MAX_DEPTH 10 /* Max number of !NULL !NULL links before assuming loop */

void SetNullLRecurse(PronHolder *pInst,Lattice *lat,int xc)
{
   PronHolder *lInst;
   LNode *thisLNode;
   LArc *la;
   int lc;
   static int depth=0;

   if (++depth>MAX_DEPTH)
      HError(8232,"SetNullRecurse: Net probably has loop contain just !NULL");
   thisLNode=pInst->ln;
   for(la=thisLNode->pred;la!=NULL;la=la->parc)
      for (lInst=(PronHolder*)la->start->sublat;lInst!=NULL;lInst=lInst->next)
         if (lInst->nphones==0)
            SetNullLRecurse(lInst,lat,xc);
   for(la=thisLNode->pred;la!=NULL;la=la->parc)
      for (lInst=(PronHolder*)la->start->sublat;
           lInst!=NULL;lInst=lInst->next) {
         if (lInst->nphones!=0) continue;
         for(lc=0;lc<xc;lc++) {
            if (lInst->lc[lc]!=NULL && pInst->lc[lc]==NULL)
               pInst->lc[lc]=(NetNode*)lat;
         }
      }
   depth--;
}

void SetNullRRecurse(PronHolder *pInst,Lattice *lat,int xc)
{
   PronHolder *rInst;
   LNode *thisLNode;
   LArc *la;
   int rc;
   static int depth=0;

   if (++depth>MAX_DEPTH)
      HError(8232,"SetNullRecurse: Net probably has loop contain just !NULL");
   thisLNode=pInst->ln;
   for(la=thisLNode->foll;la!=NULL;la=la->farc)
      for (rInst=(PronHolder*)la->end->sublat;rInst!=NULL;rInst=rInst->next)
         if (rInst->nphones==0)
            SetNullRRecurse(rInst,lat,xc);
   for(la=thisLNode->foll;la!=NULL;la=la->farc)
      for (rInst=(PronHolder*)la->end->sublat;rInst!=NULL;rInst=rInst->next) {
         if (rInst->nphones!=0) continue;
         for(rc=0;rc<xc;rc++) {
            if (rInst->rc[rc]!=NULL && pInst->rc[rc]==NULL)
               pInst->rc[rc]=(NetNode*)lat;
         }
      }
   depth--;
}

void SetNullContexts(Lattice *lat,int xc)
{
   PronHolder *lInst,*rInst,*pInst;
   LNode *thisLNode;
   LArc *thisLArc;
   Boolean doPairs;
   int i,lc,rc;

   doPairs=FALSE;
   for (i=0; i<lat->na; i++) {
      thisLArc = NumbLArc(lat, i);
      for (lInst=(PronHolder*)thisLArc->start->sublat;
           lInst!=NULL;lInst=lInst->next)
         for (rInst=(PronHolder*)thisLArc->end->sublat;
              rInst!=NULL;rInst=rInst->next) {
            if (rInst->nphones==0 && lInst->nphones==0) {
               doPairs=TRUE;
               rInst->fci=lInst->fci=TRUE;
            }
            else if (rInst->nphones==0) {
               lc = lInst->fc;
               rInst->lc[lc]=(NetNode*)lat; /* never returned by New */
            }
            else if (lInst->nphones==0) {
               rc = rInst->ic;
               lInst->rc[rc]=(NetNode*)lat; /* never returned by New */
            }
         }
   }
   if (doPairs) {
      for (i=0; i < lat->nn; i++) {
         thisLNode = lat->lnodes+i;
         for(pInst=(PronHolder*)thisLNode->sublat;
             pInst!=NULL;pInst=pInst->next) {
            if (pInst->nphones!=0 || !pInst->fci) continue;
            SetNullLRecurse(pInst,lat,xc);
            SetNullRRecurse(pInst,lat,xc);
         }
      }
   }
}

/* Process the cross word links, the first time heap!=NULL and */
/*  the links are counted and wordnodes created, the second    */
/*  time heap==NULL and link likelihoods/destinations are set. */
void ProcessCrossWordLinks(MemHeap *heap,Lattice *lat,int xc)
{
   PronHolder *lInst,*rInst;
   NetNode *wordNode;
   LArc *thisLArc;
   int i,lc,rc,type;

   /*  Currently a new word end is created for all logical contexts */
   /*  This is only needed for single phone words for which several */
   /*  models (in different contexts) connect to a single word end. */
   /*  For multi-phone words no models are shared and so a single   */
   /*  word end per distinct physical model would be fine.          */
   for (i=0; i<lat->na; i++) {
      thisLArc = NumbLArc(lat, i);
      for (lInst=(PronHolder*)thisLArc->start->sublat;
           lInst!=NULL;lInst=lInst->next)
         for (rInst=(PronHolder*)thisLArc->end->sublat;
              rInst!=NULL;rInst=rInst->next) {
            if (xc==0) {
               wordNode = FindWordNode(heap,lInst->pron,lInst,n_word);
               if (heap!=NULL)
                  wordNode->tag=SafeCopyString(heap,thisLArc->start->tag); 
               if (heap==NULL) {
                  wordNode->links[wordNode->nlinks].node=rInst->starts;
                  wordNode->links[wordNode->nlinks].like=thisLArc->lmlike;
               }
               wordNode->nlinks++;
            }
            else if (rInst->nphones==0 && lInst->nphones==0) {
               for (lc=0;lc<xc;lc++) {
                  if (lInst->lc[lc]==NULL || rInst->lc[lc]==NULL)
                     continue;
                  for (rc=0;rc<xc;rc++) {
                     if (lInst->rc[rc]==NULL || rInst->rc[rc]==NULL)
                        continue;
                     type = n_word + lc*n_lcontext + rc*n_rcontext;
                     wordNode=FindWordNode(heap,lInst->pron,lInst,type);
                     if (heap!=NULL)
                        wordNode->tag=SafeCopyString(heap,
                                                     thisLArc->start->tag); 
                     if (heap==NULL) {
                        if (rInst->ln->foll==NULL)
                           type = n_word;
                        wordNode->links[wordNode->nlinks].node=
                           FindWordNode(heap,rInst->pron,rInst,type);
                        wordNode->links[wordNode->nlinks].like=
                           thisLArc->lmlike;
                     }
                     wordNode->nlinks++;
                  }
               }
            }
            else if (rInst->nphones==0) {
               if (thisLArc->end->foll==NULL) lc=0;
               else lc = lInst->fc;
               if (lInst->fci) {
                  type = n_word + lc*n_lcontext;
                  wordNode=FindWordNode(heap,lInst->pron,lInst,type);
                  if (heap!=NULL)
                     wordNode->tag=SafeCopyString(heap,thisLArc->start->tag);
               }
               else
                  wordNode=NULL; /* Keeps the compiler happy */
               for (rc=0;rc<xc;rc++) {
                  if (rInst->rc[rc]==NULL) continue;
                  if (!lInst->fci) {
                     type = n_word + lc*n_lcontext + rc*n_rcontext;
                     wordNode=FindWordNode(heap,lInst->pron,lInst,type);
                     if (heap!=NULL)
                        wordNode->tag=SafeCopyString(heap,
                                                     thisLArc->start->tag);
                  }
                  if (heap==NULL) {
                     type = n_word + lc*n_lcontext + rc*n_rcontext;
                     wordNode->links[wordNode->nlinks].node=
                        FindWordNode(heap,rInst->pron,rInst,type);
                     wordNode->links[wordNode->nlinks].like=thisLArc->lmlike;
                  }
                  else
                     lInst->rc[rc]=wordNode;
                  wordNode->nlinks++;
               }
            }
            else if (lInst->nphones==0) {
               if (thisLArc->start->pred==NULL) rc=0;
               else rc = rInst->ic;
               for (lc=0;lc<xc;lc++) {
                  if (lInst->lc[lc]==NULL) continue;
                  type = n_word + lc*n_lcontext + rc*n_rcontext;
                  wordNode=FindWordNode(heap,lInst->pron,lInst,type);
                  if (heap!=NULL)
                     wordNode->tag=SafeCopyString(heap,
                                                  thisLArc->start->tag);
                  if (heap==NULL) {
                     wordNode->links[wordNode->nlinks].node=rInst->lc[lc];
                     wordNode->links[wordNode->nlinks].like=thisLArc->lmlike;
                  }
                  else
                     rInst->lc[lc]=(NetNode*)lat;
                  wordNode->nlinks++;
               }
            }
            else {
               lc = lInst->fc;
               rc = rInst->ic;
               if (lInst->fci) type = n_word + lc*n_lcontext;
               else type = n_word + lc*n_lcontext + rc*n_rcontext;
               wordNode = FindWordNode(heap,lInst->pron,lInst,type);
               if (heap!=NULL)
                  wordNode->tag=SafeCopyString(heap,
                                               thisLArc->start->tag);
               if (heap==NULL) {
                  wordNode->links[wordNode->nlinks].node=rInst->lc[lc];
                  wordNode->links[wordNode->nlinks].like=thisLArc->lmlike;
               }
               else {
                  lInst->rc[rc]=wordNode;
                  rInst->lc[lc]=(NetNode*)lat;
               }
               wordNode->nlinks++;
            }
         }
   }
}


void CreateWIModels(PronHolder *pInst,int p,int q,
                    Network *net,HMMSetCxtInfo *hci)
{
   NetNode *node;
   HLink hmm;
   int j;
   
   for(j=q-1;j>p;j--) {
      hmm=GetHCIModel(hci,FindLContext(hci,pInst,j,0),
                      pInst->phones[j],
                      FindRContext(hci,pInst,j,0));
      if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;
      
      nwi++;
      node=NewNode(net->heap,hmm,(pInst->chain==NULL?0:1));
      if (pInst->chain!=NULL) {
         nil++;
         node->links[0].node=pInst->chain;
         node->links[0].like=pInst->fct;
      }
      node->chain=pInst->chain;
      pInst->chain=node;
   }
}

void CreateIEModels(Word thisWord,PronHolder *pInst,int p,int q,
                    Network *net,HMMSetCxtInfo *hci)
{
   NetNode *node,*wordNode;
   HLink hmm;

   if (q==p) {
      /* One phone word */
      hmm=GetHCIModel(hci,0,pInst->phones[0],0);
      if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;

      wordNode = FindWordNode(NULL,pInst->pron,pInst,n_word);
      
      nin++; nil++;
      node=NewNode(net->heap,hmm,1);
      node->links[0].node=wordNode;
      node->links[0].like=pInst->fct;
      
      pInst->starts=node;
      pInst->nstart=1;
   }
   else {
      /* End */
      hmm=GetHCIModel(hci,FindLContext(hci,pInst,q,0),
                      pInst->phones[q],0);
      if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;

      wordNode = FindWordNode(NULL,pInst->pron,pInst,n_word);
      
      nfi++; nil++;
      node=NewNode(net->heap,hmm,1);
      node->links[0].node=wordNode;
      node->links[0].like=pInst->fct;
      
      pInst->ends=node;
      pInst->nend=1;
      
      /* Start */
      hmm=GetHCIModel(hci,0,pInst->phones[p],
                      FindRContext(hci,pInst,p,0));
      if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;
      
      nin++; nil++;
      node=NewNode(net->heap,hmm,1);
      node->links[0].node=(pInst->chain?pInst->chain:pInst->ends);
      node->links[0].like=pInst->fct;
      pInst->starts=node;
      pInst->nstart=1;
      
      /* Chain */
      if (pInst->chain!=NULL) {
         for (node=pInst->chain;node->chain!=NULL;
              node=node->chain);
         node->nlinks=1;
         nil++;
         node->links=(NetLink*) New(net->heap,
                                    sizeof(NetLink));
         node->links[0].node=pInst->ends;
         node->links[0].like=pInst->fct;
      }
   }
}

static void CreateX1Model(PronHolder *pInst,int p, int q,
                          Network *net,HMMSetCxtInfo *hci,MemHeap *heap)
{
   NetNode *node,*dest,*wordNode,*linkNode;
   NetLink *links;
   HLink hmm;
   Ptr tptr;
   Boolean tee,initTee,anyTee;
   int j,k,n;

   /* Single phone word means that we need to */
   /*  build a complete cross-bar of contexts */
   
   tee=FALSE; /* Assume okay */

   /* Special case because one phone words so expensive */
   if (IsHCIContextInd(hci,pInst->phones[p])) {
      hmm=GetHCIModel(hci,0,pInst->phones[p],0);
      if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;
      nin++; nil++;
      node=NewNode(net->heap,hmm,0);
      
      pInst->starts=node;
      
      /* As well as copies of final context free ones */
      for (n=q+1;n<pInst->nphones;n++) {
         hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
         if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;
         ncf++;
         dest=NewNode(net->heap,hmm,0);
         dest->chain=pInst->chain;pInst->chain=dest;
         
         nil++;
         node->links=(NetLink*) New(net->heap,
                                    sizeof(NetLink));
         node->nlinks=1;
         node->links[0].node=dest;
         node->links[0].like=pInst->fct;
         
         node=dest;
      }
      links=(NetLink*) New(heap,sizeof(NetLink)*hci->xc);
      for(j=0,wordNode=NULL;j<hci->xc;j++)
         if (pInst->rc[j]!=NULL) {
            wordNode=pInst->rc[j];
            for (k=0;k<node->nlinks;k++)
               if (links[k].node==wordNode) {
                  wordNode=NULL;
                  break;
               }
            if (wordNode!=NULL) {
               links[node->nlinks].node=wordNode;
               links[node->nlinks++].like=pInst->fct;
            }
         }
      
      node->links=(NetLink*) New(net->heap,sizeof(NetLink)*node->nlinks);
      for (k=0;k<node->nlinks;k++)
         node->links[k]=links[k];
      Dispose(heap,links);
      
      /* Create any previous context free nodes */
      node = pInst->starts;
      pInst->starts=NULL;
      
      for (n=p-1;n>=0;n--) {
         dest=node;
         dest->chain=pInst->chain;
         pInst->chain=dest;
         ncf++;
         hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
         if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;
         node=NewNode(net->heap,hmm,1);
         nil++;
         node->links[0].node=dest;
         node->links[0].like=pInst->fct;
      }
      pInst->nstart=1;
      pInst->starts=node;
      
      for (j=0;j<hci->xc;j++)
         if (pInst->lc[j]!=NULL)
            pInst->lc[j]=node;

   }
   else if (!hci->sLeft) {
      if (p==0) {
         /* Create NULL node */
         /*  Used as single collating point for all r contexts */
         node=(NetNode *)New(net->heap,sizeof(NetNode));
         node->nlinks=0;
         node->links=NULL;
         node->inst=NULL;
         node->type=n_word;
         node->info.pron=NULL;
         node->tag=NULL;
         node->aux=0;

         node->chain=pInst->starts;
         pInst->starts=node;
         pInst->nstart++;
         nll++;
      }
      else {
         ncf++;
         hmm=GetHCIModel(hci,-1,pInst->phones[0],-1);
         if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;
         node=NewNode(net->heap,hmm,0);
         
         /* Chain these after NULL node */
         node->chain=pInst->starts;
         pInst->starts=node;
         pInst->nstart++;
         
         /* Create any previous context free nodes */
         for (n=1;n<p;n++) {
            ncf++;
            hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
            if (hmm->transP[1][hmm->numStates]<LSMALL) pInst->tee=FALSE;
            dest=NewNode(net->heap,hmm,0);
            node->nlinks=1;
            nil++;
            node->links=(NetLink*) New(net->heap,
                                       sizeof(NetLink));
            node->links[0].node=dest;
            node->links[0].like=0.0;
            
            /* Chain these after NULL node */
            dest->chain=pInst->chain;
            pInst->chain=dest;
            
            node=dest;
         }
      }
      linkNode=node;

      for(j=0;j<hci->xc;j++)
         if (pInst->lc[j]!=NULL)
            pInst->lc[j]=pInst->starts;
      
      /* Now create actual cd phone */
      tptr=New(heap,1); /* Need place holder to free */
      anyTee=FALSE; /* Haven't seen any final tee chains */
      for(k=0;k<hci->xc;k++) {
         if (pInst->rc[k]==NULL) continue;
         
         hmm=GetHCIModel(hci,0,pInst->phones[q],k);
         for(node=pInst->ends;node!=NULL;node=node->chain)
            if (node->info.hmm==hmm) break;
         
         if (node==NULL) {
            if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Okay now */
            else tee=TRUE; /* Still could be save by final CF models */
            /* Create new model */
            nin++;
            node=NewNode(net->heap,hmm,0);
            node->chain=pInst->ends;pInst->ends=node;
            pInst->nend++;
            linkNode->nlinks++;
            
            /* As well as copies of final context free ones */
            for (n=q+1;n<pInst->nphones;n++) {
               hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
               if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Saved */
               ncf++;
               dest=NewNode(net->heap,hmm,0);
               dest->chain=pInst->chain;pInst->chain=dest;
               
               nil++;
               node->links=(NetLink*) New(net->heap,
                                          sizeof(NetLink));
               node->nlinks=1;
               node->links[0].node=dest;
               node->links[0].like=0.0;
               
               node=dest;
            }
            if (tee) anyTee=TRUE; /* A single tee chain is too many */
            
            /* Use inst pointer to get to final model in links */
            pInst->ends->inst=(NetInst*)node;
            
            node->links=(NetLink*) New(heap,sizeof(NetLink)*hci->xc);
         }
         else
            /* Find final model in links */
            node = (NetNode*)node->inst;
         
         node->links[node->nlinks].node=pInst->rc[k];
         node->links[node->nlinks].like=0.0;
         node->nlinks++;
      }
      if (!anyTee) pInst->tee=FALSE; /* Didn't see any tee chains */
      
      /* Now allocate and copy links */
      for (node=pInst->ends;node!=NULL;node=node->chain) {
         dest=(NetNode*)node->inst; /* Find end of cf models */
         nil+=dest->nlinks;
         links=(NetLink*) New(net->heap,
                              sizeof(NetLink)*dest->nlinks);
         for (n=0;n<dest->nlinks;n++)
            links[n]=dest->links[n];
         dest->links=links;
      }
      Dispose(heap,tptr);
      
      /* And finally link null node to models */
      nil+=linkNode->nlinks;
      linkNode->links=(NetLink*)New(net->heap,
                                    sizeof(NetLink)*linkNode->nlinks);
      for (dest=pInst->ends,n=0,node=NULL;dest!=NULL;dest=dest->chain) {
         node=dest;
         linkNode->links[n].node=dest;
         linkNode->links[n].like=0.0;
         n++;
      }
      /* Move these models over to chain */
      node->chain=pInst->chain;
      pInst->chain=pInst->ends;
      pInst->ends=NULL;
      pInst->nend=0;
   }
   else {
      /* Otherwise we do it properly */
      anyTee=FALSE; /* Haven't seen any tee chains */

      for(j=0;j<hci->xc;j++) {
         if (pInst->lc[j]==NULL) continue;
         
         initTee=TRUE; /* Start off assuming the worse */

         if (p==0) {
            /* Create NULL node */
            /*  Used as single collating point for all r contexts */
            node=(NetNode *)New(net->heap,sizeof(NetNode));
            node->nlinks=0;
            node->links=NULL;
            node->inst=NULL;
            node->type=n_word;
            node->info.pron=NULL;
            node->tag=NULL;
            node->aux=0;

            node->chain=pInst->starts;
            pInst->starts=node;
            pInst->nstart++;
            nll++;
            
            pInst->lc[j]=node;
         }
         else {
            ncf++;
            hmm=GetHCIModel(hci,-1,pInst->phones[0],-1);
            node=NewNode(net->heap,hmm,0);
            pInst->lc[j]=node;
            
            /* Chain these after NULL node */
            node->chain=pInst->starts;
            pInst->starts=node;
            pInst->nstart++;
            
            /* Create any previous context free nodes */
            for (n=1;n<p;n++) {
               ncf++;
               hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
               if (hmm->transP[1][hmm->numStates]<LSMALL) initTee=FALSE; /* Okay now */
               dest=NewNode(net->heap,hmm,0);
               node->nlinks=1;
               nil++;
               node->links=(NetLink*) New(net->heap,
                                          sizeof(NetLink));
               node->links[0].node=dest;
               node->links[0].like=pInst->fct;
               
               /* Chain these after NULL node */
               dest->chain=pInst->chain;
               pInst->chain=dest;
               
               node=dest;
            }
         }
         linkNode=node;
         
         /* Now create actual cd phone */
         tptr=New(heap,1); /* Need place holder to free */

         for(k=0;k<hci->xc;k++) {
            if (pInst->rc[k]==NULL) continue;
            
            hmm=GetHCIModel(hci,j,pInst->phones[q],k);
            for(node=pInst->ends;node!=NULL;node=node->chain)
               if (node->info.hmm==hmm) break;
            
            if (node==NULL) {
               if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Okay */
               else tee=initTee; /* Start at end of initial chain */

               /* Create new model */
               nin++;
               node=NewNode(net->heap,hmm,0);
               node->chain=pInst->ends;pInst->ends=node;
               pInst->nend++;
               linkNode->nlinks++;
                        
               /* As well as copies of final context free ones */
               for (n=q+1;n<pInst->nphones;n++) {
                  hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
                  if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Saved */
                  ncf++;
                  dest=NewNode(net->heap,hmm,0);
                  dest->chain=pInst->chain;pInst->chain=dest;
                  
                  nil++;
                  node->links=(NetLink*) New(net->heap,
                                             sizeof(NetLink));
                  node->nlinks=1;
                  node->links[0].node=dest;
                  node->links[0].like=pInst->fct;
                  
                  node=dest;
               }
               if (tee) anyTee=TRUE; /* A single tee chain is too many */
               
               /* Use inst pointer to get to final model in links */
               pInst->ends->inst=(NetInst*)node;
               
               node->links=(NetLink*) New(heap,sizeof(NetLink)*hci->xc);
            }
            else
               /* Find final model in links */
               node = (NetNode*)node->inst;
            
            node->links[node->nlinks].node=pInst->rc[k];
            node->links[node->nlinks].like=pInst->fct;
            node->nlinks++;
         }
         /* Now allocate and copy links */
         for (node=pInst->ends;node!=NULL;node=node->chain) {
            dest=(NetNode*)node->inst; /* Find end of cf models */
            nil+=dest->nlinks;
            links=(NetLink*) New(net->heap,
                                 sizeof(NetLink)*dest->nlinks);
            for (n=0;n<dest->nlinks;n++)
               links[n]=dest->links[n];
            dest->links=links;
         }
         Dispose(heap,tptr);
         
         /* And finally link null node to models */
         nil+=linkNode->nlinks;
         linkNode->links=(NetLink*)New(net->heap,
                                       sizeof(NetLink)*linkNode->nlinks);
         for (dest=pInst->ends,n=0,node=NULL;dest!=NULL;dest=dest->chain) {
            node=dest;
            linkNode->links[n].node=dest;
            linkNode->links[n].like=(p==0?0.0:pInst->fct);
            n++;
         }
         /* Move these models over to chain */
         node->chain=pInst->chain;
         pInst->chain=pInst->ends;
         pInst->ends=NULL;
         pInst->nend=0;
      }
      if (!anyTee) pInst->tee=FALSE; /* Didn't see any completely tee chains */
   }
}

static void CreateXEModels(PronHolder *pInst,int p, int q,
                           Network *net,HMMSetCxtInfo *hci,MemHeap *heap)
{
   NetNode *node,*dest,*chainNode,*searchNode;
   NetLink *links;
   HLink hmm;
   Ptr tptr;
   Boolean tee,anyTee;
   int j,n;

   /* Cross word context and more than one phone */
   
   /* Last cd phone */
   chainNode=NULL;searchNode=NULL;
   tptr=New(heap,1);
   anyTee=FALSE; /* Haven't seen any final tee chains */
   for(j=0;j<hci->xc;j++) {
      if (pInst->rc[j]==NULL) continue;
      hmm=GetHCIModel(hci,FindLContext(hci,pInst,q,-1),
                      pInst->phones[q],j);
      for(node=pInst->ends;node!=NULL;node=node->chain)
         if (node->info.hmm==hmm) break;
      if (node==NULL) {
         if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Okay now */
         else tee=TRUE; /* Still could be save by final CF models */
         nfi++;
         node=NewNode(net->heap,hmm,0);
         node->chain=pInst->ends;
         pInst->ends=node;
         pInst->nend++;
         
         for (n=q+1;n<pInst->nphones;n++) {
            hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
            if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Saved */
            ncf++;
            dest=NewNode(net->heap,hmm,0);
            dest->chain=chainNode;chainNode=dest;
            
            nil++;
            node->links=(NetLink*) New(net->heap,sizeof(NetLink));
            node->nlinks=1;
            node->links[0].node=dest;
            node->links[0].like=pInst->fct;
            
            node=dest;
         }
         
         /* Use inst pointer to get to final model in links */
         pInst->ends->inst=(NetInst*)node;
         
         node->links=(NetLink*) New(heap,sizeof(NetLink)*hci->xc);
         if (tee) anyTee=TRUE; /* A single tee chain is too many */
      }
      else
         /* Find end of cf models */
         node = (NetNode*)node->inst;
      
      node->links[node->nlinks].node=pInst->rc[j];
      node->links[node->nlinks].like=pInst->fct;
      node->nlinks++;
      if (pInst->fci) break; /* Only need to do this once */
   }
   if (!anyTee) pInst->tee=FALSE; /* Didn't see any tee chains */

   /* Now allocate and copy links */
   for (node=pInst->ends;node!=NULL;node=node->chain) {
      dest=(NetNode*)node->inst; /* Find end of cf models */
      nil+=dest->nlinks;
      links=(NetLink*) New(net->heap,
                           sizeof(NetLink)*dest->nlinks);
      for (j=0;j<dest->nlinks;j++)
         links[j]=dest->links[j];
      dest->links=links;
   }
   Dispose(heap,tptr);
   
   /* And finally link to ci part of word */
   if (pInst->chain!=NULL) {
      for (node=pInst->chain;node->chain!=NULL;node=node->chain);
      node->nlinks=pInst->nend;
      nil+=node->nlinks;
      node->links=(NetLink*) New(net->heap,
                                 sizeof(NetLink)*node->nlinks);
      for (dest=pInst->ends,n=0;dest!=NULL;dest=dest->chain) {
         node->links[n].node=dest;
         node->links[n].like=pInst->fct;
         n++;
      }
   }
   anyTee=FALSE; /* Haven't seen any initial tee chains */
   /* Create first cd phone */
   for(j=0;j<hci->xc;j++) {
      if (pInst->lc[j]==NULL) continue;
      hmm=GetHCIModel(hci,j,pInst->phones[p],
                      FindRContext(hci,pInst,p,-1));
      for(node=pInst->starts;node!=NULL;node=node->chain)
         if (node->info.hmm==hmm) break;
      if (node==NULL) {
         if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Okay now */
         else tee=TRUE; /* Still could be save by initial CF models */
         nin++;
         node=NewNode(net->heap,hmm,
                      (pInst->chain==NULL?pInst->nend:1));
         nil+=node->nlinks;
         
         node->chain=pInst->starts;pInst->starts=node;
         pInst->nstart++;
         
         if (pInst->chain!=NULL) {
            node->links[0].node=pInst->chain;
            node->links[0].like=pInst->fct;
         }
         else {
            /* Two phone words need crossbar linking */
            for (dest=pInst->ends,n=0;dest!=NULL;
                 dest=dest->chain,n++) {
               node->links[n].node=dest;
               node->links[n].like=pInst->fct;
            }
         }
         
         for (n=p-1;n>=0;n--) {
            dest=node;
            ncf++;
            hmm=GetHCIModel(hci,-1,pInst->phones[n],-1);
            if (hmm->transP[1][hmm->numStates]<LSMALL) tee=FALSE; /* Saved */
            node=NewNode(net->heap,hmm,1);
            if (n!=0) {
               node->chain=chainNode;
               chainNode=node;
            }
            nil++;
            node->links[0].node=dest;
            node->links[0].like=pInst->fct;
         }
         if (tee) anyTee=TRUE; /* A single tee chain is too many */

         /* Link to start of cf models */
         pInst->starts->inst=(NetInst*)node;
      }
      else
         node=(NetNode*)node->inst; /* Find start of cf models */
      
      /* Point to start of cf models */
      pInst->lc[j]=node;
   }
   if (!anyTee) pInst->tee=FALSE; /* Didn't see any tee chains */

   if (p!=0) {
      /* Need to fix starts list */
      for (node=pInst->starts,pInst->starts=NULL;node!=NULL;node=dest) {
         dest=(NetNode*)node->inst;
         dest->chain=pInst->starts;
         pInst->starts=dest;
         dest=node->chain;
         node->chain=chainNode;
         chainNode=node;
      }
   }
   if (pInst->chain==NULL)
      pInst->chain=chainNode;
   else {
      for (node=pInst->chain;node->chain!=NULL;
           node=node->chain);
      node->chain=chainNode;
   }
}

void ShowWords(Lattice *lat,Vocab *voc,HMMSetCxtInfo *hci)
{
   NetNode *node,*dest;
   Word thisWord;
   PronHolder *pInst;
   LNode *thisLNode;
   MLink ml;
   int i,j,k;

   for (i=0; i < lat->nn; i++) {
      thisLNode = lat->lnodes+i;
      thisWord = thisLNode->word;
      if (thisWord==NULL) thisWord=voc->nullWord;
      for(pInst=(PronHolder*)thisLNode->sublat;
          pInst!=NULL;pInst=pInst->next) {
         if (pInst->pron->outSym==NULL)
            printf("[%-18s]\n",pInst->pron->word->wordName->name);
         else printf("%-20s\n",pInst->pron->outSym->name);
         if (hci->xc==0) {
            for (node=pInst->starts;(node->type&n_nocontext)!=n_word;
                 node=node->links[0].node) {
               ml=FindMacroStruct(hci->hset,'h',node->info.hmm);
               if (ml==NULL)
                  printf(" null");
               else if (node->type&n_tr0)
                  printf(" (%s)",ml->id->name);
               else
                  printf(" %s",ml->id->name);
            }
            printf(" ==> %d\n",node->nlinks);
         }
         else if (pInst->clen==0) {
            printf("!NULL word contexts [L, R]\n");
            for (j=0;j<hci->xc;j++)
               if (pInst->lc[j]!=NULL) 
                  printf(" %s",ContextName(hci,j)->name);
            printf(", ");
            for (j=0;j<hci->xc;j++)
               if (pInst->rc[j]!=NULL) 
                  printf(" %s",ContextName(hci,j)->name);
            printf("\n");
         }
         else if (pInst->clen<2) {
            printf("One phone word\n");
            for (j=0;j<hci->xc;j++) {
               if (pInst->lc[j]==NULL) continue;
               if ((pInst->lc[j]->type&n_nocontext)==n_word) {
                  if (hci->sLeft)
                     printf("  %s -> \n",ContextName(hci,j)->name);
                  for (k=0;k<pInst->lc[j]->nlinks;k++) {
                     dest=pInst->lc[j]->links[k].node;
                     ml=FindMacroStruct(hci->hset,'h',dest->info.hmm);
                     printf("      %s",ml->id->name);
                     if ((dest->links[0].node->type&n_nocontext)!=n_word)
                        printf(" ... => %d\n",dest->links[0].node->nlinks);
                     else
                        printf(" => %d\n",dest->nlinks);
                  }
               }
               else {
                  node=pInst->lc[j];
                  ml=FindMacroStruct(hci->hset,'h',node->info.hmm);
                  if (node->type&n_tr0)
                     printf(" (%s)\n",ml->id->name);
                  else
                     printf(" %s\n",ml->id->name);
                  break;
               }
               if (!hci->sLeft) break;
            }
         }
         else {
            printf(" Word initial models \n");
            for (j=0;j<hci->xc;j++) {
               if (pInst->lc[j]==NULL) continue;
               ml=FindMacroStruct(hci->hset,'h',pInst->lc[j]->info.hmm);
               printf("  %s -> %s [%d]\n",
                      ContextName(hci,j)->name,ml->id->name,
                      pInst->lc[j]->nlinks);
            }
            if (pInst->chain!=NULL)
               printf(" Word internal models \n ");
            for (node=pInst->chain;node!=NULL;node=node->chain) {
               ml=FindMacroStruct(hci->hset,'h',node->info.hmm);
               if (ml==NULL)
                  printf(" null");
               else if (node->type&n_tr0)
                  printf(" (%s)",ml->id->name);
               else
                  printf(" %s",ml->id->name);
            }
            printf("\nWord final models\n");
            for (node=pInst->ends;node!=NULL;node=node->chain) {
               ml=FindMacroStruct(hci->hset,'h',node->info.hmm);
               printf("  %s",ml->id->name);
               if ((node->links[0].node->type&n_nocontext)!=n_word)
                  printf(" ... => %d\n",node->links[0].node->nlinks);
               else
                  printf(" => %d\n",node->nlinks);
            }
         }
      }
      fflush(stdout);
   }
}


HMMSetCxtInfo *GetHMMSetCxtInfo(HMMSet *hset, Boolean frcCxtInd)
{
   HMMSetCxtInfo *hci;
   LabId labid;
   MLink ml;

   if (frcCxtInd)
      labid=GetLabId("@HCI-CI@",TRUE);
   else
      labid=GetLabId("@HCI-CD@",TRUE);
   ml=FindMacroName(hset,'@',labid);
   if (ml==NULL) {
      hci=NewHMMSetCxtInfo(hset,frcCxtInd);
      if (!frcCxtInd) DefineContexts(hci);
      NewMacro(hset,0,'@',labid,hci);
   }
   else
      hci=(HMMSetCxtInfo *) ml->structure;

   return(hci);
}

Network *ExpandWordNet(MemHeap *heap,Lattice *lat,Vocab *voc,HMMSet *hset)
{
   HMMSetCxtInfo *hci;
   Network *net;
   NetNode *node,*wordNode,*chainNode;
   NetLink netlink;
   Word thisWord;
   PronHolder *pInst;
   LNode *thisLNode;
   MemHeap holderHeap;
   int i,j,p,q,nc,nNull;

   /* Initialise */
   CreateHeap(&holderHeap,"Holder Heap",MSTAK,1,0.0,80000,80000);

   net=(Network*) New(heap,sizeof(Network));
   net->heap=heap;
   net->vocab=voc;
   net->numNode=net->numLink=0;
   net->chain=NULL;

   if (!(allowXWrdExp || allowCxtExp) ||
       (forceCxtExp==FALSE && forceRightBiphones==FALSE &&
        forceLeftBiphones==FALSE && ClosedDict(voc,hset))) {
      hci=GetHMMSetCxtInfo(hset,TRUE);
      if (forceCxtExp && hci->nc==0) 
         HError(8230,"ExpandWordNet: No contexts defined and FORCECXTEXT is true");

      nc=0;
   }
   else {
      hci=GetHMMSetCxtInfo(hset,FALSE);
      nc=hci->nc;
   }
   if (allowXWrdExp && nc>0 && (forceRightBiphones || forceLeftBiphones ||
                                forceCxtExp || !InternalDict(voc,hci)))
      hci->xc=nc+1;
   else
      hci->xc=0;

   if (trace&T_CXT) {
      if (hci->xc==0)
         printf("Performing network expansion%s\n",
                (nc>0?" with word internal contexts":""));
      else {
         printf("Performing network expansion with cross word contexts\n");
      }
      if (nc>0) {
         printf(" Defined %d contexts\n",nc);
         for (i=1;i<=nc;i++)
            printf("  %s",ContextName(hci,i)->name);
         printf("\n");
      }
   }
   

   /* First create context arrays and pronunciation instances */
   nNull=InitPronHolders(net,lat,hci,voc,&holderHeap,frcSil);
   

   /* Need to find out the phonetic contexts for all NULL words */
   if (hci->xc>0 && nNull>0)
      SetNullContexts(lat,hci->xc);

   /* Count xwrd links and create word ends */
   ProcessCrossWordLinks(net->heap,lat,hci->xc);
   
   /* Build models on basis of contexts seen */
   net->teeWords=FALSE;
   for (i=0; i < lat->nn; i++) {
      thisLNode = lat->lnodes+i;
      thisWord = thisLNode->word;
      if (thisWord==NULL) thisWord=voc->nullWord;

      if (trace&T_CST) {
         printf("Building word %s\n",thisWord->wordName->name);
      }

      for(pInst=(PronHolder*)thisLNode->sublat;
          pInst!=NULL;pInst=pInst->next) {
         /* !NULL consists only of word ends */
         if (pInst->nphones==0) {
            /* Flawed */
            if (hci->xc==0) {
               /* But we need a pointer for xc==0 cases */
               wordNode = FindWordNode(NULL,pInst->pron,pInst,n_word);
               pInst->starts = wordNode;
               pInst->nstart = 0; /* Stops us adding node into chain twice */
            }
            continue;
         }

         /* Determine which bits of word are l and r cd */
         if (hci->xc>0) {
            for (p=0;p<pInst->nphones;p++)
               if (GetHCIContext(hci,pInst->phones[p])>=0) break;
            for (q=pInst->nphones-1;q>=0;q--)
               if (GetHCIContext(hci,pInst->phones[q])>=0) break;
         }
         else {
            p=0;
            q=pInst->nphones-1;
         }
         
         pInst->tee=TRUE;
         /* Make wrd-int cd phones (possibly none!) */
         CreateWIModels(pInst,p,q,net,hci);
         if (hci->xc==0) {
            /* Word internal context only */
            CreateIEModels(thisWord,pInst,p,q,net,hci);
         }
         /* Cross word context */
         else if (pInst->clen==1) {
            /* Single phone word means that we need to */
            /*  build a complete cross-bar of contexts */
            CreateX1Model(pInst,p,q,net,hci,&holderHeap);
         }
         else {
            /* Cross word context and more than one phone */
            CreateXEModels(pInst,p,q,net,hci,&holderHeap);
         }
         if (pInst->tee) {
            HError(-8232,"ExpandWordNet: Pronunciation %d of %s is 'tee' word",
                   pInst->pron->pnum,pInst->pron->word->wordName->name);
            net->teeWords=TRUE;
         }
      }
   }
   

   /* Allocate NetLinks from hash table stats. Zero counters */
   for (i=0; i<WNHASHSIZE; i++) {
      /* Build links for each word end model */
      for (node=wnHashTab[i];node!=NULL;node=node->chain) {
         if (node->nlinks>0){
            node->links=(NetLink*) New(net->heap,
                                       sizeof(NetLink)*node->nlinks);
         }else
            node->links=NULL;
         nxl+=node->nlinks;
         node->nlinks=0;
         node->aux=0;
      }
   }

   /* Finally put in the cross word links */
   ProcessCrossWordLinks(NULL,lat,hci->xc);

   if (trace & T_MOD)
      ShowWords(lat,voc,hci);

   /* First disassemble wnHashTab and link to end nodes as necessary */
   AddInitialFinal(lat,net,hci->xc); 

   for (i=0; i<WNHASHSIZE; i++) {
      AddChain(net,wnHashTab[i]);
   }

   /* Finally chain all nodes together */
   for (i=0; i < lat->nn; i++) 
      for (pInst=(PronHolder*)lat->lnodes[i].sublat;
           pInst!=NULL;pInst=pInst->next) {
         if (pInst->nstart>0)
            AddChain(net,pInst->starts);
         AddChain(net,pInst->chain);
         AddChain(net,pInst->ends);
      }

   /* And then clear up after ourselves */
   for (i=0; i < lat->nn; i++) 
      lat->lnodes[i].sublat = NULL;
   DeleteHeap(&holderHeap);


   /* Count the initial/final nodes/links */
   net->numLink=net->initial.nlinks;
   net->numNode=2;
   /* now reorder links and identify wd0 nodes */
   for (chainNode = net->chain, ncn=0; chainNode != NULL; 
        chainNode = chainNode->chain,net->numNode++,ncn++) {
      chainNode->inst=NULL;
      chainNode->type=chainNode->type&n_nocontext;
      net->numLink+=chainNode->nlinks;
      /* Make !NULL words really NULL */
      if (chainNode->type==n_word && chainNode->info.pron!=NULL &&
          net->nullWord!=NULL && chainNode->info.pron->word==net->nullWord)
         chainNode->info.pron=NULL;
      /* First make n_wd0 nodes */
      if (chainNode->type & n_hmm)
         for (i = 0; i < chainNode->nlinks; i++)
            if ( IsWd0Link(&chainNode->links[i]) ) {
               chainNode->type |= n_wd0;
               break;
            }
      /* Then put all n_tr0 nodes first */
      for (i = 0; i < chainNode->nlinks; i++) {
         /* Don't need to move any initial n_tr0 links */
         if (chainNode->links[i].node->type & n_tr0) continue;
         /* Find if there are any n_tr0 ones to swap with */
         for (j = i+1; j < chainNode->nlinks; j++)
            if (chainNode->links[j].node->type & n_tr0) break;
         /* No, finished */
         if (j >= chainNode->nlinks) break;
         /* Yes, swap then carry on */
         netlink = chainNode->links[i];
         chainNode->links[i] = chainNode->links[j];
         chainNode->links[j] = netlink;
      }
   }
   if (trace&T_CST) {
      printf("%d nodes\n",net->numNode);
      printf(" = %d word (%d null), init %d, int %d, fin %d, cf %d\n",
             nwe,nll,nin,nwi,nfi,ncf);
      printf("%d links = int %d, ext %d (%d for null words)\n",
             net->numLink,nil,nxl,nnl);
      fflush(stdout);
   }
   if (trace&T_ALL)
      PrintChain(net,hset);

   return(net);
}   

/* ------------------------ End of HNet.c ------------------------- */
