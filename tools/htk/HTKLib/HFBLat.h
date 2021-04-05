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
/*              2002  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HFBLat.h   Lattice Forward Backward routines  */
/* ----------------------------------------------------------- */

/* !HVER!HNET:   3.4.1 [CUED 12/03/09] */


/*
   This module provides facilities to apply a HMM set to 
   forward-backward align a sequence of training files and
   collect statistics from each alignment.  The exact
   statistics to capture are defined by a set of flags and
   they are stored in the accumulators attached to the HMM
   components.    For MLLR adaptation, the HAdapt routine
   AddAdaptFrame is called for each input frame within
   each mixture component.
*/   

typedef struct {
  /* protected [readonly] : */
  int T;
  LogDouble pr;
  int MPEFileLength;
  float AvgCorr; /* averate correctness. */

  /* Private: */
  ArcInfo lattices; 
  Lattice *numLat; /* for MPE. */
  int Q;
  HMMSet *hset;

  Boolean MPE;  /* currently doing MPE/MWE, this may change from call to call
		   (since we will sometimes want to do the correct transcription) */
  float AccScale; /*normally 1.0*/
  int S;
  HSetKind hsKind;              /* kind of the alignment HMM system */
  UPDSet uFlags;
  Boolean firstTime ;     /* Flag used to enable creation of ot */


  ArcInfo  *aInfo;

  Boolean twoDataFiles;
  Observation al_ot;
  Observation up_ot;
  Observation adapt_ot;   /* for parent XForm */

  BufferInfo al_info;             /* info from buffer on second data file */
  BufferInfo up_info;             /* info from buffer on second data file */
  ParmBuf al_pbuf;
  ParmBuf up_pbuf;

  MemHeap arcStack;
  MemHeap tempStack;
  MemHeap miscStack;
  
  MemHeap al_dataStack;
  MemHeap up_dataStack;

  float num_index; /*make sure set. */
  float den_index; /*only for MPE. */ /*make sure set. */

  Boolean InUse; /* FALSE if stacks are cleared and lattices empty. */
  AdaptXForm *inXForm;/* current input transform (if any) */
  AdaptXForm *paXForm;/* current parent transform (if any) */
  /* ... */
} FBLatInfo;

void InitFBLat(void);
/*
   Initialise module and set configuration parameters. 
*/

void InitialiseFBInfo(FBLatInfo *fbInfo, HMMSet *hset, UPDSet uset, Boolean TwoDataFiles); 



void FBLatAddLattice (FBLatInfo *fbInfo, Lattice *lat); /* add this lattice, can do this repeatedly. */



void FBLatSetAccScale(FBLatInfo *fbInfo, float AccScale); /*prepare to scale accumulators by this amount. */



void FBLatFirstPass(FBLatInfo *fbInfo, 
		    FileFormat dff, char *datafn, char *datafn2 /*for single-pass retraining*/,
		    Lattice *MPECorrLat /* Only used in MPE, equals correct lattice.*/ );

void FBLatSecondPass(FBLatInfo *fbInfo, 
                     int index, /* in MMI case, this is the index to store the accs. */
                     int den_index /* den_index is used only for MPE, for negative accs.*/ );

#define SUPPORT_QUINPHONE 


/*For use in HExactLat.c: */
int GetNoContextPhone(LabId phone, int *nStates_quinphone, int *state_quinphone, HArc *a, int *frame_end); 
void GetTimes(LArc *larc, int i, int *start, int *end);   /*gets times as ints. */

/* EXPORT-> SetDoingFourthAcc: Indicate whether it is currently storing MMI statistics */
void SetDoingFourthAcc(Boolean DO, int indx);

/* ------------------------- End of HFBLat.h --------------------------- */





