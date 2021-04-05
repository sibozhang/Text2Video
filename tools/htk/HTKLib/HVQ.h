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
/*              File: HVQ.h: Vector Quantisation               */
/* ----------------------------------------------------------- */

/* !HVER!HVQ:   3.4.1 [CUED 12/03/09] */

/*
   This module provides a datatype VQTable which is used to represent
   linear (flat) and binary tree VQ codebooks.  Externally, a VQ Table
   definition is stored in a file consisting of a header followed by a
   sequence of entries representing each tree node.  One tree is 
   built for each stream:
   
   header:
      magic type covkind numNodes numS sw1 sw2 ...
   node_entry:
      stream vqidx nodeId leftId rightId mean-vector [invcov matrix|vector]
      ...
   where 
      magic  = usually the original parmkind
      type   = 0 linTree, 1 binTree
      covkind   = 5 euclid, 1 inv diag covar, 2 inv full covar
      numNodes = total number of following node entries
      numS   = number of streams
      sw1,sw2 = width of each stream
      stream = stream number for this entry
      vqidx  = the vq code for this node
      nodeId,leftId,rightId = arbitrary but unique integers identifying
               each node in each tree.  Root id is always 1.
*/

#ifndef _HVQ_H_
#define _HVQ_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
   linTree,    /* linear flat codebook (right branching tree) */
   binTree     /* binary tree - every node has 0 or 2 offspring */
} TreeType;

typedef struct _VQNodeRec *VQNode;
typedef struct _VQNodeRec{
   short vqidx;         /* vq index of this node */
   Vector mean;         /* centre of this node */
   Covariance cov;      /* null or inverse variance or covariance */
   float gconst;        /* const part of log prob for INVDIAGC & FULLC */
   void * aux;          /* available to 'user' */
   short nid,lid,rid;   /* used for mapping between mem and ext def */
   VQNode left,right;   /* offspring, only right is used in linTree */
}VQNodeRec;

typedef struct _VQTabRec *VQTable;
typedef struct _VQTabRec {
   char * tabFN;        /* name of this VQ table */
   short magic;         /* magic num, usually the ParmKind */
   TreeType type;       /* linear or binary */
   CovKind ckind;       /* kind of covariance used, if any*/
   short swidth[SMAX];  /* sw[0]=num streams, sw[i]=width of stream i */
   short numNodes;      /* total num nodes in all sub trees */
   VQNode tree[SMAX];   /* 1 tree per stream */
   VQTable next;        /* used internally for housekeeping */
}VQTabRec;

void InitVQ(void);
/*
   Initialise module
*/

VQTable CreateVQTab(char *tabFN, short magic, TreeType type,
                    CovKind ck, short *swidth);
/*
   Create an empty VQTable with given attributes.
*/

VQNode CreateVQNode(short vqidx, short nid, short lid, short rid, 
                    Vector mean, CovKind ck, Covariance cov);
/*
   Create a VQ node with given values
*/

VQTable LoadVQTab(char *tabFN, short magic);
/*
   Create a VQTable in memory and load the entries stored in the
   given file.  The value of magic must match the corresponding
   entry in the definition file unless it is zero in which case
   it is ignored.
*/

void StoreVQTab(VQTable vqTab, char *tabFN);
/*
   Store the given VQTable in the specified definition file tabFN.
   If tabFN is NULL then the existing tabFN stored in the table is
   used.
*/

void PrintVQTab(VQTable vqTab);
/*
   Print the given VQTable.
*/

float VQNodeScore(VQNode n, Vector v, int size, CovKind ck);
/* 
   compute score between vector v and VQ node n, smallest score is
   best.  If ck is NULLC then a euclidean distance is used otherwise
   -ve log prob is used.
*/

void GetVQ(VQTable vqTab, int numS, Vector *fv, short *vq);
/*
   fv is an array 1..numS of parameter vectors, each is 
   quantised using vqTab and the resulting indices are stored
   in vq[1..numS].
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HVQ_H_ */

/* ------------------------ End of HVQ.h ----------------------- */
