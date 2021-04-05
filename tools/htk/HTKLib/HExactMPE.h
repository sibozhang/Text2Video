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
/*         File: HExactMPE.h   MPE implementation (exact)      */
/* ----------------------------------------------------------- */

/* !HVER!HExactMPE:   3.4.1 [CUED 12/03/09] */


/* A (rather long) routine called from HFBLat.c, relating to the 
   exact implementation of MPE.
*/
   

#define SUPPORT_EXACT_CORRECTNESS


void InitExactMPE(void); /* set configs. */


#ifdef SUPPORT_EXACT_CORRECTNESS
void DoExactCorrectness(FBLatInfo *fbInfo, Lattice *lat);   
#endif

