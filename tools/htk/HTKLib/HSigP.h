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
/*              2001  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*      File: HSigP.h:   Signal Processing Routines            */
/* ----------------------------------------------------------- */

/* !HVER!HSigP:   3.4.1 [CUED 12/03/09] */

#ifndef _HSIGP_H_
#define _HSIGP_H_

#ifdef __cplusplus
extern "C" {
#endif

void InitSigP(void);
/*
   Initialise the signal processing module.  This must be called
   before any other operation
*/

/* --------------- Speech Signal Processing Operations ------------- */

void ZeroMean(short *data, long nSamples);
/* 
   zero mean a complete speech waveform nSamples long
*/

void Ham (Vector s);
/*
   Apply Hamming Window to Speech frame s
*/

void PreEmphasise (Vector s, float k);
/*
   Apply first order preemphasis filter y[n] = x[n] - K*x[n-1] to s
*/

/* --------------- Linear Prediction Coding Operations ------------- */

void Wave2LPC (Vector s, Vector a, Vector k, float *re, float *te);
/*
   Calculate LP Filter Coef in a and LP Refl Coef in k from speech s
   Either a and k can be NULL.  Residual Energy is returned in re and
   total energy in te.
*/

void LPC2RefC (Vector a, Vector k);
void RefC2LPC (Vector k, Vector a);
/*
   Convert between filter and reflection coefs 
*/

void LPC2Cepstrum (Vector a, Vector c);
void Cepstrum2LPC (Vector c, Vector a);
/*
   Convert between LP Cepstral Coef in c and LP Coef in a
*/

/* -------------------- FFT Based Operations ----------------------- */

void FVec2Spectrum (float fzero, Vector f, Vector s);
/*
   Pads f with zeroes and applies FFT to give spectrum s
   Only the useful half of the spectrum is returned eg 
   if VectorSize(s)=128 then a 128 point FFT will be used
   but only the first 64 complex points are returned in s.
   fzero is the value of the 0'th feature vector coefficient 
   which is typically omitted by HSigP routines eg a0 = 1.0 
   for LPC
*/

void FFT(Vector s, int invert);
/*
   When called s holds nn complex values stored in the
   sequence   [ r1 , i1 , r2 , i2 , .. .. , rn , in ] where
   n = VectorSize(s) DIV 2, n must be a power of 2. On exit s
   holds the fft (or the inverse fft if invert == 1) 
*/

void Realft (Vector s);
/*
   When called s holds 2*n real values, on exit s holds the
   first  n complex points of the spectrum stored in
   the same format as for fft
*/
   
void SpecModulus(Vector s, Vector m);
void SpecLogModulus(Vector s, Vector m, Boolean invert);
void SpecPhase(Vector s, Vector m);
/*
   On entry, s should hold n complex points; VectorSize(s)=n*2
   On return, m holds (log) modulus/phase of s in first n points
*/

/* -------------------- MFCC Related Operations -------------------- */

typedef struct{
   int frameSize;       /* speech frameSize */
   int numChans;        /* number of channels */
   long sampPeriod;     /* sample period */
   int fftN;            /* fft size */
   int klo,khi;         /* lopass to hipass cut-off fft indices */
   Boolean usePower;    /* use power rather than magnitude */
   Boolean takeLogs;    /* log filterbank channels */
   float fres;          /* scaled fft resolution */
   Vector cf;           /* array[1..pOrder+1] of centre freqs */
   ShortVec loChan;     /* array[1..fftN/2] of loChan index */
   Vector loWt;         /* array[1..fftN/2] of loChan weighting */
   Vector x;            /* array[1..fftN] of fftchans */
}FBankInfo;

float Mel(int k, float fres);
/* 
   return mel-frequency corresponding to given FFT index k.  
   Resolution is normally determined by fres field of FBankInfo
   record.
*/

FBankInfo InitFBank(MemHeap *x, int frameSize, long sampPeriod, int numChans,
                    float lopass, float hipass, Boolean usePower, Boolean takeLogs,
                    Boolean doubleFFT,
                    float alpha, float warpLowCut, float warpUpCut);
/*
   Initialise an FBankInfo record prior to calling Wave2FBank.
*/


void Wave2FBank(Vector s, Vector fbank, float *te, FBankInfo info);
/*
   Convert given speech frame in s into mel-frequency filterbank
   coefficients.  The total frame energy is stored in te.  The
   info record contains precomputed filter weights and should be set
   prior to using Wave2FBank by calling InitFBank.
*/

void FBank2MFCC(Vector fbank, Vector c, int n);
/*
   Apply the DCT to fbank and store first n cepstral coeff in c.
   Note that the resulting coef are normalised by sqrt(2/numChans)
*/ 

void FBank2MelSpec(Vector fbank);
/*
   Convert the given log filterbank coef, in place, to linear
*/ 

void MelSpec2FBank(Vector melspec);
/*
   Convert the given linear filterbank coef, in place, to log
*/ 

float FBank2C0(Vector fbank);
/*
   return zero'th cepstral coefficient for given filter bank, i.e.
   compute sum of fbank channels and do standard normalisation
*/


/* ------------------- PLP Related Operations ---------------------- */

void InitPLP(FBankInfo info, int lpcOrder, Vector eql, DMatrix cm);
/*
   Initialise equal-loudness curve and cosine matrix for IDFT
*/
void FBank2ASpec(Vector fbank, Vector as, Vector eql, float compressFact,
		 FBankInfo info);
/*
   Pre-emphasise with simulated equal-loudness curve and perform
   cubic root amplitude compression.
*/
void ASpec2LPCep(Vector as, Vector ac, Vector lp, Vector c, DMatrix cm);
/*
   Do IDFT giving autocorrelation values then do linear prediction
   and finally, transform into cepstral coefficients
*/

/* ------------------- Feature Level Operations -------------------- */

void WeightCepstrum (Vector c, int start, int count, int cepLiftering);
void UnWeightCepstrum(Vector c, int start, int count, int cepLiftering);
/*
   Apply weights w[1]..w[count] to c[start] to c[start+count-1] 
   where w[i] = 1.0 + (L/2.0)*sin(i*pi/L),  L=cepLiftering
*/

/* The following apply to a sequence of 'n' vectors 'step' floats apart  */

void FZeroMean(float *data, int vSize, int n, int step);
/* 
   Zero mean the given data sequence
*/

void AddRegression(float *data, int vSize, int n, int step, int offset, 
                   int delwin, int head, int tail, Boolean simpleDiffs);
/*
   Add regression vector at +offset from source vector.  

   Each regression component is given by Sum( t*(v[+t] - v[-t])) / 2*Sum(t*t) 
   where the sum ranges over 1 to delwin and v[+t/-t] is the corresponding 
   component t steps ahead/back assuming that this vector is in the valid
   range -head...n+tail.  If simple diffs is true, then slope is 
   calculated from (v[delwin] - v[-delwin])/(2*delwin).  
*/

void AddHeadRegress(float *data, int vSize, int n, int step, int offset, 
                    int delwin, Boolean simpleDiffs);
/* 
   As for AddRegression, but deals with start case where there are no
   previous frames to regress over (assumes that there are at least
   min(delwin,1) valid following frames).  If delwin==0, then a simple
   forward difference given by v[0] - v[-1] is used.  Otherwise, the first
   available frame in the window is replicated back in time to fill the
   window.
*/

void AddTailRegress(float *data, int vSize, int n, int step, int offset, 
                    int delwin, Boolean simpleDiffs);
/* 
   As for AddRegression, but deals with start case where there are no
   previous frames to regress over (assumes that there are at least
   min(delwin,1) valid preceding frames).  If delwin==0, then a simple
   forward difference given by v[0] - v[-1] is used.  Otherwise, the first
   available frame in the window is replicated back in time to fill the
   window.  
*/

void NormaliseLogEnergy(float *data, int n, int step, float silFloor, float escale);
/* 
   normalise log energy to range -X .. 1.0 by subtracting the max log
   energy and adding 1.0.  The lowest energy level is set by the value
   of silFloor which gives the ratio between the max and min energies
   in dB.  Escale is used to scale the normalised log energy.
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HSIGP_H_ */

/* ------------------------ End of HSigP.h ------------------------- */
