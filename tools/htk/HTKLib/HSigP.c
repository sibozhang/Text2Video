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
/*      File: HSigP.c:   Signal Processing Routines            */
/* ----------------------------------------------------------- */

char *hsigp_version = "!HVER!HSigP:   3.4.1 [CUED 12/03/09]";
char *hsigp_vc_id = "$Id: HSigP.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

#include "HShell.h"        /* HTK Libraries */
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"

/*
   This module provides a set of basic speech signal processing
   routines and feature level transformations.
*/

/* ------------------------ Trace Flags ------------------------- */

static int trace = 0;
#define T_MEL  0002     /* Mel filterbank */

/* -------------------- Config and Memory ----------------------- */

static MemHeap sigpHeap;
static ConfParam *cParm[MAXGLOBS];       /* config parameters */
static int numParm = 0;

/* ---------------------- Initialisation -------------------------*/

/* EXPORT->InitSigP: initialise the SigP module */
void InitSigP(void)
{
   int i;

   Register(hsigp_version,hsigp_vc_id);
   numParm = GetConfig("HSIGP", TRUE, cParm, MAXGLOBS);
   if (numParm>0){
      if (GetConfInt(cParm,numParm,"TRACE",&i)) trace = i;
   }
   CreateHeap(&sigpHeap,"sigpHeap",MSTAK,1,0.0,5000,5000);
}

/* --------------- Windowing and PreEmphasis ---------------------*/

/* ZeroMean: zero mean a complete speech waveform */
void ZeroMean(short *data, long nSamples)
{
   long i,hiClip=0,loClip=0;
   short *x;
   double sum=0.0,y,mean;

   x = data;
   for (i=0;i<nSamples;i++,x++)
      sum += *x;
   mean = sum / (float)nSamples;
   x = data;
   for (i=0;i<nSamples;i++,x++){
      y = (double)(*x) - mean;
      if (y<-32767.0){
         y = -32767.0; ++loClip;
      }
      if (y>32767.0){
         y = 32767.0; ++hiClip;
      }
      *x = (short) ((y>0.0) ? y+0.5 : y-0.5);
   }
   if (loClip>0)
      HError(-5322,"ZeroMean: %d samples too -ve\n",loClip);
   if (hiClip>0)
      HError(-5322,"ZeroMean: %d samples too +ve\n",hiClip);
}

static int hamWinSize = 0;          /* Size of current Hamming window */
static Vector hamWin = NULL;        /* Current Hamming window */

/* GenHamWindow: generate precomputed Hamming window function */
static void GenHamWindow (int frameSize)
{
   int i;
   float a;
   
   if (hamWin==NULL || VectorSize(hamWin) < frameSize)
      hamWin = CreateVector(&sigpHeap,frameSize);
   a = TPI / (frameSize - 1);
   for (i=1;i<=frameSize;i++)
      hamWin[i] = 0.54 - 0.46 * cos(a*(i-1));
   hamWinSize = frameSize;
}

/* EXPORT->Ham: Apply Hamming Window to Speech frame s */
void Ham (Vector s)
{
   int i,frameSize;
   
   frameSize=VectorSize(s);
   if (hamWinSize != frameSize)
      GenHamWindow(frameSize);
   for (i=1;i<=frameSize;i++)
      s[i] *= hamWin[i];
}

/* EXPORT->PreEmphasise: pre-emphasise signal in s */
void PreEmphasise (Vector s, float k)
{
   int i;
   float preE;
   
   preE = k;
   for (i=VectorSize(s);i>=2;i--)
      s[i] -= s[i-1]*preE;
   s[1] *= 1.0-preE;
}

/* --------------- Linear Prediction Coding Operations ------------- */

/* AutoCorrelate: store auto coef 1 to p in r and return energy r[0] */
static float AutoCorrelate(Vector s, Vector r, int p, int frameSize)
{
   float sum,energy;
   int   i,j;

   energy = 0.0;
   for (i=0;i<=p;i++) {
      sum = 0.0;
      for (j=1;j<=frameSize-i;j++)
         sum += s[j]*s[j+i];
      if (i==0) 
         energy = sum;
      else
         r[i] = sum;
   }
   return energy;
}

/* Durbins recursion to get LP coeffs for auto values */
static float Durbin(Vector k, Vector thisA, Vector r, float E, int order)
{
   Vector newA;
   float ki;         /* Current Reflection Coefficient */
   int i,j;
 
   newA  = CreateVector(&gstack,order);
   for (i=1;i<=order;i++) {
      ki = r[i];              /* Calc next reflection coef */
      for (j=1;j<i;j++)
         ki = ki + thisA[j] * r[i - j];
      ki = ki / E;   
      if (k!=NULL) k[i] = ki;
      E *= 1 - ki*ki;         /* Update Error */
      newA[i] = -ki;          /* Calc new filter coef */
      for (j=1;j<i;j++)
         newA[j] = thisA[j] - ki * thisA[i - j];
      for (j=1;j<=i;j++)   
         thisA[j] = newA[j];
   }
   FreeVector(&gstack,newA);
   return (E);
}

/* EXPORT->Wave2LPC: Calculate LPCoef in a & RefC in k */
void Wave2LPC (Vector s, Vector a, Vector k, float *re, float *te)
{
   Vector thisA;     /* Current LP filter coefficients */
   Vector r;         /* AutoCorrelation Sequence */
   float E;          /* Prediction Error */
   int   p,frameSize;

   if (a==NULL && k==NULL)
      HError(5320,"Wave2LPC: Null a and k vectors in WaveToLPC");  
   if (a!=NULL) 
      p=VectorSize(a); 
   else
      p=VectorSize(k);
   r = CreateVector(&gstack,p);
   thisA = (a!=NULL)?a:CreateVector(&gstack,p);
   frameSize=VectorSize(s);
   E = AutoCorrelate(s,r,p,frameSize);
   *te = E;
   *re = Durbin(k,thisA,r,E,p);
   FreeVector(&gstack,r);
}

/* EXPORT->LPC2RefC: transfer from filter to ref coef */
void LPC2RefC(Vector a, Vector k)
{
   Vector thisA; /* Current LP filter coefficients */
   Vector newA;  /* New LP filter coefficients */
   int i,j,p;
   float ki,x;
   
   p=VectorSize(a);
   thisA = CreateVector(&gstack,p);
   newA  = CreateVector(&gstack,p);
   CopyVector(a,thisA);
   for (i=p;i>=1;i--)  { 
      ki = -thisA[i];
      k[i] = ki;
      x = 1 - ki*ki;
      for (j=1;j<i;j++) 
         newA[j] = (thisA[j] + ki * thisA[i - j]) / x;
      for (j=1;j<i;j++) 
         thisA[j] = newA[j];
   }
   FreeVector(&gstack,thisA);
}

/* EXPORT->RefC2LPC: transfer from ref coef to filter */
void RefC2LPC (Vector k, Vector a)
{
   Vector thisA; /* Current LP filter coefficients */
   Vector newA;  /* New LP filter coefficients */
   int i,j,p;
   float ki;
   
   p=VectorSize(k);
   thisA = CreateVector(&gstack,p);
   newA  = CreateVector(&gstack,p);
   for (i=1;i<=p;i++) { 
      ki = k[i];
      newA[i] = -ki;
      for (j=1;j<i;j++) 
         newA[j] = thisA[j] - ki * thisA[i - j];
      for (j=1;j<=i;j++)   
         thisA[j] = newA[j];
   }
   for (i=1;i<=p;i++)  a[i]=thisA[i];
   FreeVector(&gstack,thisA);
}

/* EXPORT->LPC2Cepstrum: transfer from lpc to cepstral coef */
void LPC2Cepstrum (Vector a, Vector c)
{
   int i,n,p;
   float sum;
   
   p=VectorSize(c);
   for (n=1;n<=p;n++)  { 
      sum = 0.0;
      for (i=1;i<n;i++) 
         sum = sum + (n - i) * a[i] * c[n - i];
      c[n] = -(a[n] + sum / n);
   }
}

/* EXPORT->Cepstrum2LPC: transfer from cepstral coef to lpc */
void Cepstrum2LPC (Vector c, Vector a)
{
   int i,n,p;
   float sum;
   
   p=VectorSize(a);
   for (n=1;n<=p;n++)  { 
      sum = 0.0;
      for (i=1;i<n;i++) 
         sum = sum + (n - i) * a[i] * c[n - i];
      a[n] = -(c[n] + sum / n);
   }
}

/* -------------------- FFT Based Operations ----------------------- */

/* EXPORT-> FVec2Spectrum: cvt feature vector f to a spectrum, fzero
   is the value of the 0'th feature vector coefficient which
   is typically omitted by HSigP routines eg a0 = 1.0 for LPC
*/
void FVec2Spectrum (float fzero, Vector f, Vector s)
{
   int i,p,n;
   
   p=VectorSize(f); n=VectorSize(s);
   s[1] = fzero;
   for (i=1;i<=p;i++) 
      s[i+1] = f[i];
   for (i=p+2;i<=n;i++) 
      s[i] = 0.0;
   Realft(s);
}

/* EXPORT-> FFT: apply fft/invfft to complex s */
void FFT(Vector s, int invert)
{
   int ii,jj,n,nn,limit,m,j,inc,i;
   double wx,wr,wpr,wpi,wi,theta;
   double xre,xri,x;
   
   n=VectorSize(s);
   nn=n / 2; j = 1;
   for (ii=1;ii<=nn;ii++) {
      i = 2 * ii - 1;
      if (j>i) {
         xre = s[j]; xri = s[j + 1];
         s[j] = s[i];  s[j + 1] = s[i + 1];
         s[i] = xre; s[i + 1] = xri;
      }
      m = n / 2;
      while (m >= 2  && j > m) {
         j -= m; m /= 2;
      }
      j += m;
   };
   limit = 2;
   while (limit < n) {
      inc = 2 * limit; theta = TPI / limit;
      if (invert) theta = -theta;
      x = sin(0.5 * theta);
      wpr = -2.0 * x * x; wpi = sin(theta); 
      wr = 1.0; wi = 0.0;
      for (ii=1; ii<=limit/2; ii++) {
         m = 2 * ii - 1;
         for (jj = 0; jj<=(n - m) / inc;jj++) {
            i = m + jj * inc;
            j = i + limit;
            xre = wr * s[j] - wi * s[j + 1];
            xri = wr * s[j + 1] + wi * s[j];
            s[j] = s[i] - xre; s[j + 1] = s[i + 1] - xri;
            s[i] = s[i] + xre; s[i + 1] = s[i + 1] + xri;
         }
         wx = wr;
         wr = wr * wpr - wi * wpi + wr;
         wi = wi * wpr + wx * wpi + wi;
      }
      limit = inc;
   }
   if (invert)
      for (i = 1;i<=n;i++) 
         s[i] = s[i] / nn;
   
}

/* EXPORT-> Realft: apply fft to real s */
void Realft (Vector s)
{
   int n, n2, i, i1, i2, i3, i4;
   double xr1, xi1, xr2, xi2, wrs, wis;
   double yr, yi, yr2, yi2, yr0, theta, x;

   n=VectorSize(s) / 2; n2 = n/2;
   theta = PI / n;
   FFT(s, FALSE);
   x = sin(0.5 * theta);
   yr2 = -2.0 * x * x;
   yi2 = sin(theta); yr = 1.0 + yr2; yi = yi2;
   for (i=2; i<=n2; i++) {
      i1 = i + i - 1;      i2 = i1 + 1;
      i3 = n + n + 3 - i2; i4 = i3 + 1;
      wrs = yr; wis = yi;
      xr1 = (s[i1] + s[i3])/2.0; xi1 = (s[i2] - s[i4])/2.0;
      xr2 = (s[i2] + s[i4])/2.0; xi2 = (s[i3] - s[i1])/2.0;
      s[i1] = xr1 + wrs * xr2 - wis * xi2;
      s[i2] = xi1 + wrs * xi2 + wis * xr2;
      s[i3] = xr1 - wrs * xr2 + wis * xi2;
      s[i4] = -xi1 + wrs * xi2 + wis * xr2;
      yr0 = yr;
      yr = yr * yr2 - yi  * yi2 + yr;
      yi = yi * yr2 + yr0 * yi2 + yi;
   }
   xr1 = s[1];
   s[1] = xr1 + s[2];
   s[2] = 0.0;
}
   
/* EXPORT-> SpecModulus: store modulus of s in m */
void SpecModulus(Vector s, Vector m)
{
   int i,j;
   float x,y;
   
   for (i=1;i<=VectorSize(s)/2;i++) {
      j=i+i; x=s[j-1]; y=s[j];
      m[i]=sqrt(x*x + y*y);
   }
}

/* EXPORT-> SpecLogModulus: store log modulus of s in m */
void SpecLogModulus(Vector s, Vector m, Boolean invert)
{
   int i,j;
   float x,y;
   
   for (i=1;i<=VectorSize(s)/2;i++) {
      j=i+i; x=s[j-1]; y=s[j];
      x=0.5*log(x*x + y*y);
      m[i] = invert ? -x : x;
   }
}

/* EXPORT-> SpecPhase: store phase of s in m */
void SpecPhase(Vector s, Vector m)
{
   int i,j;
   float ph,re,im;
   
   for (i=1;i<=VectorSize(s)/2;i++) {
      j=i+i;
      re=s[j-1]; im=s[j];
      if (re==0.0) 
         ph = (im>=0.0) ? PI/2.0 : -PI/2.0;
      else {
         ph=atan(im/re);
         if (ph<0.0 && re<0.0)
            ph += PI;
         else if (ph>0.0 && im<0.0)
            ph -= PI;
      }
      m[i]=ph;
   }
}

/* -------------------- MFCC Related Operations -------------------- */

/* EXPORT->Mel: return mel-frequency corresponding to given FFT index */
float Mel(int k,float fres)
{
   return 1127 * log(1 + (k-1)*fres);
}

/* EXPORT->WarpFreq: return warped frequency */
float WarpFreq (float fcl, float fcu, float freq, float minFreq, float maxFreq , float alpha)
{
   if (alpha == 1.0)
      return freq;
   else {
      float scale = 1.0 / alpha;
      float cu = fcu * 2 / (1 + scale);
      float cl = fcl * 2 / (1 + scale);

      float au = (maxFreq - cu * scale) / (maxFreq - cu);
      float al = (cl * scale - minFreq) / (cl - minFreq);
      
      if (freq > cu)
         return  au * (freq - cu) + scale * cu ;
      else if (freq < cl)
         return al * (freq - minFreq) + minFreq ;
      else
         return scale * freq ;
   }
}

/* EXPORT->InitFBank: Initialise an FBankInfo record */
FBankInfo InitFBank(MemHeap *x, int frameSize, long sampPeriod, int numChans,
                    float lopass, float hipass, Boolean usePower, Boolean takeLogs,
                    Boolean doubleFFT,
                    float alpha, float warpLowCut, float warpUpCut)
{
   FBankInfo fb;
   float mlo,mhi,ms,melk;
   int k,chan,maxChan,Nby2;

   /* Save sizes to cross-check subsequent usage */
   fb.frameSize = frameSize; fb.numChans = numChans;
   fb.sampPeriod = sampPeriod; 
   fb.usePower = usePower; fb.takeLogs = takeLogs;
   /* Calculate required FFT size */
   fb.fftN = 2;   
   while (frameSize>fb.fftN) fb.fftN *= 2;
   if (doubleFFT) 
      fb.fftN *= 2;
   Nby2 = fb.fftN / 2;
   fb.fres = 1.0E7/(sampPeriod * fb.fftN * 700.0);
   maxChan = numChans+1;
   /* set lo and hi pass cut offs if any */
   fb.klo = 2; fb.khi = Nby2;       /* apply lo/hi pass filtering */
   mlo = 0; mhi = Mel(Nby2+1,fb.fres);
   if (lopass>=0.0) {
      mlo = 1127*log(1+lopass/700.0);
      fb.klo = (int) ((lopass * sampPeriod * 1.0e-7 * fb.fftN) + 2.5);
      if (fb.klo<2) fb.klo = 2;
   }
   if (hipass>=0.0) {
      mhi = 1127*log(1+hipass/700.0);
      fb.khi = (int) ((hipass * sampPeriod * 1.0e-7 * fb.fftN) + 0.5);
      if (fb.khi>Nby2) fb.khi = Nby2;
   }
   if (trace&T_MEL){
      printf("FFT passband %d to %d out of 1 to %d\n",fb.klo,fb.khi,Nby2);
      printf("Mel passband %f to %f\n",mlo,mhi);
   }
   /* Create vector of fbank centre frequencies */
   fb.cf = CreateVector(x,maxChan);
   ms = mhi - mlo;
   for (chan=1; chan <= maxChan; chan++) {
      if (alpha == 1.0) {
         fb.cf[chan] = ((float)chan/(float)maxChan)*ms + mlo;
      }
      else {
         /* scale assuming scaling starts at lopass */
         float minFreq = 700.0 * (exp (mlo / 1127.0) - 1.0 );
         float maxFreq = 700.0 * (exp (mhi / 1127.0) - 1.0 );
         float cf = ((float)chan / (float) maxChan) * ms + mlo;
         
         cf = 700 * (exp (cf / 1127.0) - 1.0);
         
         fb.cf[chan] = 1127.0 * log (1.0 + WarpFreq (warpLowCut, warpUpCut, cf, minFreq, maxFreq, alpha) / 700.0);
      }
   }
   
   /* Create loChan map, loChan[fftindex] -> lower channel index */
   fb.loChan = CreateShortVec(x,Nby2);
   for (k=1,chan=1; k<=Nby2; k++){
      melk = Mel(k,fb.fres);
      if (k<fb.klo || k>fb.khi) fb.loChan[k]=-1;
      else {
         while (fb.cf[chan] < melk  && chan<=maxChan) ++chan;
         fb.loChan[k] = chan-1;
      }
   }

   /* Create vector of lower channel weights */   
   fb.loWt = CreateVector(x,Nby2);
   for (k=1; k<=Nby2; k++) {
      chan = fb.loChan[k];
      if (k<fb.klo || k>fb.khi) fb.loWt[k]=0.0;
      else {
         if (chan>0) 
            fb.loWt[k] = ((fb.cf[chan+1] - Mel(k,fb.fres)) / 
                          (fb.cf[chan+1] - fb.cf[chan]));
         else
            fb.loWt[k] = (fb.cf[1]-Mel(k,fb.fres))/(fb.cf[1] - mlo);
      }
   }
   /* Create workspace for fft */
   fb.x = CreateVector(x,fb.fftN);
   return fb;
}

/* EXPORT->Wave2FBank:  Perform filterbank analysis on speech s */
void Wave2FBank(Vector s, Vector fbank, float *te, FBankInfo info)
{
   const float melfloor = 1.0;
   int k, bin;
   float t1,t2;   /* real and imag parts */
   float ek;      /* energy of k'th fft channel */
   
   /* Check that info record is compatible */
   if (info.frameSize != VectorSize(s))
      HError(5321,"Wave2FBank: frame size mismatch");
   if (info.numChans != VectorSize(fbank))
      HError(5321,"Wave2FBank: num channels mismatch");
   /* Compute frame energy if needed */
   if (te != NULL){
      *te = 0.0;  
      for (k=1; k<=info.frameSize; k++) 
         *te += (s[k]*s[k]);
   }
   /* Apply FFT */
   for (k=1; k<=info.frameSize; k++) 
      info.x[k] = s[k];    /* copy to workspace */
   for (k=info.frameSize+1; k<=info.fftN; k++) 
      info.x[k] = 0.0;   /* pad with zeroes */
   Realft(info.x);                            /* take fft */

   /* Fill filterbank channels */
   ZeroVector(fbank); 
   for (k = info.klo; k <= info.khi; k++) {             /* fill bins */
      t1 = info.x[2*k-1]; t2 = info.x[2*k];
      if (info.usePower)
         ek = t1*t1 + t2*t2;
      else
         ek = sqrt(t1*t1 + t2*t2);
      bin = info.loChan[k];
      t1 = info.loWt[k]*ek;
      if (bin>0) fbank[bin] += t1;
      if (bin<info.numChans) fbank[bin+1] += ek - t1;
   }

   /* Take logs */
   if (info.takeLogs)
      for (bin=1; bin<=info.numChans; bin++) { 
         t1 = fbank[bin];
         if (t1<melfloor) t1 = melfloor;
         fbank[bin] = log(t1);
      }
}

/* EXPORT->FBank2MFCC: compute first n cepstral coeff */
void FBank2MFCC(Vector fbank, Vector c, int n)
{
   int j,k,numChan;
   float mfnorm,pi_factor,x;
   
   numChan = VectorSize(fbank);
   mfnorm = sqrt(2.0/(float)numChan);
   pi_factor = PI/(float)numChan;
   for (j=1; j<=n; j++)  {
      c[j] = 0.0; x = (float)j * pi_factor;
      for (k=1; k<=numChan; k++)
         c[j] += fbank[k] * cos(x*(k-0.5));
      c[j] *= mfnorm;
   }        
}

/* EXPORT->FBank2MelSpec: convert log fbank to linear */
void FBank2MelSpec(Vector fbank)
{
   int i;
   
   for (i=1; i<=VectorSize(fbank); i++)
      fbank[i] = exp(fbank[i]);
}
   

/* EXPORT->MelSpec2FBank: convert lin mel spectrum to log fbank */
void MelSpec2FBank(Vector melspec)
{
   int i;
   float x;
   
   for (i=1; i<=VectorSize(melspec); i++){
      x = melspec[i];
      if (x<1.0) x = 1.0;
      melspec[i] = log(x);
   }
}

/* EXPORT->FBank2C0: return zero'th cepstral coefficient */
float FBank2C0(Vector fbank)
{
   int k,numChan;
   float mfnorm,sum;
   
   numChan = VectorSize(fbank);
   mfnorm = sqrt(2.0/(float)numChan);
   sum = 0.0; 
   for (k=1; k<=numChan; k++)
      sum += fbank[k];
   return sum * mfnorm;
}

/* --------------------- PLP Related Operations -------------------- */

/* EXPORT->InitPLP: Initialise equal-loudness curve & IDT cosine matrix */
void InitPLP (FBankInfo info, int lpcOrder, Vector eql, DMatrix cm)
{
   int i,j;
   double baseAngle;
   float f_hz_mid, fsub, fsq;
   int  nAuto, nFreq;

   /* Create the equal-loudness curve */
   for (i=1; i<=info.numChans; i++) {
      f_hz_mid = 700*(exp(info.cf[i]/1127)-1); /* Mel to Hz conversion */
      fsq = (f_hz_mid * f_hz_mid);
      fsub = fsq / (fsq + 1.6e5);
      eql[i] = fsub * fsub * ((fsq + 1.44e6)  /(fsq + 9.61e6));
   }

   /* Builds up matrix of cosines for IDFT */
   nAuto = lpcOrder+1; 
   nFreq = info.numChans+2;
   baseAngle =  PI / (double)(nFreq - 1);
   for (i=0; i<nAuto; i++) {
      cm[i+1][1] = 1.0;
      for (j=1; j<(nFreq-1); j++)
         cm[i+1][j+1] = 2.0 * cos(baseAngle * (double)i * (double)j);

      cm[i+1][nFreq] = cos(baseAngle * (double)i * (double)(nFreq-1));
   }
}

/* EXPORT->FBank2ASpec: Pre-emphasise filter bank output with the simulated 
           equal-loudness curve and perform amplitude compression */
void FBank2ASpec (Vector fbank, Vector as, Vector eql, float compressFact, 
                  FBankInfo info)
{
   const float melfloor = 1.0;
   int i;

   for (i=1; i<=info.numChans; i++) {
      if (fbank[i] < melfloor) fbank[i] = melfloor;
      as[i+1] = fbank[i] * eql[i]; /* Apply equal-loudness curve */
      as[i+1] = pow((double) as[i+1], (double) compressFact);
   }
   as[1] = as[2];  /* Duplicate values at either end */
   as[info.numChans+2] = as[info.numChans+1];
}


/* Matrix IDFT converts from auditory spectrum into autocorrelation values */
float MatrixIDFT(Vector as, Vector ac, DMatrix cm)
{
   double acc;
   float E;
   int nAuto, nFreq;
   int i, j;

   nFreq = VectorSize(as);
   nAuto = VectorSize(ac);
   E=0.0;
   for (i=0; i<nAuto; i++) {
      acc = cm[i+1][1] * (double)as[1];
      for (j=1; j<nFreq; j++)
         acc += cm[i+1][j+1] * (double)as[j+1];

      if (i>0) 
         ac[i] = (float)(acc / (double)(2.0 * (nFreq-1)));
      else  
         E = (float)(acc / (double)(2.0 * (nFreq-1)));
   }     
   return E; /* Return zero'th auto value separately */
}

/* EXPORT->ASpec2LPCep: Perform IDFT to get autocorrelation values then 
           produce autoregressive coeffs. and cepstral transform them */
void ASpec2LPCep (Vector as, Vector ac, Vector lp, Vector c, DMatrix cm)
{
   float lpcGain, E;

   /* Do IDFT to get autocorrelation values */
   E = MatrixIDFT(as, ac, cm);
   lp[VectorSize(lp)] = 0.0;    /* init to make Purify et al. happy */
   /* do Durbin recursion to get predictor coefficients */
   lpcGain = Durbin(NULL,lp,ac,E,VectorSize(ac)-1);
   if (lpcGain<=0) 
      HError(-5323,"ASpec2LPCep: Negative lpcgain");
   LPC2Cepstrum(lp,c);
   c[VectorSize(c)] = (float) -log((double) 1.0/lpcGain); /* value forms C0 */
}

/* ------------------- Feature Level Operations -------------------- */

static int cepWinSize=0;            /* Size of current cepstral weight window */
static int cepWinL=0;               /* Current liftering coeff */
static Vector cepWin = NULL;        /* Current cepstral weight window */

/* GenCepWin: generate a new cep liftering vector */
static void GenCepWin (int cepLiftering, int count)
{
   int i;
   float a, Lby2;
   
   if (cepWin==NULL || VectorSize(cepWin) < count)
      cepWin = CreateVector(&sigpHeap,count);
   a = PI/cepLiftering;
   Lby2 = cepLiftering/2.0;
   for (i=1;i<=count;i++)
      cepWin[i] = 1.0 + Lby2*sin(i * a);
   cepWinL = cepLiftering;
   cepWinSize = count;
}  

/* EXPORT->WeightCepstrum: Apply cepstral weighting to c */
void WeightCepstrum (Vector c, int start, int count, int cepLiftering)
{
   int i,j;
   
   if (cepWinL != cepLiftering || count > cepWinSize)
      GenCepWin(cepLiftering,count);
   j = start;
   for (i=1;i<=count;i++)
      c[j++] *= cepWin[i];
}

/* EXPORT->UnWeightCepstrum: Undo cepstral weighting of c */
void UnWeightCepstrum(Vector c, int start, int count, int cepLiftering)
{
   int i,j;
   
   if (cepWinL != cepLiftering || count > cepWinSize)
      GenCepWin(cepLiftering,count);
   j = start;
   for (i=1;i<=count;i++)
      c[j++] /= cepWin[i];
}

/* The following operations apply to a sequence of n vectors step apart.
   They are used to operate on the 'columns' of data files 
   containing a sequence of feature vectors packed together to form a
   continguous block of floats.  The logical size of each vector is
   vSize (<=step) */

/* EXPORT->FZeroMean: Zero mean the given data sequence */
void FZeroMean(float *data, int vSize, int n, int step)
{
   double sum;
   float *fp,mean;
   int i,j;

   for (i=0; i<vSize; i++){
      /* find mean over i'th component */
      sum = 0.0;
      fp = data+i;
      for (j=0;j<n;j++){
         sum += *fp; fp += step;
      }
      mean = sum / (double)n;
      /* subtract mean from i'th components */
      fp = data+i;
      for (j=0;j<n;j++){
         *fp -= mean; fp += step;
      }
   }
}

/* Regression: add regression vector at +offset from source vector.  If head
   or tail is less than delwin then duplicate first/last vector to compensate */
static void Regress(float *data, int vSize, int n, int step, int offset,
                    int delwin, int head, int tail, Boolean simpleDiffs)
{
   float *fp,*fp1,*fp2, *back, *forw;
   float sum, sigmaT2;
   int i,t,j;
   
   sigmaT2 = 0.0;
   for (t=1;t<=delwin;t++)
      sigmaT2 += t*t;
   sigmaT2 *= 2.0;
   fp = data;
   for (i=1;i<=n;i++){
      fp1 = fp; fp2 = fp+offset;
      for (j=1;j<=vSize;j++){
         back = forw = fp1; sum = 0.0;
         for (t=1;t<=delwin;t++) {
            if (head+i-t > 0)     back -= step;
            if (tail+n-i+1-t > 0) forw += step;
            if (!simpleDiffs) sum += t * (*forw - *back);
         }
         if (simpleDiffs)
            *fp2 = (*forw - *back) / (2*delwin);
         else
            *fp2 = sum / sigmaT2;
         ++fp1; ++fp2;
      }
      fp += step;
   }
}


/* EXPORT->AddRegression: add regression vector at +offset from source vector */
void AddRegression(float *data, int vSize, int n, int step, int offset,
                   int delwin, int head, int tail, Boolean simpleDiffs)
{
   Regress(data,vSize,n,step,offset,delwin,head,tail,simpleDiffs);
}

/* EXPORT->AddHeadRegress: add regression at start of data */
void AddHeadRegress(float *data, int vSize, int n, int step, int offset,
                    int delwin, Boolean simpleDiffs)
{
   float *fp,*fp1,*fp2;
   int i,j;

   fp = data;
   if (delwin==0){
      for (i=1;i<=n;i++){
         fp1 = fp; fp2 = fp+offset;
         for (j=1;j<=vSize;j++){
            *fp2 = *(fp1+step) - *fp1;
            ++fp1; ++fp2;
         }
         fp += step;
      }
   }else{
      Regress(data,vSize,n,step,offset,delwin,0,delwin,simpleDiffs);
   }
}

/* EXPORT->AddTailRegress: add regression at end of data */
void AddTailRegress(float *data, int vSize, int n, int step, int offset,
                    int delwin, Boolean simpleDiffs)
{
   float *fp,*fp1,*fp2;
   int i,j;

   fp = data;
   if (delwin==0){
      for (i=1;i<=n;i++){
         fp1 = fp; fp2 = fp+offset;
         for (j=1;j<=vSize;j++){
            *fp2 = *fp1 - *(fp1-step);
            ++fp1; ++fp2;
         }
         fp += step;
      }
   }else{
      Regress(data,vSize,n,step,offset,delwin,delwin,0,simpleDiffs);      
   }
}

/* EXPORT->NormaliseLogEnergy: normalise log energy to range -X .. 1.0 */
void NormaliseLogEnergy(float *data,int n,int step,float silFloor,float escale)
{
   float *p,max,min;
   int i;

   /* find max log energy */
   p = data; max = *p;
   for (i=1;i<n;i++){
      p += step;                   /* step p to next e val */
      if (*p > max) max = *p;
   }
   min = max - (silFloor*log(10.0))/10.0;  /* set the silence floor */
   /* normalise */
   p = data;
   for (i=0;i<n;i++){
      if (*p < min) *p = min;          /* clamp to silence floor */
      *p = 1.0 - (max - *p) * escale;  /* normalise */
      p += step; 
   }
}


/* ------------------------ End of HSigP.c ------------------------- */
