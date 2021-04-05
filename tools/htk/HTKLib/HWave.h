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
/*         File: HWave.h: Speech Waveform File Input           */
/* ----------------------------------------------------------- */
/* !HVER!HWave:   3.4.1 [CUED 12/03/09] */

/*  Configuration Parameters:
   NSAMPLES       - num samples in alien file input via a pipe
   HEADERSIZE     - size of header in alien file
   BYTEORDER      - define byte order VAX or other
   STEREOMODE     - select LEFT or RIGHT chan (default both)
   SOURCERATE     - sample period of source
   TARGETRATE     - sample period of target
   SOURCEFORMAT   - input file format
   TARGETFORMAT   - output file format
   TRACE          - trace level
*/

#ifndef _HWAVE_H_
#define _HWAVE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Added for esignal supprt */
typedef struct FieldSpec **HFieldList;

typedef enum {
        NOHEAD,            /* Headerless File */
        HAUDIO,            /* Direct Audio Input */
        HTK,               /* used for both wave and parm files */
        TIMIT,             /* Prototype TIMIT database */
        NIST,              /* NIST databases eg RM1,TIMIT */
        SCRIBE,            /* UK Scribe databases */
        AIFF,              /* Apple Audio Interchange format */
        SDES1,             /* Sound Designer I format */
        SUNAU8,            /* Sun 8 bit MuLaw .au format */
        OGI,               /* Oregon Institute format (similar to TIMIT) */
        ESPS,              /* used for both wave and parm files */
	ESIG,              /* used for both wave and parm files */
	WAV,               /* Microsoft WAVE format */
        UNUSED,
        ALIEN,             /* Unknown */
        UNDEFF
} FileFormat;

typedef struct _Wave *Wave;  /* Abstract type representing waveform file */

void InitWave(void);
/*
   Initialise module
*/

Wave OpenWaveInput(MemHeap *x, char *fname, FileFormat fmt, HTime winDur, 
                   HTime frPeriod, HTime *sampPeriod);
/*
   Open the named input file with the given format and return a
   Wave object. If fmt==UNDEFF then the value of the configuration
   parameter SOURCEFORMAT is used.  If this is not set, then the format
   HTK is assumed. Samples are returned in frames of duration winDur.  The 
   period between successive frames is given by frPeriod.  If the value of 
   sampPeriod is not 0.0, then it overrides the sample period specified in
   the file, otherwise the actual value is returned. Returns NULL on error.
*/

void CloseWaveInput(Wave w);
/* 
   Terminate Wave input and free any resources associated with w
*/

void ZeroMeanWave(Wave w);
/*
   Ensure that mean of wave w is zero
*/

int FramesInWave(Wave w);
/*
   Return number of whole frames which are currently
   available in the given Wave
*/

int SampsInWaveFrame(Wave w);
/*
   Return number of samples in each frame of the given Wave
*/

void GetWave(Wave w, int nFrames, float *buf);
/* 
   Get next nFrames from Wave input buffer and store sequentially in
   buf as floats.  If a frame overlap has been set then samples will be
   duplicated in buf.  It is a fatal error to request more frames
   than exist in the Wave (as determined by FramesInWave.
*/

short *GetWaveDirect(Wave w, long *nSamples);
/* 
   Returns a pointer to the waveform stored in w.
*/

Wave OpenWaveOutput(MemHeap *x, HTime *sampPeriod, long bufSize);
/*
   Initialise a Wave object to store waveform data at the given 
   sample period, using buffer of bufSize shorts.  
*/

void PutWaveSample(Wave w, long nSamples, short *buf);
/*
   Append given nSamples in buf to wave w.
*/

ReturnStatus CloseWaveOutput(Wave w, FileFormat fmt, char *fname);
/* 
   Output wave w to file fname in given fmt and free any 
   associated resources.  If fmt==UNDEFF then value of
   configuration variable TARGETFORMAT is used, if any,
   otherwise the HTK format is used. If an error then 
   returns FAIL and does not free any memory. 
*/

FileFormat WaveFormat(Wave w);
/* 
   Return format of given wave
*/

char *Format2Str(FileFormat format);
FileFormat Str2Format(char *fmt);
/*
   Convert between FileFormat enum type & string.
*/

/* --------------------- HTK Header Routines --------------------- */

Boolean ReadHTKHeader(FILE *f,long *nSamp,long *sampP,short *sampS,
                      short *kind, Boolean *bSwap);
/* 
   Get header info from HTK file f, return false if apparently not
   a HTK file.  If byte-swapped bswap returns true.  NB only
   the user can specify required byte order via NATREADORDER config var 
   since it is not defined for HTK files)
*/

void WriteHTKHeader(FILE *f, long nSamp, long sampP, short sampS, 
		    short kind, Boolean *bSwap);
/* 
   Write header info to HTK file f.  
   Sets bSwap to indicate whether header was byte swapped before writing.
*/

void StoreESIGFieldList(HFieldList fList);
/*
   Store the field list of an ESIG input file 
*/
void RetrieveESIGFieldList(HFieldList *fList);
/*
   Retrieve the field list of an ESIG input file 
*/

Boolean ReadEsignalHeader(FILE *f, long *nSamp, long *sampP, short *sampS,
 			  short *kind, Boolean *bSwap, long *hdrS,
 			  Boolean isPipe);
/*
    Get header from Esignal file f; return FALSE in case of failure.
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HWAVE_H_ */

/* ------------------------ End of HWave.h ----------------------- */

