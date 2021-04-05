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
/*         File: HAudio.h:   Audio Input/Output                */
/* ----------------------------------------------------------- */

/* !HVER!HAudio:   3.4.1 [CUED 12/03/09] */

#ifndef _HAUDIO_H_
#define _HAUDIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NULLSIG   0   /* indicates no signals */

/* -- audio inputs -- */
#define HA_OUT_NONE    0x00   /* use whatever currently selected */
#define HA_OUT_SPEAKER 0x01   /* Internal speaker */
#define HA_OUT_PHONES  0x02   /* Headphones */
#define HA_OUT_LINE    0x04   /* Line level out */

/* -- audio outputs -- */
#define HA_IN_NONE  0x00      /* use whatever currently selected */
#define HA_IN_MIC   0x01      /* Microphone input */
#define HA_IN_LINE  0x02      /* Line level input */

typedef enum {
   AI_CLEARED,    /* Not sampling and buffer empty */
   AI_WAITSIG,    /* Wait for start signal */
   AI_SAMPLING,   /* Sampling speech and filling buffer */   
   AI_STOPPED,    /* Stopped but waiting for buffer to be emptied */
   AI_ERROR       /* Error state - eg buffer overflow */
}AudioInStatus;

typedef struct _AudioIn  *AudioIn;    /* Abstract audio input stream */
typedef struct _AudioOut *AudioOut;   /* Abstract audio output stream */

void InitAudio(void);
/*
   Initialise audio module
*/


AudioIn OpenAudioInput(MemHeap *x, HTime *sampPeriod, 
                       HTime winDur, HTime frPeriod);
/*
   Initialise and return an audio stream to sample with given period.
   Samples are returned in frames with duration winDur.  Period between
   successive frames is given by frPeriod.  If *sampPeriod is 0.0 then a 
   default period is used (eg it might be set by some type of audio 
   control panel) and this default period is assigned to *sampPeriod.  
   If audio is not supported then NULL is returned.
*/

void AttachReplayBuf(AudioIn a, int bufSize);
/*
   Attach a replay buffer of given size to a
*/

void StartAudioInput(AudioIn a, int sig);
/*
   if sig>NULLSIG then install sig and set AI_WAITSIG
   if sig<NULLSIG then wait for keypress
   Start audio device

   Signal handler: 
      if AI_WAITSIG then 
         Start audio device
      if AI_SAMPLING then
         Stop audio device and set AI_STOPPED
*/

void StopAudioInput(AudioIn a);
/*
   Stop audio device and set AI_STOPPED.
*/

void CloseAudioInput(AudioIn a);
/* 
   Terminate audio input stream and free buffer memory
*/

int FramesInAudio(AudioIn a);
/*
   CheckAudioInput and return number of whole frames which are 
   currently available from the given audio stream
*/

int SamplesInAudio(AudioIn a);
/*
   CheckAudioInput and return number of speech samples which are 
   currently available from the given audio stream
*/

AudioInStatus GetAIStatus(AudioIn a);
/*
   CheckAudioInput and return the current status of the given 
   audio stream
*/
float GetCurrentVol(AudioIn a);

int SampsInAudioFrame(AudioIn a);
/*
   Return number of samples in each frame of the given Audio
*/

void GetAudio(AudioIn a, int nFrames, float *buf);
/* 
   Get nFrames from given audio stream and store sequentially as
   floats in buf.  If a frame overlap has been set then samples will
   be duplicated in buf.  If a 'replay buffer' has been
   specified then samples are saved in this buffer (with wrap around).
   Every call to StartAudio resets this buffer.  If more frames
   are requested than are available, the call blocks.
*/

void GetRawAudio(AudioIn a, int nSamples, short *buf);
/* 
   Get nSamples from given audio stream and store sequentially in buf.
*/

int GetReplayBuf(AudioIn a, int nSamples, short *buf);
/* 
   Get upto nSamples from replay buffer and store sequentially in buf.
   Return num samples actually copied.
*/

AudioOut OpenAudioOutput(MemHeap *x, HTime *sampPeriod);
/*
   Initialise and return an audio stream for output at given sample
   rate.  If *sampPeriod is 0.0 then a default period is used (eg it might
   be set by some type of audio control panel) and this default
   period is assigned to *sampPeriod.  If audio is not supported then 
   NULL is returned.
*/

void StartAudioOutput(AudioOut a, long nSamples, short *buf);
/*
   Output nSamples to audio stream a using data stored in buf.
*/

void PlayReplayBuffer(AudioOut ao, AudioIn ai);
/*
   Output nSamples from ai's replay buffer to ao
*/

void CloseAudioOutput(AudioOut a);
/* 
   Terminate audio stream a
*/

void SetVolume(AudioOut a, int volume);
/*
   Set the volume of the given audio device. Volume range
   is 0 to 100.
*/

float GetCurrentVol(AudioIn a);
/* 
   Obtain current volume of audio input device
*/

int AudioDevInput(int *mask);
/* 
   Query/set audio device input - mic, linein, etc
*/

int AudioDevOutput(int *mask);
/* 
   Query/set audio device output - speaker, headphones, lineout, etc
*/

int SamplesToPlay(AudioOut a);
/*
   Return num samples left to play in output stream a
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HAUDIO_H_ */

/* ------------------------ End of HAudio.h ----------------------- */
