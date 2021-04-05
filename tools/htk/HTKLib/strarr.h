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

#ifndef _STRARR_H_
#define _STRARR_H_

#ifdef __cplusplus
extern "C" {
#endif

char ** StrArrFromRect(long *dim, void *data);
void StrArrToRect(char **strarr, long **dimenp, void **datap);
int StrArrLen(char **str_arr);
int StrArrMaxLen(char **str_arr);

#ifdef __cplusplus
}
#endif

#endif  /* _STRARR_H_ */
