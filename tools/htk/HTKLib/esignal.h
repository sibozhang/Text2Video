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

/* !HVER!esignal:   3.4.1 [CUED 12/03/09] */
/*
 *
 * Example programs for Esignal public external file format.
 * Include file.
 *
 * Author:  Rod Johnson
 */


#ifndef _ESIGNAL_H_
#define _ESIGNAL_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CONSTANTS
 */

#define TRUE	1
#define FALSE	0

/* Preamble */

#define PREAM_MAX   8		/* longest preamble line (incl. '\n') */
#define MAGIC	    "Esignal"	/* "magic number" */
#define VERSION	    "0.0B"	/* Esignal external format version */


/* Characters */

#define DOT	'.'	/* Field name separator. */

/* Types */

#define	NO_TYPE		1
#define	ARRAY		2
#define	DOUBLE		3
#define	FLOAT		4
#define	LONG		5
#define	ULONG		6
#define	SHORT		7
#define	USHORT		8
#define	SCHAR		9
#define	UCHAR		10
#define	BOOL		11
#define	DOUBLE_COMPLEX	12
#define	FLOAT_COMPLEX	13
#define	LONG_COMPLEX	14
#define	SHORT_COMPLEX	15
#define	SCHAR_COMPLEX	16
#define	CHAR		17
#define	WCHAR		18

    /* length of longest type name (including terminating null) */

#define MAX_TYPE_LEN	sizeof("DOUBLE_COMPLEX")

/* Occurrence Classes */

#define	GLOBAL		1
#define	REQUIRED	2
#define	OPTIONAL	3
#define	VIRTUAL		4
#define	INCLUDED	5

/* Field_order? */

#define TYPE_ORDER	0
#define FIELD_ORDER	1

/* Format Variant */

#define UNKNOWN		0
#define EDR1		1
#define EDR2		2
#define NATIVE		3
#define ASCII		4

/*
 * TYPEDEFS AND STRUCTURES
 */

/* Types */

typedef unsigned long			Ulong;
typedef unsigned short			Ushort;
typedef signed char			Schar;
typedef unsigned char			Uchar;
typedef struct {double	real, imag;}	DoubleComplex;
typedef struct {float	real, imag;}	FloatComplex;
typedef struct {long	real, imag;}	LongComplex;
typedef struct {short	real, imag;}	ShortComplex;
typedef struct {Schar	real, imag;}	ScharComplex;
typedef unsigned char			Bool;
typedef unsigned short			Wchar;

/* Arrays */

typedef struct Array	Array;

struct Array {
    short       type;           /* data type code */
    short       rank;           /* number of dimensions */
    long        *dim;           /* vector of dimensions */
    void        *data;          /* storage area for data */
};

/* Field Specifications */

typedef struct FieldSpec    FieldSpec;
typedef FieldSpec	    **FieldList;

struct FieldSpec {
    short       type;           /* data type code */
    short       rank;           /* number of dimensions */
    long        *dim;           /* vector of dimensions */
    short       occurrence;     /* REQUIRED, GLOBAL, OPTIONAL, etc. */
    char        *name;          /* identifying character string */
    FieldList   subfields;      /* field specs of subfields */
    char        *units;         /* string giving physical units */
    double      scale, offset;  /* scale factor and offset relating
				   raw numbers to physical quantities */
    char        **axis_names;   /* optional strings identifying axes */
    void        *data;          /* GLOBAL data area */
    Bool	present;	/* is OPTIONAL field present in record? */
    char	*fullname;	/* name including parent name, if any,
				   prefixed with connecting "." */
};

/* Ascii Annotations */

typedef struct Annot	Annot;

struct Annot {
    int     position;
    int     indent;
    int     width;
    long    recnum;
};

/*
 * FUNCTION DECLARATIONS
 */

/*
 * Miscellaneous.
 */

char	    *StrDup(char *str);
long	    LongProd(int n, long *arr);

/*
 * Debug Output
 */

void 	    DebugPrint(char *msg);

/*
 * Types.
 */

int	    ValidType(int type);
char	    *TypeName(int type);
int	    TypeCode(char *name);
long	    InternTypeSize(int type);
long	    ExternTypeSize(int type, int arch);

/*
 * Field specifications and lists.
 */

FieldSpec   *NewFieldSpec(int type, int rank);
void	    FreeFieldSpec(FieldSpec *spec);
void	    FreeFieldList(FieldList list);
void	    FreeAxisNames(char **axis_names, int rank);
long	    FieldLength(FieldSpec *field);
int	    FieldListLength(FieldList list);
FieldSpec   *FindField(FieldList list, char *name);
int	    AddField(FieldList *list, FieldSpec *field);
int	    AddSubfield(FieldSpec *field, FieldSpec *subfield);
FieldSpec   **FieldOrder(FieldList list);
FieldSpec   **TypeOrder(FieldList list);
int	    GetFieldOrdering(FieldList list, int *order);
int	    SetFieldOrdering(FieldList *list, int order);

/*
 * General I/O.
 */

int	    ReadPreamble(char **version, char **arch, long *pre_size,
			 long *hdr_size, long *rec_size, FILE *file);
int	    ReadFieldList(FieldList *list, int arch, FILE *file);
FieldList   ReadHeader(char **version, int *arch, long *pre_size,
		       long *hdr_size, long *rec_size, FILE *file);
FieldList   OpenIn(char *filename, char **version, int *arch,
		   long *pre_size, long *hdr_size, long *rec_size,
		   FILE **file);
int	    WritePreamble(char *arch,
			 long fld_size, long rec_size, FILE *file);
int	    WriteFieldList(FieldList list,
			   int arch, FILE *file, Annot *annotate);
int	    WriteHeader(FieldList list,
			int arch, FILE *file, Annot *annotate);
int	    OpenOut(char *filename, FieldList list,
		    int arch, FILE **file, Annot *annotate);
long	    RecordSize(FieldList list, int arch);
int	    ReadRecord(FieldSpec **fields, int arch, FILE *file);
int	    WriteRecord(FieldSpec **fields,
			int arch, FILE *file, Annot *annotate);
long        ReadSamples(void *data, long nrec, FieldSpec **fields,
                        int arch, FILE *file);
long        WriteSamples(void *data, long nrec, FieldSpec **fields,
                         int arch, FILE *file, Annot *annotate);
void	    *AllocSamples(long nrec, FieldSpec **fields);

/*
 * Native binary I/O.
 */

long	    NativeTypeSize(int type);
int	    ReadNativeFieldList(FieldList *list, FILE *file);
int	    WriteNativeFieldList(FieldList list, FILE *file);
long	    NativeRecordSize(FieldList list);
int	    ReadNativeRecord(FieldSpec **fields, FILE *file);
int	    WriteNativeRecord(FieldSpec **fields, FILE *file);
long        ReadNativeSamples(void *data, long nrec, FieldSpec **fields,
                              FILE *file);
long        WriteNativeSamples(void *data, long nrec, FieldSpec **fields,
                               FILE *file);

/*
 * EDR binary I/O.
 */

long	    EdrTypeSize(int type, int longlong);
int	    ReadEdrFieldList(FieldList *list, FILE *file, int longlong);
int	    WriteEdrFieldList(FieldList list, FILE *file, int longlong);
long	    EdrRecordSize(FieldList list, int longlong);
int	    ReadEdrRecord(FieldSpec **fields, FILE *file, int longlong);
int	    WriteEdrRecord(FieldSpec **fields, FILE *file, int longlong);
long        ReadEdrSamples(void *data, long nrec, FieldSpec **fields,
                           FILE *file, int longlong);
long        WriteEdrSamples(void *data, long nrec, FieldSpec **fields,
                            FILE *file, int longlong);

/*
 * Ascii I/O.
 */

int	    ReadAsciiFieldList(FieldList *list, FILE *file);
int	    WriteAsciiFieldList(FieldList list,
				FILE *file, Annot *annotate);
int	    ReadAsciiRecord(FieldSpec **fields, FILE *file);
int	    WriteAsciiRecord(FieldSpec **fields,
			     FILE *file, Annot *annotate);
long        ReadAsciiSamples(void *data, long nrec, FieldSpec **fields,
                             FILE *file);
long        WriteAsciiSamples(void *data, long nrec, FieldSpec **fields,
                              FILE *file, Annot *annotate);

/*
 * MACROS
 */

extern int	DebugMsgLevel;
extern void	(*DebugMsgFunc)(char *msg);

#ifndef NoDEBUG
#define DebugMsg(Level, Msg) \
((DebugMsgLevel >= (Level) && DebugMsgFunc != NULL) \
 ? (void) (*DebugMsgFunc)(Msg) \
 : (void) 0)
#else
#define DebugMsg(Level, Msg)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _ESIGNAL_H */
