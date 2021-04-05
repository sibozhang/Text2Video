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


/*
 * Example programs for Esignal public external file format.
 * Ascii I/O.
 *
 * Author:  Rod Johnson
 */


#include "esignal.h"
#include <ctype.h>

/*
 * LOCAL CONSTANTS
 */

#define MAX_FLD_DEPTH   200
#define STR_ALLOC_SIZE  100

#define Q(X)        QUOTE(X)
#define QUOTE(X)    #X

#define DBL_W       23
#define DBL_FMT     "%.16g"
#define DBL_WFMT    "%#" Q(DBL_W) ".16g"

#define FLT_W       14
#define FLT_FMT     "%.8g"
#define FLT_WFMT    "%#" Q(FLT_W) ".8g"

#define LNG_W       11
#define LNG_FMT     "%ld"
#define LNG_WFMT    "%" Q(LNG_W) "ld"

#define ULN_W       11
#define ULN_FMT     "%lu"
#define ULN_WFMT    "%" Q(ULN_W) "lu"

#define SHR_W       6
#define SHR_FMT     "%d"
#define SHR_WFMT    "%" Q(SHR_W) "d"

#define USH_W       5
#define USH_FMT     "%u"
#define USH_WFMT    "%" Q(USH_W) "u"

#define SCH_W       4
#define SCH_FMT     "%d"
#define SCH_WFMT    "%" Q(SCH_W) "d"

#define UCH_W       3
#define UCH_FMT     "%u"
#define UCH_WFMT    "%" Q(UCH_W) "u"

#define DBLCX_W     DBL_W
#define DBLCX_FMT   DBL_FMT " " DBL_FMT
#define DBLCX_WFMT  "(" DBL_WFMT ", " DBL_WFMT ")"

#define FLTCX_W     FLT_W
#define FLTCX_FMT   FLT_FMT " " FLT_FMT
#define FLTCX_WFMT  "(" FLT_WFMT ", " FLT_WFMT ")"

#define LNGCX_W     LNG_W
#define LNGCX_FMT   LNG_FMT " " LNG_FMT
#define LNGCX_WFMT  "(" LNG_WFMT ", " LNG_WFMT ")"

#define SHRCX_W     SHR_W
#define SHRCX_FMT   SHR_FMT " " SHR_FMT
#define SHRCX_WFMT  "(" SHR_WFMT ", " SHR_WFMT ")"

#define SCHCX_W     SCH_W
#define SCHCX_FMT   SCH_FMT " " SCH_FMT
#define SCHCX_WFMT  "(" SCH_WFMT ", " SCH_WFMT ")"

#define CHR_W       2

#define WCH_W       6

/*
 *  LOCAL FUNCTION DECLARATIONS
 */

/* Functions for input */

static int  ReadAsciiData(FieldSpec *field, FILE *file);
static int  ReadAsciiArray(Array *array, FILE *file);
static int  ReadAsciiFieldSpec(FieldSpec **field, int depth,
                               char **names, FILE *file);
static int  ReadAsciiFieldStruct(FieldSpec **field,
                                 char *name, FILE *file);
static int  ReadAsciiName(int depth, char **names, FILE *file);
static int  ReadAsciiType(short *type, FILE *file);
static int  ReadAsciiDims(short *rank, long**dim, FILE *file);
static int  ReadAsciiMisc(int rank, char **units,
                          double *scale, double *offset,
                          char ***axis_names, FILE *file);
static int  ReadAsciiString(char **string, FILE *file);
static int  ReadAsciiDouble(double *x, FILE *file);
static int  ReadAsciiAxisNames(char ***axis_names,
                               int rank, FILE *file);
static int  ReadAsciiOccurrence(short *occurrence, FILE *file);
static int  AsciiRead(void *data, int type, long length, FILE *file);
static int  ReadAsciiEscape(unsigned long *val, FILE *file);
static int  ReadAsciiNewline(FILE *file);
static int  SkipSpace(int ch, FILE *file);
static int  SkipComment(FILE *file);
static int  GetNonSpace(FILE *file);
static int  ReadSpace(FILE *file);
static int  ReadOptSpace(FILE *file);
static int  AddChar(int ch, char **str, long *alloc_len, long *len);

/* Functions for output */

static int  WriteAsciiData(FieldSpec *field,
                           FILE *file, Annot *annotate);
static int  AsciiWriteSub(void *data, int type, long length,
                          FILE *file, int rank, long *dim, Annot *annotate);
static int  AsciiWrite(void *data, int type,
                       long length, FILE *file, Annot *annotate);
static int  WriteAsciiArray(Array *array, FILE *file, Annot *annotate);
static int  WriteAsciiFieldSpec(FieldSpec *field,
                                int depth, FILE *file, Annot *annotate);
static int  WriteAsciiString(char *data, FILE *file);
static int  AsciiWriteChar(char *data, long length, FILE *file);
static int  AsciiWriteWchar(Wchar *data, long length, FILE *file);
static long ApproxWidth(int type);

/*
 * PUBLIC FUNCTION DEFINITIONS
 * - ReadAsciiFieldList
 * - ReadAsciiRecord
 * - ReadAsciiSamples
 * - WriteAsciiFieldList
 * - WriteAsciiRecord
 * - WriteAsciiSamples
 */

/*
 * Read field list in Ascii format from file.  Return TRUE on success,
 * FALSE on failure.
 */

int
ReadAsciiFieldList(FieldList    *listp,
                   FILE         *file)
{
   static char  *names[MAX_FLD_DEPTH]; /* field name components */
   FieldList    list;
   int          name_len;       /* number of name components */
   FieldSpec    *spec;

   if (file == NULL)
      {
         DebugMsg(1, "ReadAsciiFieldList: NULL file");
         return FALSE;
      }

   list = NULL;

   name_len = ReadAsciiName(0, names, file);

   while (name_len == 1)
      {
         name_len = ReadAsciiFieldSpec(&spec, 1, names, file);

         if (spec == NULL || !AddField(&list, spec))
            {
               if (list != NULL)
                  FreeFieldList(list);
               DebugMsg(2, ((spec == NULL)
                            ? "ReadAsciiFieldList: couldn't read field spec."
                            : ("ReadAsciiFieldList: "
                               "couldn't add field spec to list.")));
               return FALSE;
            }
      }

   if (listp != NULL)
      *listp = list;

   if (name_len != 0)
      {
         DebugMsg(1, "ReadAsciiFieldList: bad name length.");
         return FALSE;
      }

   return TRUE;
}


/*
 * Read one record in Ascii format from file into the "data" members of
 * the field specs on a NULL-terminated linear array of REQUIRED and
 * OPTIONAL fields, like those produced by TypeOrder and FieldOrder.
 * Return TRUE on success, FALSE on failure.
 */

int
ReadAsciiRecord(FieldSpec **fields,
                FILE      *file)
{
   int     ch;
   long    i;


   if (file == NULL || fields == NULL)
      {
         DebugMsg(1, "ReadAsciiRecord: NULL argument.");
         return FALSE;
      }

   for (i = 0 ; fields[i] != NULL; i++)
      {
         if (fields[i]->occurrence == OPTIONAL)
            {
               if (!ReadOptSpace(file))
                  {
                     DebugMsg(2, "ReadAsciiRecord: EOF encountered.");
                     return FALSE;
                  }

               ch = getc(file);

               if (ch == '*')
                  {
                     fields[i]->present = FALSE;
                  }
               else
                  {
                     fields[i]->present = TRUE;
                     ungetc(ch, file);

                     if (!ReadAsciiData(fields[i], file))
                        {
                           DebugMsg(2, ("ReadAsciiRecord: "
                                        "couldn't read data for OPTIONAL field."));
                           return FALSE;
                        }
                  }
            }
         else           /* REQUIRED field */
            {
               if (!ReadAsciiData(fields[i], file))
                  {
                     DebugMsg(1, ("ReadAsciiRecord: "
                                  "couldn't read data for REQUIRED field."));
                     return FALSE;
                  }
            }

         if (!ReadSpace(file))
            {
               DebugMsg(1, ("ReadAsciiRecord: "
                            "couldn't skip space at end of field."));
               return FALSE;
            }
      }

   return TRUE;
}


/*
 * Read nrec one-field records in Ascii format from file into the array
 * indicated by data.  The field is specified by fields,
 * a NULL-terminated linear array like those produced by TypeOrder
 * and FieldOrder and containing exactly one REQUIRED field.
 * Return the number of complete records read.
 */

long
ReadAsciiSamples(void       *data,
                 long       nrec,
                 FieldSpec  **fields,
                 FILE       *file)
{
   int          type;
   long length;
   FieldSpec    *field;

   if (data == NULL)
      {
         DebugMsg(1, "ReadAsciiSamples: NULL data pointer.");
         return 0;
      }

   if (nrec < 0)
      {
         DebugMsg(1, ("ReadAsciiSamples: "
                      "negative number or records specified."));
         return 0;
      }

   if (fields == NULL || (field = fields[0]) == NULL || fields[1] != NULL
       || field->occurrence != REQUIRED)
      {
         DebugMsg(1, "ReadAsciiSamples: bad \"fields\" array.");
         return 0;
      }

   if (file == NULL)
      {
         DebugMsg(1, "ReadAsciiSamples: NULL file pointer.");
         return 0;
      }

   type = field->type;

   if (type == NO_TYPE)
      return 0;

   length = FieldLength(field);

   if (nrec*length == 0)
      return nrec;

   return AsciiRead(data, type, nrec*length, file) / length;
}


/*
 * Write field list to file in Ascii format.
 * If annotate != NULL, add annotations in "[]" for readability.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteAsciiFieldList(FieldList list,
                    FILE      *file,
                    Annot     *annotate)
{
   int     i;                   /* loop index */

   if (file == NULL || list == NULL)
      {
         DebugMsg(1, "WriteAsciiFieldList: NULL argument.");
         return FALSE;
      }

   for (i = 0; list[i] != NULL; i++)
      if (!WriteAsciiFieldSpec(list[i], 1, file, annotate))
         {
            DebugMsg(1, "WriteAsciiFieldList: Failed writing field spec.");
            return FALSE;
         }

   return TRUE;         /* Success. */
}


/*
 * Write to file one record in Ascii format, consisting of the contents
 * of the "data" members of the field specs on a NULL-terminated linear
 * array of REQUIRED and OPTIONAL fields, like those produced by
 * TypeOrder and FieldOrder.
 * If annotate != NULL, add annotations in "[]" for readability.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteAsciiRecord(FieldSpec **fields,    /* linear array of fields */
                 FILE      *file,       /* output file */
                 Annot     *annotate)   /* supply annotations? */
{
   long i;
   FieldSpec    *field;


   /* Check arguments. */

   if (file == NULL || fields == NULL)
      {
         DebugMsg(1, "WriteAsciiRecord: NULL argument.");
         return FALSE;
      }

   /* Write record number if annotating. */

   if (annotate != NULL)
      {
         fprintf(file, "[Record %ld]\n", annotate->recnum);
         annotate->recnum++;
         annotate->indent = 4;
      }

   /* Traverse NULL-terminated list of fields. */

   for (i = 0; (field = fields[i]) != NULL; i++)
      {
         if (annotate != NULL)  /* Write field name if annotating. */
            {
               fprintf(file, "  [%s]", field->fullname);
            }

         if (field->occurrence == OPTIONAL && !field->present)
            fprintf(file, (annotate != NULL) ? " *\n" : "*\n");
         else
            {
               if (annotate != NULL && field->rank == 0)
                  fprintf(file, "  ");

               if (WriteAsciiData(field, file, annotate))
                  fprintf(file, "\n");
               else
                  {
                     DebugMsg(1, "WriteAsciiRecord: Failed writing field.");
                     return FALSE;
                  }
            }
      }

   return TRUE;         /* Success. */
}


/*
 * Write nrec one-field records in Ascii format to file from the
 * array indicated by data.  The field is specified by fields, a
 * NULL-terminated linear array like those produced by TypeOrder
 * and FieldOrder and containing exactly one REQUIRED field.
 * Return the number of complete records written.
 */

long
WriteAsciiSamples(void      *data,
                  long      nrec,
                  FieldSpec **fields,
                  FILE      *file,
                  Annot     *annotate)
{
   int          type;
   long length;
   long rtn = 0L;
   FieldSpec    *field;
   int          i, rank;
   long j, step, stride, recnum;

   if (data == NULL)
      {
         DebugMsg(1, "WriteAsciiSamples: NULL data pointer.");
         return 0;
      }

   if (nrec < 0)
      {
         DebugMsg(1, ("WriteAsciiSamples: "
                      "negative number of records specified."));
         return 0;
      }

   if (fields == NULL || (field = fields[0]) == NULL || fields[1] != NULL
       || field->occurrence != REQUIRED)
      {
         DebugMsg(1, "WriteAsciiSamples: bad \"fields\" array.");
         return 0;
      }

   if (file == NULL)
      {
         DebugMsg(1, "WriteAsciiSamples: NULL file pointer.");
         return 0;
      }

   type = field->type;

   if (type == NO_TYPE)
      return 0;

   length = FieldLength(field);

   if (nrec*length == 0)
      return nrec;

   if (annotate == NULL)
      {
         rtn = AsciiWrite(data, type, nrec*length, file, annotate) / length;
      }
   else
      {
         rank = field->rank;
         recnum = annotate->recnum;
         annotate->recnum += nrec;

         if (length == 1)
            {
               if (type == ARRAY)
                  step = 1;
               else
                  {
                     step = (annotate->width - annotate->indent - 3*rank + 1)
                        / ApproxWidth(type);
                     if (step < 1)
                        step = 1;
                  }

               stride = step*InternTypeSize(type);

               for (j = 0; nrec > 0; j += step, nrec -= step)
                  {
                     if (j > 0)
                        {
                           fprintf(file, "\n");
                           data = (char *) data + stride;
                        }
                     annotate->indent =
                        fprintf(file, "[%ld]", j + recnum)
                        + 3*rank + 2;
                     for (i = 0; i < rank; i++)
                        fprintf(file, "[0]");
                     fprintf(file, "  ");
                     AsciiWrite(data, type,
                                (nrec < step) ? nrec : step, file, annotate);
                  }
            }
         else
            {
               stride = length*InternTypeSize(type);
               for (j = 0; j < nrec; j++)
                  {
                     if (j > 0)
                        fprintf(file, "\n");

                     annotate->indent =
                        fprintf(file, "[%ld]", j + recnum);

                     AsciiWriteSub(j*stride + (char *) data, type, length,
                                   file, rank, field->dim, annotate);
                  }
            }

         annotate->indent = 0;
      }

   fprintf(file, "\n");

   return rtn;
}


/*
 * LOCAL FUNCTION DEFINITIONS
 */

/*
 * *** FUNCTIONS FOR INPUT
 */

/*
 * Read Ascii data from file into the "data" member of field.
 * The number of elements and the data type are obtained from field.
 * If field->data is NULL, allocate space before reading the data.
 * Return TRUE on success, FALSE on failure.
 */

static int
ReadAsciiData(FieldSpec *field, FILE *file)
{
   size_t  size;
   long    length;

   if (file == NULL || field == NULL)
      {
         DebugMsg(1, "ReadAsciiData: NULL arguments.");
         return FALSE;
      }

   if (field->type == NO_TYPE)
      {
         DebugMsg(1, "ReadAsciiData: type NO_TYPE.");
         return FALSE;
      }

   size = InternTypeSize(field->type);
   length = FieldLength(field);

   if (length == 0)
      return TRUE;

   if (field->data == NULL)
      {
         field->data = malloc(length * size);
         if (field->data == NULL)
            {
               DebugMsg(1, "ReadAsciiData: allocation failure.");
               return FALSE;
            }
      }

   return (AsciiRead(field->data, field->type, length, file) == length);
}


/*
 * Read from file the Ascii representation of an item of type ARRAY,
 * not including the surrounding parentheses.  Store the type, rank,
 * dimensions, and data in the members of the array structure,
 * allocating the necessary space for dimensions and data.
 * Return TRUE for success, FALSE for failure.
 */

static int
ReadAsciiArray(Array    *array,
               FILE     *file)
{
   short   type;
   short   rank;
   long    *dim;
   void    *data;
   size_t  size;
   long    length;
   int     ch;

   if (!ReadAsciiType(&type, file))
      {
         DebugMsg(1, "ReadAsciiArray: couldn't read type.");
         return FALSE;
      }

   if (type == NO_TYPE)
      {
         DebugMsg(1, "ReadAsciiArray: type NO_TYPE.");
         return FALSE;
      }

   if (!ReadAsciiDims(&rank, &dim, file))
      {
         DebugMsg(1, "ReadAsciiArray: couldn't read dimensions.");
         return FALSE;
      }

   ch = GetNonSpace(file);

   if (ch != ':')
      {
         if (dim != NULL)
            free(dim);
         DebugMsg(1, "ReadAsciiArray: no colon after dimensions.");
         return FALSE;
      }

   length = LongProd(rank, dim);
   size = InternTypeSize(type);

   if (length == 0)
      data = NULL;
   else
      {
         data = malloc(length*size);
         if (data == NULL
             || AsciiRead(data, type, length, file) != length)
            {
               if (dim != NULL)
                  free(dim);
               DebugMsg(1, "ReadAsciiArray: couldn't read data.");
               return FALSE;
            }
      }

   array->type = type;
   array->rank = rank;
   array->dim = dim;
   array->data = data;

   return TRUE;
}


/*
 * Read from file an Ascii field specification, together with any
 * folowing specifications for subfields.  Create a new FieldSpec
 * structure containing the result.  Assign to the "subfields" member
 * of the new structure a field list comprising the subfield
 * specifications.  Store a pointer to the new structure in the
 * variable whose address is "field" (unless "field" is NULL).
 * It is assumed that before this function is called, the full name
 * of the field (including the following colon) has been read and
 * parsed into its components, and that the components are stored
 * in the string array "names"; the number of components is given
 * by "depth".  This function uses ReadAsciiFieldStruct to create
 * the new structure and fill in the members other than "subfields".
 * It then uses ReadAsciiName and recursive calls of
 * ReadAsciiFieldSpec to read names and subfield specifications.
 * It terminates after the reading of a name that is not the name
 * of a subfield, or when no more names are found.  The array
 * "names" is updated with the components of the last name read,
 * if any, and the return value of ReadAsciiFieldSpec is the number
 * of components.  A return value of 0 indicates that no more field
 * names were found.  A value of -1 is an error return.
 */

static int
ReadAsciiFieldSpec(FieldSpec    **field,
                   int          depth,
                   char         **names,
                   FILE         *file)
{
   FieldSpec    *spec;
   FieldSpec    *subfield;
   int          name_len;


   if (!ReadAsciiFieldStruct(&spec, StrDup(names[depth-1]), file))
      {
         *field = NULL;
         return -1;             /* Error return. */
      }

   name_len = ReadAsciiName(depth, names, file);

   while (name_len == depth + 1)
      {
         name_len = ReadAsciiFieldSpec(&subfield,
                                       depth + 1, names, file);

         if (subfield == NULL || !AddSubfield(spec, subfield))
            {
               FreeFieldList(spec->subfields);
               FreeFieldSpec(spec);
               *field = NULL;
               return -1;               /* Error return. */
            }
      }

   if (field != NULL)
      *field = spec;

   return name_len;
}


/*
 * Read from file the part of an Ascii field specification that follows
 * the initial <full name> ":".  The last component of the name is
 * supplied via an argument.  If successful, allocate a new FieldSpec
 * structure, fill in the name, type, rank, dimensions, miscellaneous
 * attributes, and occurrence code.  If the field data is in the header
 * (e.g. GLOBAL fields) also fill in the data.  Store a pointer to the
 * new structure in *field.  (The subfields are not read by this
 * function; that is the task of ReadAsciiFieldSpec.)
 * Return TRUE for success, FALSE for failure.
 */

static int
ReadAsciiFieldStruct(FieldSpec  **field,
                     char       *name,
                     FILE       *file)
{
   short        type;
   short        rank;
   long *dim;
   FieldSpec    *spec = NULL;
   int          i;

   if (ReadAsciiType(&type, file)
       && ReadAsciiDims(&rank, &dim, file)
       && (spec = NewFieldSpec(type, rank)) != NULL
       && ReadAsciiMisc(rank, &spec->units, &spec->scale,
                        &spec->offset, &spec->axis_names, file)
       && ReadAsciiOccurrence(&spec->occurrence, file))
      {
         spec->name = name;

         for (i = 0; i < rank; i++)
            spec->dim[i] = dim[i];

         if (dim != NULL)
            free(dim);

         if ((type == NO_TYPE
              || spec->occurrence == REQUIRED || spec->occurrence == OPTIONAL 
              || ReadAsciiData(spec, file))
             && ReadAsciiNewline(file))
            {
               *field = spec;
               return TRUE;     /* Success. */
            }
      }

   if (spec != NULL)
      FreeFieldSpec(spec);

   return FALSE;                /* Error return. */
}


/*
 * Attempt to read from "file" a field name consisting of 1 or more
 * components separated by dots ("."); each component has the syntax of
 * a C identifier.  Parse the name into components.  Upon normal
 * completion successive elements of the string array "names" contain
 * the component identifiers, and the return value is the number of
 * components.
 *
 * This function is used by ReadAsciiFieldList and ReadAsciiFieldSpec
 * to read and check field names in the course of reading a field list.
 *
 * A return value of 0 indicates that no name was found, i.e. the
 * first character read (after skipping whitespace and bracketed
 * comments) was not a letter or underscore.  Normally this implies
 * the end of a field list, and the non-letter non-underscore character
 * (if not EOF) is the first character of a record.  The character is
 * returned to the input stream by means of "ungetc" and is the first
 * to be read by a subsequent attempt to read a record.
 *
 * Upon entry to ReadAsciiName, a positive value of "depth" indicates
 * that the first "depth" elements of "names" are the components of
 * a previously read field name.  In this case, ReadAsciiName checks
 * that the name being read is one that can legally follow the
 * previously read name. If not, it returns an error indication of
 * -1 instead of the normal nonnegative value.  The criterion for
 * legality is that all components of the new name, except the last,
 * must match the corresponding components of the previously read name,
 * but the last component, if any, must not match.  Examples:
 *  1.  "names" on entry    { "aaa", "bbb", "ccc", ...}
 *      "depth"             3
 *      read from file      "aaa.bbb.ccc.ddd"
 *      OK: we've just read a spec for a field "aaa.bbb.ccc", and
 *        we're about to read a spec for a subfield.
 *      "names" on exit     { "aaa", "bbb", "ccc", "ddd", ...}
 *      return              4
 *  2.  "names" on entru    { "aaa", "bbb", "ccc", "ddd", ...}
 *      "depth"             4
 *      read from file      "aaa.xxx"
 *      OK: we've just terminated the subfield lists for "aaa.bbb.ccc"
 *        and "aaa.bbb", and we're about to get another subfield of "aaa".
 *      "names" on exit     { "aaa", "xxx", ...}
 *      return              2
 *  3.  "names" on entry    { "aaa", "bbb", "ccc", ...}
 *      "depth"             3
 *      read from file      "aaa.bbb"
 *      Error: we must have seen "aaa.bbb" before, as an ancestor of
 *        "aaa.bbb.ccc"; we shouldn't have another spec for "aaa.bbb".
 *      return              -1
 *  4.  "names" on entry    { "aaa", "bbb", ...}
 *      "depth"             2
 *      read from file      "aaa.bbb.ccc.ddd"
 *      Error: we're missing a spec for "aaa.bbb.ccc".
 *      return              -1
 */

static int
ReadAsciiName(int depth, char **names, FILE *file)
{
   int     n_id;                /* number of component identifiers */
   char    *id = NULL;          /* component to match */
   int     n_ch = 0;            /* number of characters */
   int     ch;                  /* input character */
   long    alloc_size;


   ch = GetNonSpace(file);

   for (n_id = 0; n_id < depth; n_id++)
      {
         id = names[n_id];

         for (n_ch = 0; id[n_ch] != '\0' && ch == id[n_ch]; n_ch++)
            ch = getc(file);

         if (id[n_ch] != '\0')
            break;
         else if (isalnum(ch) || ch == '_')
            break;
         else if (ch != DOT)
            return -1;

         ch = getc(file);
      }

   if (n_id == depth)
      {
         if (depth >= MAX_FLD_DEPTH)
            return -1;

         id = names[n_id];
         n_ch = 0;

         alloc_size = STR_ALLOC_SIZE;
         id = (char *)
            ((id == NULL)
             ? malloc(alloc_size)
             : realloc(id, alloc_size));
      }
   else
      {
         alloc_size = n_ch + STR_ALLOC_SIZE;
         id = (char *) realloc(id, alloc_size);
      }

   if (id == NULL)
      return -1;

   names[n_id] = id;

   if (n_ch == 0)
      {
         if (isalpha(ch) || ch == '_')
            {
               id[n_ch++] = ch;
               ch = getc(file);
            }
         else if (n_id == 0)
            {
               if (ch != EOF)
                  ungetc(ch, file);
               return 0;
            }
         else return -1;
      }

   while (isalnum(ch) || ch == '_')
      {
         if (n_ch >= alloc_size)
            {
               alloc_size += STR_ALLOC_SIZE;
               id = (char *) realloc(id, alloc_size);

               if (id == NULL)
                  return -1;

               names[n_id] = id;
            }

         id[n_ch++] = ch;
         ch = getc(file);
      }

   alloc_size = n_ch + 1;
   id = (char *) realloc(id, alloc_size);

   if (id == NULL)
      return -1;

   names[n_id] = id;

   id[n_ch] = '\0';

   ch = SkipSpace(ch, file);

   if (ch != ':')
      return -1;

   return n_id + 1;
}


/*
 * Read from file a string of upper-case letters and underscores.
 * If this is a valid elib type name (e.g. "DOUBLE_COMPLEX")
 * assign the corresponding numeric type code to *type and return
 * TRUE.  In case of error return FALSE.
 */

static int
ReadAsciiType(short *type, FILE *file)
{
   char    name[MAX_TYPE_LEN];
   int     ch;
   int     i;
   int     code;

   ch = GetNonSpace(file);
   for (i = 0; i < MAX_TYPE_LEN && (isupper(ch) || ch == '_'); i++)
      {
         name[i] = ch;
         ch = getc(file);
      }

   if (i == MAX_TYPE_LEN)
      return FALSE;

   name[i] = '\0';

   if (ch != EOF)
      ungetc(ch, file);

   code = TypeCode(name);

   if (code == 0)
      return FALSE;

   *type = code;

   return TRUE;
}

/*
 * Read a possibly empty sequence of unsigned longs (typically
 * array dimensions) from file.  If it's nonempty, allocate an
 * array to hold the values.  Assign to *rank the number of values
 * read.  Assign to *dim a pointer to the allocated array, if any,
 * or NULL if the sequence was empty.  (But if rank or dim is NULL,
 * omit the corresponding assignment.)  Return TRUE for success,
 * FALSE for error.
 */

static int
ReadAsciiDims(short *rank, long **dim, FILE *file)
{
   long    d;
   long    *dd;
   int     r;

   /*! Check for overflow.  fscanf doesn't, and Sun's strtol doesn't. */

   if (!ReadSpace(file) || fscanf(file, "%lu", &d) != 1)
      {
         r = 0;
         dd = NULL;
      }
   else
      {
         dd = (long *) malloc(sizeof(*dd));
         if (dd == NULL)
            return FALSE;

         dd[0] = d;

         for (r = 1;
              ReadSpace(file) && fscanf(file, "%lu", &d) == 1;
              r++)
            {
               dd = (long *) realloc(dd, (r + 1) * sizeof(*dd));
               if (dd == NULL)
                  return FALSE;

               dd[r] = d;
            }
      }

   if (rank != NULL)
      *rank = r;

   if (dim != NULL)
      *dim = dd;

   return TRUE;
}


/*
 * Read Ascii representations of the miscellaneous attributes
 * (units, scale, offset, axis_names) from file, and output the
 * values found by assignment to the variables *units, *scale, *offset,
 * and *axis_names (if defined).  If no explicit representation
 * for an attribute is found, omit the corresponding assignment.
 * (The caller is assumed to have assigned default values to the
 * variables before calling ReadAsciiMisc).  If any of the arguments
 * units, scale, offset, and axis_names is NULL, omit the corresponding
 * assignment.  Return TRUE upon success and FALSE in case of error.
 */

static int
ReadAsciiMisc(int rank, char **units, double *scale,
              double *offset, char ***axis_names, FILE *file)
{
   /*
    * Length of longest attribute name ("axis_names"), including
    * terminal null character.
    */
#define MAX_ATTR        11

   char    attr[MAX_ATTR];
   int     ch;
   int     i;
   char    *un;
   double  sc;
   double  offsetr;
   char    **ax;
   char    got_units = FALSE;
   char    got_scale = FALSE;
   char    got_offset = FALSE;
   char    got_axis_names = FALSE;

   ch = GetNonSpace(file);

   if (ch != '{')
      {
         ungetc(ch, file);
         return TRUE;
      }

   do {
      ch = GetNonSpace(file);
      for (i = 0; i < MAX_ATTR && (islower(ch) || ch == '_'); i++)
         {
            attr[i] = ch;
            ch = getc(file);
         }

      if (i == MAX_ATTR)
         return FALSE;

      attr[i] = '\0';

      ch = SkipSpace(ch, file);
      if (ch != ':')
         return FALSE;

      if (strcmp(attr, "units") == 0)
         {
            if (!got_units++
                && ReadAsciiString(&un, file))
               {
                  if (units != NULL)
                     *units = un;
               }
            else
               return FALSE;
         }
      else if (strcmp(attr, "scale") == 0)
         {
            if (!got_scale++
                && ReadAsciiDouble(&sc, file))
               {
                  if (scale != NULL)
                     *scale = sc;
               }
            else
               return FALSE;
         }
      else if (strcmp(attr, "offset") == 0)
         {
            if (!got_offset++
                && ReadAsciiDouble(&offsetr, file))
               {
                  if (offset != NULL)
                     *offset = offsetr;
               }
            else
               return FALSE;
         }
      else if (strcmp(attr, "axis_names") == 0)
         {
            if (!got_axis_names++
                && ReadAsciiAxisNames(&ax, rank, file))
               {
                  if (axis_names != NULL)
                     *axis_names = ax;
               }
            else
               return FALSE;
         }
      else
         return FALSE;

      ch = GetNonSpace(file);

   } while (ch == ';');

   if (ch != '}')
      return FALSE;

   return TRUE;

#undef MAX_ATTR
}


/*
 * Read from file a string constant consisting of Ascii printing characters
 * and including surrounding double quote characters("").  Interpret escape
 * sequences \\, \?, \', and \"  by removing the backslash.  Allocate space
 * to hold the string value, including terminating null character.  Store
 * a pointer to the string in the variable whose address is given by
 * string (unless string is NULL).  Return TRUE on success, FALSE on
 * failure.
 */

static int
ReadAsciiString(char **string, FILE *file)
{
   int     ch;
   char    *str = NULL;
   long    alloc_len = 0;
   long    len = 0;

   ch = GetNonSpace(file);
   if (ch != '"')
      return FALSE;

   for (ch = getc(file); isprint(ch) && ch != '"'; ch = getc(file))
      {
         if (ch == '\\')
            {
               ch = getc(file);

               switch (ch)
                  {
                  case '\\':    /* fall through */
                  case '?':     /* fall through */
                  case '\'':    /* fall through */
                  case '"':
                     if (!AddChar(ch, &str, &alloc_len, &len))
                        return FALSE;
                     break;
                  default:
                     free(str);
                     return FALSE;
                  }
            }
         else if (!AddChar(ch, &str, &alloc_len, &len))
            return FALSE;

      }

   if (ch != '"' || !AddChar('\0', &str, &alloc_len, &len))
      {
         free(str);
         return FALSE;
      }

   if (string != NULL)
      *string = str;

   return TRUE;
}


/*
 * Read a floating-point constant (with optional leading whitespace)
 * from file and assign the value to the variable whose address
 * is given by x.
 */

static int
ReadAsciiDouble(double *x, FILE *file)
{
   return (ReadOptSpace(file) && fscanf(file, "%lf", x) == 1);
}


/*
 * Read from file a comma-separated list with at most "rank" members,
 * each of which is either a C identifier or empty.  (This construct
 * occurs in the Ascii form of a specification for the miscellaneous
 * field attribute "axis_names".)  Allocate a pointer array of length
 * rank (or NULL if rank is 0) and assign to its elements:
 * - a string pointer for each identifier on the list,
 * - a NULL for each empty member of the list
 * - a NULL for each array position beyond the end of the list.
 * Store a pointer to the array (or NULL) in the variable whose
 * address is given by "axis_names" (unless that argument is NULL).
 * Return TRUE for success, FALSE for failure.
 */

static int
ReadAsciiAxisNames(char ***axis_names, int rank, FILE *file)
{
   char    **ax;
   int     ch;
   char    *str;
   long    len;
   long    alloc_len;
   int     i;

   if (rank == 0)
      {
         ax = NULL;

         ch = GetNonSpace(file);

         if (isalpha(ch) || ch == '_' || ch == ',')
            return FALSE;
      }
   else
      {
         ax = (char **) malloc(rank * sizeof(*ax));
         if (ax == NULL)
            return FALSE;

         i = 0;

         do {
            ch = GetNonSpace(file);

            if (isalpha(ch) || ch == '_')
               {
                  str = NULL;
                  len = 0;
                  alloc_len = 0;

                  if (!AddChar(ch, &str, &alloc_len, &len))
                     {
                        FreeAxisNames(ax, rank);
                        return FALSE;
                     }

                  for (ch = getc(file);
                       isalnum(ch) || ch == '_';
                       ch = getc(file))
                     {
                        if (!AddChar(ch, &str, &alloc_len, &len))
                           {
                              FreeAxisNames(ax, rank);
                              return FALSE;
                           }
                     }

                  if (!AddChar('\0', &str, &alloc_len, &len))
                     {
                        FreeAxisNames(ax, rank);
                        return FALSE;
                     }

                  ax[i] = str;

                  ch = SkipSpace(ch, file);
               }
            else
               ax[i] = NULL;

            i++;

         } while (ch == ',' && i < rank);

         if (ch == ',')
            {
               FreeAxisNames(ax, rank);
               return FALSE;
            }
         else
            for ( ; i < rank; i++)
               ax[i] = NULL;
      }

   ungetc(ch, file);

   if (axis_names != NULL)
      *axis_names = ax;

   return TRUE;
}


/*
 * Read from file an "occurrence" construct (which occurs in Ascii field
 * specifications: <g>, <r>, <o>, <v), or <i>.  Assign the corresponding code
 * (GLOBAL, REQUIRED, OPTIONAL, VIRTUAL, or INCLUDED) to the variable
 * whose address is occurrence (unless that argument is NULL).
 * Return TRUE for success, FALSE for failure.
 */

static int
ReadAsciiOccurrence(short *occurrence, FILE *file)
{
   int     ch;
   int     occ;

   ch = GetNonSpace(file);

   if (ch != '<')
      return FALSE;

   switch (getc(file))
      {
      case 'g':
         occ = GLOBAL;
         break;
      case 'r':
         occ = REQUIRED;
         break;
      case 'o':
         occ = OPTIONAL;
         break;
      case 'v':
         occ = VIRTUAL;
         break;
      case 'i':
         occ = INCLUDED;
         break;
      default:
         return FALSE;
      }

   if (getc(file) != '>')
      return FALSE;

   if (occurrence != NULL)
      *occurrence = occ;

   return TRUE;
}


/*
 * Read from "file" a sequence of length "length" of values of data
 * type "type" (in Ascii representation); store the results starting
 * at the location indicated by "datap".  Return the number of values
 * successfully read, which will be less than "length" in case of
 * error.
 */

static int
AsciiRead(void  *datap,
          int   type,
          long  length,
          FILE  *file)
{
   long    j;
   int     ch;

   if (length == 0)
      return TRUE;

   if (!ReadOptSpace(file))
      return 0;

   j = 0;

   switch (type)
      {
      case ARRAY:
         {
            Array   *data = (Array *) datap;

            do {
               ch = getc(file);

               if (ch != '(')
                  return j;

               if (!ReadOptSpace(file)
                   || !ReadAsciiArray(&data[j], file))
                  {
                     return j;
                  }

               ch = GetNonSpace(file);
               if (ch != ')')
                  return j;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case DOUBLE:
         {
            double  *data = (double *) datap;

            do {
               if (fscanf(file, "%lf", &data[j]) != 1)
                  return j;
            } while (++j < length && ReadSpace(file));
         }
         break;
      case FLOAT:
         {
            float   *data = (float *) datap;

            do {
               if (fscanf(file, "%f", &data[j]) != 1)
                  return j;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case LONG:
         {
            long    *data = (long *) datap;

            do {
               if (fscanf(file, "%ld", &data[j]) != 1)
                  return j;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case ULONG:
         {
            unsigned long   *data = (unsigned long *) datap;

            do {
               if (fscanf(file, "%lu", &data[j]) != 1)
                  return j;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case SHORT:
         {
            short   *data = (short *) datap;

            do {
               if (fscanf(file, "%hd", &data[j]) != 1)
                  return j;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case USHORT:
         {
            unsigned short  *data = (unsigned short *) datap;

            do {
               if (fscanf(file, "%hu", &data[j]) != 1)
                  return j;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case SCHAR:
         {
            Schar   *data = (Schar *) datap;
            int     n;

            do {
               if (fscanf(file, "%d", &n) != 1)
                  return j;
               data[j] = n;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case UCHAR:
         {
            Uchar       *data = (Uchar *) datap;
            unsigned    u;

            do {
               if (fscanf(file, "%u", &u) != 1)
                  return j;
               data[j] = u;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case BOOL:
         {
            Bool    *data = (Bool *) datap;

            do {
               switch (getc(file))
                  {
                  case '0':
                     data[j] = 0;
                     break;
                  case '1':
                     data[j] = 1;
                     break;
                  default:
                     return j;
                  }
            } while (++j < length && ReadOptSpace(file));
         }
         break;
      case DOUBLE_COMPLEX:
         {
            DoubleComplex   *data = (DoubleComplex *) datap;
            double          re, im;

            do {
               if (fscanf(file, "%lf", &re) == 1)
                  {
                     if (!ReadSpace(file)
                         || fscanf(file, "%lf", &im) != 1)
                        {
                           return j;
                        }
                  }
               else
                  {
                     ch = getc(file);
                     if (ch != '(')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%lf", &re) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ',')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%lf", &im) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ')')
                        return j;
                  }

               data[j].real = re;
               data[j].imag = im;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case FLOAT_COMPLEX:
         {
            FloatComplex    *data = (FloatComplex *) datap;
            float           re, im;

            do {
               if (fscanf(file, "%f", &re) == 1)
                  {
                     if (!ReadSpace(file)
                         || fscanf(file, "%f", &im) != 1)
                        {
                           return j;
                        }
                  }
               else
                  {
                     ch = getc(file);
                     if (ch != '(')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%f", &re) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ',')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%f", &im) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ')')
                        return j;
                  }

               data[j].real = re;
               data[j].imag = im;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case LONG_COMPLEX:
         {
            LongComplex     *data = (LongComplex *) datap;
            long            re, im;

            do {
               if (fscanf(file, "%ld", &re) == 1)
                  {
                     if (!ReadSpace(file)
                         || fscanf(file, "%ld", &im) != 1)
                        {
                           return j;
                        }
                  }
               else
                  {
                     ch = getc(file);
                     if (ch != '(')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%ld", &re) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ',')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%ld", &im) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ')')
                        return j;
                  }

               data[j].real = re;
               data[j].imag = im;


            } while (++j < length && ReadSpace(file));
         }
         break;
      case SHORT_COMPLEX:
         {
            ShortComplex    *data = (ShortComplex *) datap;
            short           re, im;

            do {
               if (fscanf(file, "%hd", &re) == 1)
                  {
                     if (!ReadSpace(file)
                         || fscanf(file, "%hd", &im) != 1)
                        {
                           return j;
                        }
                  }
               else
                  {
                     ch = getc(file);
                     if (ch != '(')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%hd", &re) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ',')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%hd", &im) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ')')
                        return j;
                  }

               data[j].real = re;
               data[j].imag = im;


            } while (++j < length && ReadSpace(file));
         }
         break;
      case SCHAR_COMPLEX:
         {
            ScharComplex    *data = (ScharComplex *) datap;
            int             re, im;

            do {
               if (fscanf(file, "%d", &re) == 1)
                  {
                     if (!ReadSpace(file)
                         || fscanf(file, "%d", &im) != 1)
                        {
                           return j;
                        }
                  }
               else
                  {
                     ch = getc(file);
                     if (ch != '(')
                        return j;

                     if (!ReadOptSpace(file)
                         || fscanf(file, "%d", &re) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ',')
                        return j;


                     if (!ReadOptSpace(file)
                         || fscanf(file, "%d", &im) != 1)
                        {
                           return j;
                        }

                     ch = GetNonSpace(file);
                     if (ch != ')')
                        return j;
                  }

               data[j].real = re;
               data[j].imag = im;

            } while (++j < length && ReadSpace(file));
         }
         break;
      case CHAR:
         {
            char            *data = (char *) datap;
            unsigned long   val;

            ch = getc(file);
            if (ch == '"')
               {
                  for (j = 0; j < length; j++)
                     {
                        ch = getc(file);

                        if (ch == '"')
                           {
                              if (j == 0)
                                 return j;
                              ch = GetNonSpace(file);
                              if (ch != '"')
                                 return j;
                              ch = getc(file);
                           }

                        if (ch == '\\')
                           {
                              if (ReadAsciiEscape(&val, file)
                                  && val <= UCHAR_MAX)
                                 {
                                    data[j] = val;
                                 }
                              else
                                 return j;
                           }
                        else if (ch == '"')
                           return j;
                        else if (isprint(ch))
                           data[j] = ch;
                        else
                           return j;
                     }

                  ch = getc(file);
                  if (ch != '"')
                     return j;
               }
            else if (length > 0)
               return j;
            else if (ch != EOF)
               ungetc(ch, file);
         }
         break;
      case WCHAR:
         {
            Wchar           *data = (Wchar *) datap;
            unsigned long   val;

            ch = getc(file);
            if (ch == '"')
               {
                  for (j = 0; j < length; j++)
                     {
                        ch = getc(file);

                        if (ch == '"')
                           {
                              if (j == 0)
                                 return j;
                              ch = GetNonSpace(file);
                              if (ch != '"')
                                 return j;
                              ch = getc(file);
                           }

                        if (ch == '\\')
                           {
                              if (ReadAsciiEscape(&val, file)
                                  && val <= USHRT_MAX)
                                 data[j] = val;
                              else
                                 return j;
                           }
                        else if (ch == '"')
                           return j;
                        else if (isprint(ch))
                           data[j] = ch;
                        else
                           return j;
                     }

                  ch = getc(file);
                  if (ch != '"')
                     return j;
               }
            else if (length > 0)
               return j;
            else if (ch != EOF)
               ungetc(ch, file);
         }
         break;
      default:
         return 0;
      }

   return j;
}


/*
 * Read from file an escape sequence like those in C character constants
 * and string literals and the Esignal Ascii representations of CHAR
 * and WCHAR data.  The initial backslash (\) is assumed to have already
 * been read; this function reads the rest of the construct.  The backslash
 * can be followed by various single characters, by an octal number with
 * up to three digits, or by an "x" followed by a hexadecimal number with
 * any number of digits.  Assign the corresponding Ascii numeric value
 * to the variable whose address is val (which must not be NULL).
 */

static int
ReadAsciiEscape(unsigned long   *val,
                FILE            *file)
{
   int     ch;

   ch = getc(file);

   switch (ch)
      {
      case '\\':
      case '?':
      case '\'':
      case '"':
         *val = ch;
         break;
      case 'a':
         *val = '\a';
         break;
      case 'b':
         *val = '\b';
         break;
      case 'f':
         *val = '\f';
         break;
      case 'n':
         *val = '\n';
         break;
      case 'r':
         *val = '\r';
         break;
      case 't':
         *val = '\t';
         break;
      case 'v':
         *val = '\v';
         break;
      case 'x':
         {
            const unsigned long chkval = ULONG_MAX >> 4;
            unsigned long       xval;

            ch = getc(file);

            if (!isxdigit(ch))
               return FALSE;

            xval = 0;

            do {
               if (isdigit(ch))
                  ch -= '0';
               else
                  switch (ch)
                     {
                     case 'A':
                     case 'a':
                        ch = 10;
                        break;
                     case 'B':
                     case 'b':
                        ch = 11;
                        break;
                     case 'C':
                     case 'c':
                        ch = 12;
                        break;
                     case 'D':
                     case 'd':
                        ch = 13;
                        break;
                     case 'E':
                     case 'e':
                        ch = 14;
                        break;
                     case 'F':
                     case 'f':
                        ch = 15;
                        break;
                     }

               if (xval > chkval) /* Will overflow. */
                  return FALSE;

               xval = 16*xval + ch;

               ch = getc(file);

            } while (isxdigit(ch));

            if (ch != EOF)
               ungetc(ch, file);

            *val = xval;
         }
         break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
         ungetc(ch, file);
         fscanf(file, "%3lo", val);
         break;
      default:
         return FALSE;
      }

   return TRUE;
}


/*
 * Scan file, skipping whitespace and bracketed comments, searching for
 * a newline character.  If successful, return TRUE, leaving the file
 * positioned just past the newline.  If unsuccessful, return FALSE.
 */

static int
ReadAsciiNewline(FILE *file)
{
   int     ch;

   ch = getc(file);

   for (;;)
      {
         if (isspace(ch) && ch != '\n')
            ch = getc(file);
         else if (ch == '[')
            ch = SkipComment(file);
         else break;
      }


   if (ch != '\n')
      return FALSE;

   return TRUE;
}


/*
 * Scan a sequence of characters, starting with ch and continuing with
 * characters read from file, skipping whitespace and bracketed comments.
 * Return the last character examined (ch or the last character read),
 * which will not be a whitespace character or part of a bracketed
 * comment (but may be EOF).
 */

static int
SkipSpace(int ch, FILE *file)
{
   for (;;)
      {
         if (isspace(ch))
            ch = getc(file);
         else if (ch == '[')
            ch = SkipComment(file);
         else break;
      }

   return ch;
}


/*
 * Skip over a bracketed comment in file.  The opening bracket is
 * assumed to have already been read; the function reads the rest
 * of the construct, then reads and returns the character after the
 * closing bracket (possibly EOF).  The return value is EOF if no
 * closing bracket is found.
 */

static int
SkipComment(FILE *file)
{
   int     ch;

   do {
      ch = getc(file);
   } while ((isgraph(ch) && ch != '[' && ch != ']')
            || isspace(ch));

   if (ch == ']')
      {
         ch = getc(file);
         return ch;
      }
   else
      return EOF;
}


/*
 * Scan file, skipping whitespace and bracketed comments.
 * Return the last character read, which will not be a whitespace
 * character or part of a bracketed comment (but may be EOF).
 */

static int
GetNonSpace(FILE *file)
{
   int     ch;

   ch = getc(file);
   ch = SkipSpace(ch, file);

   return ch;
}


/*
 * Scan file, skipping whitespace and bracketed comments, and leave
 * the file positioned just before the first character that is not
 * whitespace or part of a bracketed comment (or at EOF if no
 * such character is found).  Return TRUE if at least one whitespace
 * character or bracketed comment is found; otherwise return FALSE.
 */

static int
ReadSpace(FILE *file)
{
   int     ch;
   int     okay;

   ch = getc(file);

   if (!isspace(ch) && ch != '[')
      okay = FALSE;
   else
      {
         ch = SkipSpace(ch, file);
         okay = TRUE;
      }

   if (ch != EOF)
      ungetc(ch, file);

   return okay;
}


/*
 * Scan file, skipping whitespace and bracketed comments, and leave
 * the file positioned just before the first character that is not
 * whitespace or part of a bracketed comment, or at EOF if no
 * such character is found.  Return TRUE if such a character is found,
 * FALSE if EOF is reached.
 */

static int
ReadOptSpace(FILE *file)
{
   int     ch;

   ch = GetNonSpace(file);

   if (ch == EOF)
      return FALSE;

   ungetc(ch, file);
   return TRUE;
}


/*
 * Append a character ch to a character array of length *len
 * contained in a malloc'ed storage block with a size of
 * *alloc_len bytes, at the location indicated by the pointer *str.
 * If the block isn't large enough, the function reallocates the block
 * and updates *str and *alloc_len accordingly.  The function also
 * increments *len by 1.  If reallocation is necessary, the function
 * allocates more storage than is required so that a number of subsequent
 * calls of AddChar may be made without the need to reallocate.
 * The arguments str, alloc_len, and len are addresses of variables,
 * rather than value, so that the function can assign to them.
 * For example str is the address of a variable containing a pointer
 * to the beginning of the storage area.  To start things off,
 * initialize *str to NULL, *alloc_len to 0, and *len to 0.
 * The function returns TRUE upon success and FALSE in case of
 * allocation failure.
 */

static int
AddChar(int     ch,
        char    **str,
        long    *alloc_len,
        long    *len)
{
   if (*len >= *alloc_len)
      {
         *alloc_len = *len + STR_ALLOC_SIZE;
         *str = (char *)
            ((*str == NULL)
             ? malloc(*alloc_len)
             : realloc(*str, *alloc_len));

         if (*str == NULL)
            return FALSE;
      }

   (*str)[(*len)++] = ch;

   return TRUE;
}


/*
 * *** FUNCTIONS FOR OUTPUT
 */

/*
 * Write "data" member of field to file in Ascii format.
 * If annotate != NULL, add annotations in "[]" for readability.
 * Return TRUE on success, FALSE on failure.
 */

static int
WriteAsciiData(FieldSpec *field,
               FILE      *file,
               Annot     *annotate)
{
   long    length;              /* number of data elements */

   if (file == NULL || field == NULL || field->type == NO_TYPE)
      return FALSE;

   length = FieldLength(field);

   if (length == 0)
      return TRUE;

   if (field->data == NULL)
      return FALSE;

   if (annotate == NULL)
      return (AsciiWrite(field->data, field->type,
                         length, file, annotate) == length);
   else
      {
         if (length > 1 || field->type == ARRAY)
            fprintf(file, "\n%*.0s", annotate->indent, "");

         return AsciiWriteSub(field->data, field->type, length, file,
                              field->rank, field->dim, annotate);
      }
}


/*
 *
 */

static int
AsciiWriteSub(void  *data,
              int   type,
              long  length,
              FILE  *file,
              int   rank,
              long  *dim,
              Annot *annotate)
{
   int          i;
   long dim0, j;
   long step, stride;
   int          ind;

   if (rank == 0)
      {
         AsciiWrite(data, type, 1, file, annotate);
      }
   else if (length == dim[0])
      {
         if (type == ARRAY)
            step = 1;
         else
            {
               step = (annotate->width - annotate->indent - 3*rank - 2)
                  / ApproxWidth(type);
               if (step < 1)
                  step = 1;
            }
         stride = step*InternTypeSize(type);

         ind = annotate->indent;
         for (j = 0; length > 0; j += step, length -= step)
            {
               if (j > 0)
                  {
                     fprintf(file, "\n%*.0s", ind, "");
                     data = (char *) data + stride;
                  }
               annotate->indent =
                  ind + fprintf(file, "[%ld]", j) + 3*(rank - 1) + 2;
               for (i = 1; i < rank; i++)
                  fprintf(file, "[0]");
               fprintf(file, "  ");
               AsciiWrite(data, type,
                          (length < step) ? length : step, file, annotate);
            }
         annotate->indent = ind;
      }
   else
      {
         dim0 = dim[0];

         dim += 1;
         rank -= 1;
         length /= dim0;
         stride = length*InternTypeSize(type);
         ind = annotate->indent;
         for (j = 0; j < dim0; j++)
            {
               if (j > 0)
                  fprintf(file, "\n%*.0s", ind, "");
               annotate->indent = ind + fprintf(file, "[%ld]", j);
               AsciiWriteSub(j*stride + (char *) data, type, length, file,
                             rank, dim, annotate);
            }
         annotate->indent = ind;
      }

   /*! error checks needed */
   return TRUE;
}


/*
 * Write to file in Ascii format a sequence of "length" values of data
 * type "type", found starting at the location indicated by "datap".
 * Return the number of values successfully written, which will be less
 * than "length" in case of error.
 */

static int
AsciiWrite(void     *datap,
           int      type,
           long     length,
           FILE     *file,
           Annot    *annotate)
{
   long    j;

   /* AsciiWrite is called from WriteAsciiSamples, WriteAsciiData,
    * WriteAsciiArray, and AsciiWrite itself.  In all cases it has
    * been checked that file != NULL, data != NULL, and length != 0.
    */

   j = 0;

   switch (type)
      {
      case ARRAY:
         {
            Array   *data = (Array *) datap;

            WriteAsciiArray(&data[0], file, annotate);
            for (j = 1; j < length; j++)
               {
                  putc(' ', file);
                  WriteAsciiArray(&data[j], file, annotate);
               }
         }
         break;
      case DOUBLE:
         {
            double  *data = (double *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, DBL_FMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " DBL_FMT, data[j]);
               }
            else
               {
                  fprintf(file, DBL_WFMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " DBL_WFMT, data[j]);
               }
         }
         break;
      case FLOAT:
         {
            float   *data = (float *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, FLT_FMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " FLT_FMT, data[j]);
               }
            else
               {
                  fprintf(file, FLT_WFMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " FLT_WFMT, data[j]);
               }
         }
         break;
      case LONG:
         {
            long    *data = (long *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, LNG_FMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " LNG_FMT, data[j]);
               }
            else
               {
                  fprintf(file, LNG_WFMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " LNG_WFMT, data[j]);
               }
         }
         break;
      case ULONG:
         {
            Ulong   *data = (Ulong *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, ULN_FMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " ULN_FMT, data[j]);
               }
            else
               {
                  fprintf(file, ULN_WFMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " ULN_WFMT, data[j]);
               }
         }
         break;
      case SHORT:
         {
            short   *data = (short *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, SHR_FMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SHR_FMT, data[j]);
               }
            else
               {
                  fprintf(file, SHR_WFMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SHR_WFMT, data[j]);
               }
         }
         break;
      case USHORT:
         {
            Ushort  *data = (Ushort *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, USH_FMT, (unsigned) data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " USH_FMT, (unsigned) data[j]);
               }
            else
               {
                  fprintf(file, USH_WFMT, (unsigned) data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " USH_WFMT, (unsigned) data[j]);
               }
         }
         break;
      case SCHAR:
         {
            Schar   *data = (Schar *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, SCH_FMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SCH_FMT, data[j]);
               }
            else
               {
                  fprintf(file, SCH_WFMT, data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SCH_WFMT, data[j]);
               }
         }
         break;
      case UCHAR:
         {
            Uchar   *data = (Uchar *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, UCH_FMT, (unsigned) data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " UCH_FMT, (unsigned) data[j]);
               }
            else
               {
                  fprintf(file, UCH_WFMT, (unsigned) data[0]);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " UCH_WFMT, (unsigned) data[j]);
               }
         }
         break;
      case BOOL:
         {
            Bool    *data = (Bool *) datap;

            for (j = 0; j < length; j++)
               putc((data[j]) ? '1' : '0', file);
         }
         break;
      case DOUBLE_COMPLEX:
         {
            DoubleComplex  *data = (DoubleComplex *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, DBLCX_FMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " DBLCX_FMT,
                             data[j].real, data[j].imag);
               }
            else
               {
                  fprintf(file, DBLCX_WFMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " DBLCX_WFMT,
                             data[j].real, data[j].imag);
               }
         }
         break;
      case FLOAT_COMPLEX:
         {
            FloatComplex   *data = (FloatComplex *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, FLTCX_FMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " FLTCX_FMT,
                             data[j].real, data[j].imag);
               }
            else
               {
                  fprintf(file, FLTCX_WFMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " FLTCX_WFMT,
                             data[j].real, data[j].imag);
               }
         }
         break;
      case LONG_COMPLEX:
         {
            LongComplex    *data = (LongComplex *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, LNGCX_FMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " LNGCX_FMT,
                             data[j].real, data[j].imag);
               }
            else
               {
                  fprintf(file, LNGCX_WFMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " LNGCX_WFMT,
                             data[j].real, data[j].imag);
               }
         }
         break;
      case SHORT_COMPLEX:
         {
            ShortComplex   *data = (ShortComplex *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, SHRCX_FMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SHRCX_FMT,
                             data[j].real, data[j].imag);
               }
            else
               {
                  fprintf(file, SHRCX_WFMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SHRCX_WFMT,
                             data[j].real, data[j].imag);
               }
         }
         break;
      case SCHAR_COMPLEX:
         {
            ScharComplex   *data = (ScharComplex *) datap;

            if (annotate == NULL)
               {
                  fprintf(file, SCHCX_FMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SCHCX_FMT,
                             data[j].real, data[j].imag);
               }
            else
               {
                  fprintf(file, SCHCX_WFMT,
                          data[0].real, data[0].imag);
                  for (j = 1; j < length; j++)
                     fprintf(file, " " SCHCX_WFMT,
                             data[j].real, data[j].imag);
               }
         }
         break;
      case CHAR:
         j = AsciiWriteChar((char *) datap, length, file);
         break;
      case WCHAR:
         j = AsciiWriteWchar((Wchar *) datap, length, file);
         break;
      default:
         return 0;              /* Unsupported type. */
      }

   return j;
}


/*
 * Write to file the Ascii representation of one item of type ARRAY,
 * including the surrounding parentheses, data-type name, dimensions,
 * separating colon, and data values.  Return TRUE for success, FALSE
 * for failure.
 */

static int
WriteAsciiArray(Array *array, FILE *file, Annot *annotate)
{
   short   type;
   short   rank;
   long    *dim;
   void    *data;
   int     i;
   long    length;
   int      ind;

   if (file == NULL || array == NULL)
      return FALSE;

   type = array->type;
   rank = array->rank;
   dim = array->dim;
   data = array->data;

   if (!ValidType(type))
      return FALSE;

   fprintf(file, "(");
   fprintf(file, "%s ", TypeName(type));

   if (rank > 0 && dim == NULL)
      return FALSE;

   for (i = 0; i < rank; i++)
      fprintf(file, "%ld ", dim[i]);

   fprintf(file, ": ");

   length = LongProd(rank, dim);

   if (length > 0)
      {
         if (data == NULL)
            return FALSE;

         if (annotate == NULL)
            {
               if (AsciiWrite(data, type, length, file, annotate) != length)
                  return FALSE;
            }
         else
            {
               ind = annotate->indent;
               annotate->indent += 2;

               if (length > 1 || type == ARRAY)
                  fprintf(file, "\n%*.0s", annotate->indent, "");

               if (!AsciiWriteSub(data, type,
                                  length, file, rank, dim, annotate))
                  {
                     annotate->indent = ind;
                     return FALSE;
                  }

               annotate->indent = ind;
            }
      }

   fprintf(file, ")");

   return TRUE;
}


/*
 * Write to file in Ascii format the field specification indicated by
 * field, including any subfields.  The name consists of (depth - 1)
 * components found in the string array "names", followed by the string
 * field->name, with separating dots (".") supplied by the function.
 * If annotate != NULL, add annotations in "[]" for readability.
 * The top-level call (in WriteAsciiFieldList) has depth = 1.
 * Subfields are written by recursive calls within this function,
 * with depth > 1.
 */

static int
WriteAsciiFieldSpec(FieldSpec *field,
                    int       depth,
                    FILE      *file,
                    Annot     *annotate)
{
   static char  *names[MAX_FLD_DEPTH]; /* field name components */
   int          i;              /* loop index */
   int          rank;           /* number of dimensions */
   long *dim;           /* dimensions */
   int          num_misc;       /* number of "misc" attributes */
   char **axis_names;   /* axis_names attribute */
   char occ;            /* occurrence-class code */
   FieldList    subfields;      /* subfield list */


   if (file == NULL || field == NULL)
      return FALSE;

   if (depth > MAX_FLD_DEPTH)
      return FALSE;             /* Subfield structure too deep. */

   /*
    * Write full name, ":".
    */

   names[depth-1] = field->name;

   fprintf(file, "%s", names[0]);

   for (i = 1; i < depth; i++)
      fprintf(file, ".%s", names[i]);

   fprintf(file, ": ");

   /*
    * Write data type.
    */

   if (!ValidType(field->type))
      return FALSE;             /* Bad data type. */

   fprintf(file, "%s ", TypeName(field->type));

   /*
    * Write dimensions.
    */

   rank = field->rank;
   dim = field->dim;

   if (rank > 0 && dim == NULL)
      return FALSE;             /* Inconsistent rank and dimensions. */

   for (i = 0; i < rank; i++)
      fprintf(file, "%ld ", dim[i]);

   /*
    * Write misc attributes, if any non-default.
    */

   num_misc = 0;

   /* units */

   if (field->units != NULL && *field->units != '\0')
      {
         fprintf(file, (num_misc == 0) ? "{" : "; ");
         num_misc++;

         fprintf(file, "units: ");
         (void) WriteAsciiString(field->units, file);
      }

   /* scale */

   if (field->scale != 1.0)
      {
         fprintf(file, (num_misc == 0) ? "{" : "; ");
         num_misc++;

         fprintf(file, "scale: " DBL_FMT, field->scale);
      }

   /* offset */

   if (field->offset != 0.0)
      {
         fprintf(file, (num_misc == 0) ? "{" : "; ");
         num_misc++;

         fprintf(file, "offset: " DBL_FMT, field->offset);
      }

   /* axis_names */

   if (field->axis_names != NULL)
      {
         fprintf(file, (num_misc == 0) ? "{" : "; ");
         num_misc++;

         fprintf(file, "axis_names: ");
         axis_names = field->axis_names;
         for (i = 0; i < rank; i++)
            fprintf(file, (i == 0) ? "%s" : ", %s",
                    (axis_names[i] == NULL) ? "" : axis_names[i]);
      }

   /* "}" if needed */

   if (num_misc > 0)
      fprintf(file, "} ");

   /*
    * Write occurrence class.
    */

   switch (field->occurrence)
      {
      case GLOBAL:
         occ = 'g';
         break;
      case REQUIRED:
         occ = 'r';
         break;
      case OPTIONAL:
         occ = 'o';
         break;
      case VIRTUAL:
         occ = 'v';
         break;
      case INCLUDED:
         occ = 'i';
         break;
      default:
         return FALSE;          /* Bad occurrence class. */
      }

   fprintf(file, "<%c>", occ);

   /*
    * Write data.
    */

   if (field->type != NO_TYPE
       && field->occurrence != REQUIRED && field->occurrence != OPTIONAL)
      {
         putc(' ', file);

         if (annotate != NULL)
            {
               annotate->indent = 2;
            }

         if (!WriteAsciiData(field, file, annotate))
            return FALSE;       /* Failed writing data. */

         if (annotate != NULL)
            {
               annotate->indent = 0;
            }
      }

   /*
    * Write terminating newline.
    */

   fprintf(file, "\n");

   /*
    * Write subfield specs.
    */

   subfields = field->subfields;
   if (subfields != NULL)
      for (i = 0; subfields[i] != NULL; i++)
         if (!WriteAsciiFieldSpec(subfields[i],
                                  depth + 1, file, annotate))
            return FALSE;       /* Failed writing subfield spec. */

   return TRUE;         /* Success. */
}


/*
 * Write to file a string constant representing the string beginning
 * at "data".  The string is enclosed in double quotes (").  Printing
 * characters other than double quote and backslash (\) represent
 * themselves.  Double quote, and backslash are represented by Ansi C
 * single-character escapes.
 */

static int
WriteAsciiString(char *data,
FILE *file)
{
    int    ch;                  /* character to write */
    long    j;                  /* loop index */

    putc('"', file);

    for (j = 0; (ch = data[j]) != '\0'; j++)
        switch (ch)
        {
        case '"':
            fputs("\\\"", file);
            break;
        case '\\':
            fputs("\\\\", file);
            break;
        default:
            if (isprint(ch))
            {
                putc(ch, file);
            }
            else return FALSE;

            break;
        }

    putc('"', file);

    return TRUE;
}


/*
 * Write to file a string constant representing the array of "size"
 * characters beginning at "data".  The string is enclosed in double
 * quotes (").  Printing characters other than double quote and
 * backslash (\) represent themselves.  Double quote, backslash, and
 * non-printing characters are represented by Ansi C single-character
 * escapes or two-digit hex escapes.
 */

static int
AsciiWriteChar(char     *data,
               long     size,
               FILE     *file)
{
    int    ch;                  /* character to write */
    long    j;                  /* loop index */

    putc('"', file);

    for (j = 0; j < size; j++)
    {
        ch = (Uchar) data[j];

        switch (ch)
        {
        case '"':
            fputs("\\\"", file);
            break;
        case '\\':
            fputs("\\\\", file);
            break;
        case '\0':
            fputs("\\0", file);
            break;
        case '\a':
            fputs("\\a", file);
            break;
        case '\b':
            fputs("\\b", file);
            break;
        case '\f':
            fputs("\\f", file);
            break;
        case '\n':
            fputs("\\n", file);
            break;
        case '\r':
            fputs("\\r", file);
            break;
        case '\t':
            fputs("\\t", file);
            break;
        case '\v':
            fputs("\\v", file);
            break;
        default:
            if (isprint(ch))
            {
                putc(ch, file);
            }
            else
            {
                /*
                 * Non-printing characters without special single-
                 * character escapes are written in hex.  A two-digit
                 * hex escape like \x2f followed by a character like f
                 * that happens to be a hex digit is interpreted as a
                 * single three-digit hex escape, rather than two
                 * characters, if written solid ("\x2ff").  Making a
                 * break (e.g. "\x2f" "f") resolves the ambiguity.
                 */
                fprintf(file,
                        (j < size - 1 && isxdigit((int) data[j+1]))
                        ? "\\x%02x\" \""
                        : "\\x%02x",
                        ch);
            }
            break;
        }
    }

    putc('"', file);

    return size;
}


/*
 * Write to file a string constant representing the array of "size"
 * wide characters beginning at "data".  The string is enclosed in
 * double quotes (").  Printing characters other than double quote and
 * backslash (\) represent themselves.  Double quote, backslash, and
 * non-printing characters are represented by Ansi C single-character
 * escapes or four-digit hex escapes.
 */

static int
AsciiWriteWchar(Wchar   *data,
                long    size,
                FILE    *file)
{
   long    ch;                  /* wide character to write */
   long    j;                   /* loop index */

   putc('"', file);

   for (j = 0; j < size; j++)
      {
         ch = data[j];

         switch (ch)
            {
            case '"':
               fputs("\\\"", file);
               break;
            case '\\':
               fputs("\\\\", file);
               break;
            case '\0':
               fputs("\\0", file);
               break;
            case '\a':
               fputs("\\a", file);
               break;
            case '\b':
               fputs("\\b", file);
               break;
            case '\f':
               fputs("\\f", file);
               break;
            case '\n':
               fputs("\\n", file);
               break;
            case '\r':
               fputs("\\r", file);
               break;
            case '\t':
               fputs("\\t", file);
               break;
            case '\v':
               fputs("\\v", file);
               break;
            default:
               if (((Ulong) ch < 0x7f) && isprint(ch))
                  {
                     putc(ch, file);
                  }
               else
                  {
                     /*
                      * Non-printing characters without special single-
                      * character escapes are written as four-digit hex
                      * escapes.  (Cf. two digits for CHAR.)
                      */
                     fprintf(file,
                             (j < size - 1 && (((Ulong) data[j+1] < 0x7f)
                                               && isxdigit(data[j+1])))
                             ? "\\x%04lx\" \""
                             : "\\x%04lx",
                             ch);
                  }
               break;
            }
      }

   putc('"', file);

   return size;
}


/*
 *
 */

static long
ApproxWidth(int type)
{
   switch (type)
      {
      case ARRAY:
         return -1;
      case DOUBLE:
         return (DBL_W) + 1;
      case FLOAT:
         return (FLT_W) + 1;
      case LONG:
         return (ULN_W) + 1;
      case ULONG:
         return (LNG_W) + 1;
      case SHORT:
         return (SHR_W) + 1;
      case USHORT:
         return (USH_W) + 1;
      case SCHAR:
         return (SCH_W) + 1;
      case UCHAR:
         return (UCH_W) + 1;
      case BOOL:
         return 1;
      case DOUBLE_COMPLEX:
         return 2*(DBLCX_W) + 5;
      case FLOAT_COMPLEX:
         return 2*(FLTCX_W) + 5;
      case LONG_COMPLEX:
         return 2*(LNGCX_W) + 5;
      case SHORT_COMPLEX:
         return 2*(SHRCX_W) + 5;
      case SCHAR_COMPLEX:
         return 2*(SCHCX_W) + 5;
      case CHAR:
         return CHR_W;
      case WCHAR:
         return WCH_W;
      default:
         {
            DebugMsg(1, "ApproxWidth: Invalid code or NO_TYPE.");
            return 0;
         }
      }
}
