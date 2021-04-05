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
 * EDR1 and EDR2 portable binary I/O.
 *
 * Author:  Rod Johnson
 */


#include "esignal.h"

/*
 * LOCAL CONSTANTS
 */

/*
 *  LOCAL FUNCTION DECLARATIONS
 */

/* Functions for input */

static int          ReadEdrData(FieldSpec *field,
                                FILE *file, int longlong);
static FieldSpec    *ReadEdrFieldSpec(FILE *file, int longlong);
static int          ReadEdrString(char **string,
                                  FILE *file, int longlong);
static int          EdrRead(void *data, int type,
                            long length, FILE *file, int longlong);
static int          ReadEdrArray(Array *array,
                                 FILE *file, int longlong);

static int          EdrReadArray(Array *data,
                                 long length, FILE *file, int longlong);
static int          EdrReadDouble(double *data,
                                  long length, FILE *file);
static int          EdrReadFloat(float *data,
                                 long length, FILE *file);
static int          EdrReadLong(long *data,
                                long length, FILE *file, int longlong);
static int          EdrReadUlong(Ulong *data,
                                 long length, FILE *file, int longlong);
static int          EdrReadShort(short *data,
                                 long length, FILE *file);
static int          EdrReadUshort(Ushort *data,
                                  long length, FILE *file);
static int          EdrReadSchar(Schar *data,
                                 long length, FILE *file);
static int          EdrReadUchar(Uchar *data,
                                 long length, FILE *file);
static int          EdrReadBool(Bool *data,
                                long length, FILE *file);
static int          EdrReadDoubleComplex(DoubleComplex *data,
                                         long length, FILE *file);
static int          EdrReadFloatComplex(FloatComplex *data,
                                        long length, FILE *file);
static int          EdrReadLongComplex(LongComplex *data, long length,
                                       FILE *file, int longlong);
static int          EdrReadShortComplex(ShortComplex *data,
                                        long length, FILE *file);
static int          EdrReadScharComplex(ScharComplex *data,
                                        long length, FILE *file);
static int          EdrReadChar(char *data,
                                long length, FILE *file);
static int          EdrReadWchar(Wchar *data,
                                 long length, FILE *file);

/* Functions for output */

static int          WriteEdrData(FieldSpec *field,
                                 FILE *file, int longlong);
static int          WriteEdrFieldSpec(FieldSpec *field,
                                      FILE *file, int longlong);
static int          WriteEdrString(char *string,
                                   FILE *file, int longlong);
static int          EdrWrite(void *data, int type,
                             long length, FILE *file, int longlong);
static int          WriteEdrArray(Array *array,
                                  FILE *file, int longlong);

static int          EdrWriteArray(Array *data, long length,
                                  FILE *file, int longlong);
static int          EdrWriteDouble(double *data,
                                   long length, FILE *file);
static int          EdrWriteFloat(float *data, long length, FILE *file);
static int          EdrWriteLong(long *data,
                                 long length, FILE *file, int longlong);
static int          EdrWriteUlong(Ulong *data,
                                  long length, FILE *file, int longlong);
static int          EdrWriteShort(short *data, long length, FILE *file);
static int          EdrWriteUshort(Ushort *data, long length, FILE *file);
static int          EdrWriteSchar(Schar *data, long length, FILE *file);
static int          EdrWriteUchar(Uchar *data, long length, FILE *file);
static int          EdrWriteBool(Bool *data, long length, FILE *file);
static int          EdrWriteDoubleComplex(DoubleComplex *data,
                                          long length, FILE *file);
static int          EdrWriteFloatComplex(FloatComplex *data,
                                         long length, FILE *file);
static int          EdrWriteLongComplex(LongComplex *data, long length,
                                        FILE *file, int longlong);
static int          EdrWriteShortComplex(ShortComplex *data,
                                         long length, FILE *file);
static int          EdrWriteScharComplex(ScharComplex *data,
                                         long length, FILE *file);
static int          EdrWriteChar(char *data,
                                 long length, FILE *file);
static int          EdrWriteWchar(Wchar *data,
                                  long length, FILE *file);

/*
 * PUBLIC FUNCTION DEFINITIONS
 * - EdrTypeSize
 * - EdrRecordSize
 * - ReadEdrFieldList
 * - ReadEdrRecord
 * - ReadEdrSamples
 * - WriteEdrFieldList
 * - WriteEdrRecord
 * - WriteEdrSamples
 */

/*
 * Size in bytes of EDR1 or EDR2 external representation of data type.
 */

long
EdrTypeSize(int type,           /* numeric data-type code */
            int longlong)
{
   switch (type)
      {
      case ARRAY:
         return -1;
      case DOUBLE:
         return 8;
      case FLOAT:
         return 4;
      case LONG:
      case ULONG:
         switch (longlong)
            {
            case EDR1:  return 4;
            case EDR2:  return 8;
            default:    return 0;
            }
      case SHORT:
      case USHORT:
         return 2;
      case SCHAR:
      case UCHAR:
         return 1;
      case BOOL:
         return 1;
      case DOUBLE_COMPLEX:
         return 16;
      case FLOAT_COMPLEX:
         return 8;
      case LONG_COMPLEX:
         switch (longlong)
            {
            case EDR1:  return 8;
            case EDR2:  return 16;
            default:    return 0;
            }
      case SHORT_COMPLEX:
         return 4;
      case SCHAR_COMPLEX:
         return 2;
      case CHAR:
         return 1;
      case WCHAR:
         return 2;
      default:
         {
            DebugMsg(1, "EdrTypeSize: Invalid code or NO_TYPE.");
            return 0;
         }
      }
}


/*
 * Return the size, in bytes, of one record in EDR1 or EDR2 format,
 * according to the field specs on a field list.
 * Return 0 on NULL input; -1 is a code for variable-length records.
 */

long
EdrRecordSize(FieldList list,
              int       longlong)
{
   long size;           /* total array size in bytes */
   FieldSpec    **fld_order;
   long i;
   long fld_size;

   fld_order = FieldOrder(list);

   if (fld_order == NULL)
      return 0;

   size = 0;

   for (i = 0 ; fld_order[i] != NULL; i++)
      {
         if (fld_order[i]->occurrence == OPTIONAL)
            {
               size = -1;
               break;
            }

         fld_size =
            FieldLength(fld_order[i])
            * EdrTypeSize(fld_order[i]->type, longlong);

         if (fld_size < 0)
            {
               size = -1;
               break;
            }
         else
            size += fld_size;
      }

   free(fld_order);

   return size;
}


/*
 * Read field list in EDR1 or EDR2 format from file.  Return TRUE on
 * success, FALSE on failure.
 */

int
ReadEdrFieldList(FieldList  *listp,  /* output variable */
                 FILE       *file,   /* input file */
                 int        longlong)
{
   FieldList    list;
   long num_fields;
   long i;
   FieldSpec    *spec;


   if (EdrReadLong(&num_fields, 1, file, longlong) != 1)
      {
         DebugMsg(1, "ReadEdrFieldList: Couldn't get number of fields.");
         return FALSE;
      }

   list = NULL;

   for (i = 0; i < num_fields; i++)
      {
         spec = ReadEdrFieldSpec(file, longlong);
         if (spec == NULL || !AddField(&list, spec))
            {
               if (list != NULL)
                  FreeFieldList(list);
               DebugMsg(2, ((spec == NULL)
                            ? "ReadEdrFieldList: couldn't read field spec."
                            : ("ReadEdrFieldList: "
                               "couldn't add field spec to list.")));
               return FALSE;
            }
      }

   *listp = list;

   return TRUE;
}


/*
 * Read one record from file in EDR1 or EDR2 format, depending on the
 * value of longlong.  Read the record into the "data" members of
 * the field specs on a NULL-terminated linear array of REQUIRED and
 * OPTIONAL fields, like those produced by TypeOrder and FieldOrder.
 * Return TRUE on success, FALSE on failure.
 */

int
ReadEdrRecord(FieldSpec **fields,
              FILE      *file,
              int       longlong)
{
   long    i;
   long    nopt;
   Uchar   flags;

   if (file == NULL || fields == NULL)
      {
         DebugMsg(1, "ReadEdrRecord: NULL argument.");
         return FALSE;
      }

   /*! If FieldOrder & TypeOrder returned a linear array of OPTIONAL
    * fields as well as the array of REQUIRED & OPTIONAL, we could
    * avoid scanning all of "fields" checking for OPTIONAL entries
    * for every record read. */

   nopt = 0;

   for (i = 0 ; fields[i] != NULL; i++)
      {
         if (fields[i]->occurrence == OPTIONAL)
            {
               if (nopt % 8 == 0)
                  {
                     if (EdrReadUchar(&flags, 1, file) != 1)
                        {
                           DebugMsg(1, ("ReadEdrRecord: can't read "
                                        "\"presence\" flag for OPTIONAL field."));
                           return FALSE;
                        }
                  }
               else
                  flags <<= 1;

               fields[i]->present = ((flags & 0x80) != 0);

               nopt++;
            }
      }

   for (i = 0 ; fields[i] != NULL; i++)
      if (fields[i]->occurrence == REQUIRED || fields[i]->present)
         {
            if (!ReadEdrData(fields[i], file, longlong))
               {
                  DebugMsg(2, "ReadEdrRecord: couldn't read field data.");
                  return FALSE;
               }
         }

   return TRUE;
}


/*
 * Read nrec one-field records from file in EDR1 or EDR2 format,
 * depending on the value of longlong.  Read the records into
 * the array indicated by data.  The field is specified by fields,
 * a NULL-terminated linear array like those produced by TypeOrder
 * and FieldOrder and containing exactly one REQUIRED field.
 * Return the number of complete records read.
 */

long
ReadEdrSamples(void         *data,
               long         nrec,
               FieldSpec    **fields,
               FILE         *file,
               int          longlong)
{
   int      type;
   long    length;

   if (data == NULL)
      {
         DebugMsg(1, "ReadEdrSamples: NULL data pointer.");
         return 0;
      }
   if (nrec < 0)
      {
         DebugMsg(1, ("ReadEdrSamples: "
                      "negative number of records specified."));
         return 0;
      }
   if (fields == NULL || fields[0] == NULL || fields[1] != NULL
       || fields[0]->occurrence != REQUIRED)
      {
         DebugMsg(1, "ReadEdrSamples: bad \"fields\" array.");
         return 0;
      }
   if (file == NULL)
      {
         DebugMsg(1, "ReadEdrSamples: NULL file pointer.");
         return 0;
      }

   type = fields[0]->type;

   if (type == NO_TYPE)
      return 0;

   length = FieldLength(fields[0]);

   if (nrec*length == 0)
      return nrec;

   return EdrRead(data, type, nrec*length, file, longlong) / length;
}


/*
 * Write field list to file in EDR1 or EDR2 format.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteEdrFieldList(FieldList list,   /* field list */
                  FILE      *file,  /* output file */
                  int       longlong)
{
   long    num_fields;          /* number of fields */
   long    i;                   /* loop index */

   if (file == NULL)
      {
         DebugMsg(1, "WriteEdrFieldList: NULL file pointer.");
         return FALSE;
      }

   num_fields = FieldListLength(list);
   if (EdrWriteLong(&num_fields, 1, file, longlong) != 1)
      {
         DebugMsg(1, "WriteEdrFieldList: couldn't write number of fields.");
         return FALSE;
      }

   for (i = 0; i < num_fields; i++)
      if (!WriteEdrFieldSpec(list[i], file, longlong))
         {
            DebugMsg(2, "WriteEdrFieldList: couldn't write field spec.");
            return FALSE;
         }

   return TRUE;         /* Success. */
}


/*
 * Write to file one record in EDR1 or EDR2 format, consisting of the
 * contents of the "data" members of the field specs on a NULL-terminated
 * linear array of REQUIRED and OPTIONAL fields, like those produced by
 * TypeOrder and FieldOrder.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteEdrRecord(FieldSpec    **fields,
               FILE         *file,
               int          longlong)
{
   long    i;
   long    nopt;
   Uchar   flags;

   if (file == NULL || fields == NULL)
      {
         DebugMsg(1, "WriteEdrRecord: NULL argument.");
         return FALSE;
      }

   /*! If FieldOrder & TypeOrder returned a linear array of OPTIONAL
    * fields as well as the array of REQUIRED & OPTIONAL, we could
    * avoid scanning all of "fields" checking for OPTIONAL entries
    * for every record written. */

   nopt = 0;
   flags = 0;

   for (i = 0; fields[i] != NULL; i++)
      {
         if (fields[i]->occurrence == OPTIONAL)
            {
               flags |= fields[i]->present;

               nopt++;

               if (nopt % 8 == 0)
                  {
                     if (EdrWriteUchar(&flags, 1, file) != 1)
                        {
                           DebugMsg(2, ("WriteEdrRecord: couldn't write "
                                        "\"presence\" flag for OPTIONAL field."));
                           return FALSE;
                        }

                     flags = 0;
                  }
               else
                  flags <<= 1;
            }
      }

   if (nopt % 8 != 0)
      {
         flags <<= (7 - nopt % 8);

         if (EdrWriteUchar(&flags, 1, file) != 1)
            {
               DebugMsg(1, ("WriteEdrRecord: couldn't write "
                            "\"presence\" flags for OPTIONAL fields."));
               return FALSE;
            }
      }

   for (i = 0; fields[i] != NULL; i++)
      {
         if (fields[i]->occurrence == REQUIRED || fields[i]->present)
            {
               if (!WriteEdrData(fields[i], file, longlong))
                  {
                     DebugMsg(2, "WriteEdrRecord: couldn't write field data.");
                     return FALSE;
                  }
            }
      }

   return TRUE;
}


/*
 * Write nrec one-field records to file in EDR1 or EDR2 format,
 * depending on the value of longlong.  The values are obtained
 * from the array indicated by data.  The field is specified by fields,
 * a NULL-terminated linear array like those produced by TypeOrder
 * and FieldOrder and containing exactly one REQUIRED field.
 * Return the number of complete records written.
 */

long
WriteEdrSamples(void        *data,
                long        nrec,
                FieldSpec   **fields,
                FILE        *file,
                int         longlong)
{
   int          type;
   long length;

   if (data == NULL)
      {
         DebugMsg(1, "WriteEdrSamples: NULL data pointer.");
         return 0;
      }
   if (nrec < 0)
      {
         DebugMsg(1, ("WriteEdrSamples: "
                      "negative number of records specified."));
         return 0;
      }
   if (fields == NULL || fields[0] == NULL || fields[1] != NULL
       || fields[0]->occurrence != REQUIRED)
      {
         DebugMsg(1, "WriteEdrSamples: bad \"fields\" array.");
         return 0;
      }
   if (file == NULL)
      {
         DebugMsg(1, "WriteEdrSamples: NULL file pointer.");
         return 0;
      }

   type = fields[0]->type;

   if (type == NO_TYPE)
      return 0;

   length = FieldLength(fields[0]);

   if (nrec*length == 0)
      return nrec;

   return EdrWrite(data, type, nrec*length, file, longlong) /length;
}


/*
 * LOCAL FUNCTION DEFINITIONS
 */

/*
 * *** FUNCTIONS FOR INPUT
 */

/*
 * Read "data" member of field from file in EDR format;
 * allocate if NULL.  Return TRUE on success, FALSE on failure.
 */

static int
ReadEdrData(FieldSpec *field,
            FILE *file,
            int longlong)
{
   long  size;          /* size of element (bytes) */
   long    length;              /* number of elements */

   if (file  == NULL || field == NULL || field->type == NO_TYPE)
      {
         DebugMsg(1, "ReadEdrData: NULL argument or type NO_TYPE.");
         return FALSE;
      }

   size = InternTypeSize(field->type);
   length = FieldLength(field);

   if (field->data == NULL && length != 0)
      {
         field->data = malloc(length * size);
         if (field->data == NULL)
            {
               DebugMsg(1, "ReadEdrData: allocation failure.");
               return FALSE;
            }
      }

   return (EdrRead(field->data, field->type, length, file, longlong)
           == length);
}


/*
 * Read field specification in EDR format from file and return
 * pointer to it; return NULL on failure.
 */

static FieldSpec *
ReadEdrFieldSpec(FILE   *file,
                 int    longlong)
{
   char *name;          /* field name */
   short        type;           /* data type code */
   short        rank;           /* number of dimensions */
   int          i;              /* loop index */
   FieldSpec    *field;         /* field spec being read */
   long num_ax_names;   /* number of axis names */

   /* Read name, type, rank. */

   if (!ReadEdrString(&name, file, longlong))
      return NULL;              /* Failure getting field name. */

   if (EdrReadShort(&type, 1, file)  != 1)
      return NULL;              /* Couldn't get type. */

   if (EdrReadShort(&rank, 1, file) != 1)
      return NULL;              /* Couldn't get rank. */

   /* Allocate structure. */

   field = NewFieldSpec(type, rank);
   if (field == NULL)
      return NULL;              /* Couldn't create field spec. */

   field->name = name;

   /* Read dimensions. */

   if (rank != 0 && field->dim == NULL)
      return NULL;              /* Inconsistent rank and dimensions. */

   if (EdrReadLong(field->dim, rank, file, longlong) != rank)
      return NULL;      /* Couldn't get dimensions. */

   /* Read units, scale, offset, axis_names. */

   if (!ReadEdrString(&field->units, file, longlong))
      return NULL;

   if (EdrReadDouble(&field->scale, 1, file) != 1)
      return NULL;              /* Couldn't get scale. */

   if (EdrReadDouble(&field->offset, 1, file) != 1)
      return NULL;              /* Couldn't get offset. */

   if (EdrReadLong(&num_ax_names, 1, file, longlong) != 1)
      return NULL;              /* Couldn't get number of axis names. */

   if (num_ax_names > rank)
      return NULL;              /* Bad value for num_ax_names. */

   if (num_ax_names != 0)
      {
         field->axis_names = (char **) malloc(rank * sizeof(char *));
         if (field->axis_names == NULL)
            return NULL;        /* Allocation failure. */

         for (i = 0; i < num_ax_names; i++)
            ReadEdrString(&field->axis_names[i], file, longlong);

         for ( ; i < rank; i++)
            field->axis_names[i] = NULL;
      }

   /* Read occurrence class. */

   if (EdrReadShort(&field->occurrence, 1, file) != 1)
      return NULL;              /* Couldn't get occurrence code. */

   /* Read data if required. */

   if (type != NO_TYPE
       && field->occurrence != REQUIRED && field->occurrence != OPTIONAL)
      {
         if (!ReadEdrData(field, file, longlong))
            return NULL;        /* Failure reading data. */
      }

   /* Read subfield list. */

   if (!ReadEdrFieldList(&field->subfields, file, longlong))
      {
         FreeFieldSpec(field);
         return NULL;           /* Failure getting subfields. */
      }

   return field;                /* Success. */
}


/*
 * Read character string in EDR format from file and output
 * pointer to it.  Return TRUE on success, FALSE on failure.
 */

static int
ReadEdrString(char  **string,
              FILE  *file,
              int   longlong)
{
   long    length;
   char    *str;
   long    i;
   int     ch;

   if (EdrReadLong(&length, 1, file, longlong) != 1)
      return FALSE;             /* Couldn't get length. */

   str = (char *) malloc(length + 1);
   if (str ==  NULL)
      return FALSE;

   for (i = 0; i < length && (ch = getc(file)) != EOF; i++)
      str[i] = ch;

   if (i < length)
      {
         free(str);
         return FALSE;
      }

   str[length] = '\0';

   if (string != NULL)
      *string = str;

   return TRUE;
}


/*
 * Read items of data from a file into a block of storage indicated
 * by a pointer "data".  The number of items is given by "length",
 * and their data type is indicated by the integer code "type".
 * Return the number of successfully read items.  (A value other
 * than "length" indicates an error).
 */

static int
EdrRead(void    *data,
        int     type,
        long    length,
        FILE    *file,
        int     longlong)
{
   if (file == NULL || data == NULL || length <= 0)
      return 0;

   switch (type)
      {
      case ARRAY:
         return EdrReadArray((Array *) data, length, file, longlong);
      case DOUBLE:
         return EdrReadDouble((double *) data, length, file);
      case FLOAT:
         return EdrReadFloat((float *) data, length, file);
      case LONG:
         return EdrReadLong((long *) data, length, file, longlong);
      case ULONG:
         return EdrReadUlong((Ulong *) data, length, file, longlong);
      case SHORT:
         return EdrReadShort((short *) data, length, file);
      case USHORT:
         return EdrReadUshort((Ushort *) data, length, file);
      case SCHAR:
         return EdrReadSchar((Schar *) data, length, file);
      case UCHAR:
         return EdrReadUchar((Uchar *) data, length, file);
      case BOOL:
         return EdrReadBool((Bool *) data, length, file);
      case DOUBLE_COMPLEX:
         return EdrReadDoubleComplex((DoubleComplex *) data, length, file);
      case FLOAT_COMPLEX:
         return EdrReadFloatComplex((FloatComplex *) data, length, file);
      case LONG_COMPLEX:
         return EdrReadLongComplex((LongComplex *) data,
                                   length, file, longlong);
      case SHORT_COMPLEX:
         return EdrReadShortComplex((ShortComplex *) data, length, file);
      case SCHAR_COMPLEX:
         return EdrReadScharComplex((ScharComplex *) data, length, file);
      case CHAR:
         return EdrReadChar((char *) data, length, file);
      case WCHAR:
         return EdrReadWchar((Wchar *) data, length, file);
      default:    /* Invalid code or NO_TYPE. */
         return 0;
      }
}


/*
 * Read from file the EDR1 or EDR2 representation of one item of type
 * ARRAY.  Store the type, rank, dimensions, and data in the members of
 * the array structure, allocating the necessary space for dimensions
 * and data.  Return TRUE for success, FALSE for failure.
 */

static int
ReadEdrArray(Array  *array,
             FILE   *file,
             int    longlong)
{
   short        type;           /* data type code */
   short        rank;           /* number of dimensions */
   long *dim;           /* dimensions */
   void *data;
   long length;
   long size;

   if (array == NULL || file == NULL)
      return FALSE;

   if (EdrReadShort(&type, 1, file)  != 1)
      return FALSE;             /* Couldn't get type. */

   if (EdrReadShort(&rank, 1, file) != 1)
      return FALSE;             /* Couldn't get rank. */

   if (rank == 0)
      dim = NULL;
   else
      {
         dim = (long *) malloc(rank * sizeof(long));
         if (dim == NULL)
            return FALSE;

         if (EdrReadLong(dim, rank, file, longlong) != rank)
            {
               free(dim);
               return FALSE;
            }
      }

   length = LongProd(rank, dim);
   size = InternTypeSize(type);

   if (length == 0)
      data = NULL;
   else
      {
         data = malloc(length*size);

         if (data == NULL
             || EdrRead(data, type, length, file, longlong) != length)
            {
               if (dim != NULL)
                  free(dim);
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
 * Each of the following functions with names of the form
 *
 *      EdrRead<type>
 *
 * takes three arguments:
 *      <type>  *data,
 *      long    length,
 *      FILE    *file,
 * and in some cases a fourth:
 *      int     longlong
 * The function reads "length" items of data of the indicated data type
 * in EDR portable binary format from the stream "file" into a block of
 * storage indicated by "data".  The fourth argument, "longlong", is
 * present when the EDR1 and EDR2 representations of the data type differ;
 * a value of 1 indicates EDR1, and 2 indicates EDR2.  The int return
 * value is the number of items successfully read, which will be less
 * than "length" in case of error.
 */

/* Read items of type ARRAY */

static int
EdrReadArray(Array      *data,
             long       length,
             FILE       *file,
             int        longlong)
{
   long n;

   for (n = 0;
        n < length && ReadEdrArray(&data[n], file, longlong);
        n++)
      { }

   return n;
}


/* Read items of type DOUBLE */

static int
EdrReadDouble(double    *data,
              long      length,
              FILE      *file)
{
   long     n;          /* number of items read */
   unsigned long   hi;          /* 28 high_order fraction bits
                                   (and hidden bit) */
   unsigned long   lo;          /* 24 low_order fraction bits  */
   int              ex;         /* exponent */
   int              sn;         /* sign */
   int              ch;         /* input character */
   double           item;       /* one input data item */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;
         sn = ((ch & 0x80) != 0); /* sign */
         ex = ch & 0x7f;                /* 7 bits of exponent */

         if ((ch = getc(file)) == EOF)
            break;
         ex = (ex << 4) | (ch >> 4); /* rest of exponent */
         hi = ch & 0x0f;                /* first 4 fraction bits */

         if ((ch = getc(file)) == EOF)
            break;
         hi = (hi << 8) | ch;

         if ((ch = getc(file)) == EOF)
            break;
         hi = (hi << 8) | ch;

         if ((ch = getc(file)) == EOF)
            break;
         hi = (hi << 8) | ch;

         if ((ch = getc(file)) == EOF)
            break;
         lo = ch;               /* start on low-order 24 bits */

         if ((ch = getc(file)) == EOF)
            break;
         lo = (lo << 8) | ch;

         if ((ch = getc(file)) == EOF)
            break;
         lo = (lo << 8) | ch;

         if (ex == 2047)                /* Inf or NaN */
            item = HUGE_VAL;
         else
            {
               if (ex == 0)     /* unnormalized */
                  ex = 1;               /* adjust for different exponent bias */
               else             /* normalized */
                  hi |= 0x10000000; /* hidden bit */

               item = ldexp((double) hi, 24) + (double) lo;
               /* fraction bits as integer, i.e. scaled by 2^52 */

               item = ldexp(item, ex - 1075);
               /* 1075: number of fractional places (52)
                  minus exponent bias (-1023) */
            }

         data[n] = (sn) ? -item : item;
      }

   return n;
}


/* Read items of type FLOAT */

static int
EdrReadFloat(float  *data,
             long   length,
             FILE   *file)
{
   long     n;          /* number of items read */
   unsigned long   fr;          /* 23 fraction bits (and hidden bit) */
   int              ex;         /* exponent */
   int              sn;         /* sign */
   int              ch;         /* input character */
   double           item;       /* one input data item */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;
         sn = ((ch & 0x80) != 0); /* sign */
         ex = ch & 0x7f;                /* 7 bits of exponent */

         if ((ch = getc(file)) == EOF)
            break;
         ex = (ex << 1) | (ch >> 7); /* rest of exponent */
         fr = ch & 0x7f;                /* first 7 fraction bits */

         if ((ch = getc(file)) == EOF)
            break;
         fr = (fr << 8) | ch;

         if ((ch = getc(file)) == EOF)
            break;
         fr = (fr << 8) | ch;

         if (ex == 255)         /* Inf or NaN */
            item = FLT_MAX;
         /*! What's a portable way to get the float equivalent of HUGE_VAL
          * in ANSI C? */
         else
            {
               if (ex == 0)     /* unnormalized */
                  ex = 1;               /* adjust for different exponent bias */
               else             /* normalized */
                  fr |= 0x800000;       /* hidden bit */

               item = ldexp((double) fr, ex - 150);
               /* 150: number of fractional places (23)
                  minus exponent bias (-127) */

               if (item > FLT_MAX)
                  item = FLT_MAX;
            }

         data[n] = (sn) ? -item : item;
      }

   return n;
}


/* Read items of type LONG */

static int
EdrReadLong(long    *data,
            long    length,
            FILE    *file,
            int     longlong)
{
   long     n;          /* number of items read */
   int              ch;         /* input character */

   switch (longlong)
      {
      case EDR1:
         {
            unsigned long
               item;    /* one input data item */

            for (n = 0; n < length; n++)
               {
                  if ((ch = getc(file)) == EOF)
                     break;
                  item = ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  item = (item << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  item = (item << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  item = (item << 8) | ch;

                  if ((item & 0x80000000) == 0)
                     data[n] = item;
#if (LONG_MIN == -0xffffffff)   /* Unlikely on 2's-complement machine,
                                 * but allowed by ANSI. */
                  else
                     if (item == 0x80000000)
                        {
                           data[n] = LONG_MIN;
                           /* CLIPPING */
                        }
#endif
                     else
                        data[n] = -1L - (long) (item ^ 0xffffffff);
               }

            return n;
         }
         break;
      case EDR2:
         {
            unsigned long
               hi, lo;

            for (n = 0; n < length; n++)
               {
                  /* Get high-order 4 bytes */

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = (hi << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = (hi << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = (hi << 8) | ch;

                  /* Get low-order 4 bytes */

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = (lo << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = (lo << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = (lo << 8) | ch;

                  /* Combine the pieces */

#if (ULONG_MAX > 0xffffffffUL)  /* unsigned longs more than 32 bits */
                  {
                     const unsigned long
                        lmaxhi = LONG_MAX >> 32,
                        lmaxlo = LONG_MAX & 0xffffffffUL,
                        lminhi = ~ (~ (unsigned long) LONG_MIN >> 32),
                        lminlo = LONG_MIN & 0xffffffffUL,
                        sgnext = ~ 0xffffffffUL;

                     if ((hi & 0x80000000) == 0)        /* non-negative */
                        {
                           if (hi > lmaxhi || hi == lmaxhi && lo > lmaxlo)
                              {
                                 data[n] = LONG_MAX;
                                 /* CLIPPING */
                              }
                           else
                              data[n] = (hi << 32) | lo;
                        }
                     else       /* negative */
                        {
                           hi |= sgnext;

                           if (hi < lminhi || hi == lminhi && lo < lminlo)
                              {
                                 data[n] = LONG_MIN;
                                 /* CLIPPING */
                              }
                           else
                              data[n] = -1L - (long) ~((hi << 32) | lo);
                        }
                  }
#else  /* 32-bit unsigned longs */
                  {
                     if ((hi & 0x80000000) == 0)        /* non-negative */
                        {
                           if (hi != 0 || lo > LONG_MAX)
                              {
                                 data[n] = LONG_MAX;
                                 /* CLIPPING */
                              }
                           else
                              data[n] = lo;
                        }
                     else       /* negative */
                        {
                           if (hi != -1UL || lo < LONG_MIN)
                              {
                                 data[n] = LONG_MIN;
                                 /* CLIPPING */
                              }
                           else
                              data[n] = -1L - (long) (0xffffffff ^ lo);
                        }
                  }
#endif
               }

            return n;
         }
         break;
      default:
         return 0;
      }
}


/* Read items of type ULONG */

static int
EdrReadUlong(Ulong *data,
             long length,
             FILE *file,
             int longlong)
{
   long     n;          /* number of items read */
   int              ch;         /* input character */

   switch (longlong)
      {
      case EDR1:
         {
            unsigned long
               item;    /* one input data item */

            for (n = 0; n < length; n++)
               {
                  if ((ch = getc(file)) == EOF)
                     break;
                  item = ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  item = (item << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  item = (item << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  item = (item << 8) | ch;

                  data[n] = item;
               }

            return n;
         }
         break;
      case EDR2:
         {
            unsigned long
               hi, lo;

            for (n = 0; n < length; n++)
               {
                  /* Get high-order 4 bytes */

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = (hi << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = (hi << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  hi = (hi << 8) | ch;

                  /* Get low-order 4 bytes */

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = (lo << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = (lo << 8) | ch;

                  if ((ch = getc(file)) == EOF)
                     break;
                  lo = (lo << 8) | ch;

                  /* Combine the pieces */

#if (ULONG_MAX > 0xffffffffUL)  /* unsigned longs more than 32 bits */
                  {
                     const unsigned long
                        ulmaxhi = ULONG_MAX >> 32,
                        ulmaxlo = ULONG_MAX & 0xffffffffUL;

                     if (hi > ulmaxhi || hi == ulmaxhi && lo > ulmaxlo)
                        {
                           data[n] = ULONG_MAX;
                           /* CLIPPING */
                        }
                     else
                        data[n] = (hi << 32) | lo;
                  }
#else  /* 32-bit unsigned longs */
                  {
                     if (hi != 0)
                        {
                           data[n] = ULONG_MAX;
                           /* CLIPPING */
                        }
                     else
                        data[n] = lo;
                  }
#endif
               }

            return n;
         }
         break;
      default:
         return 0;
      }
}


/* Read items of type SHORT */

static int
EdrReadShort(short  *data,
             long   length,
             FILE   *file)
{
   long     n;          /* number of items read */
   unsigned int    item;        /* one input data item */
   int              ch;         /* input character */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;
         item = ch;

         if ((ch = getc(file)) == EOF)
            break;
         item = (item << 8) | ch;

         data[n] =
            (item & 0x8000)
            ? (-1 - (int) (item ^ 0xffff))
            : item;
      }

   return n;
}


/* Read items of type USHORT */

static int
EdrReadUshort(Ushort *data,
              long length,
              FILE *file)
{
   long     n;          /* number of items read */
   unsigned int    item;        /* one input data item */
   int              ch;         /* input character */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;
         item = ch;

         if ((ch = getc(file)) == EOF)
            break;
         item = (item << 8) | ch;

         data[n] = item;
      }

   return n;
}


/* Read items of type SCHAR */

static int
EdrReadSchar(Schar  *data,
             long   length,
             FILE   *file)
{
   long    n;                   /* number of items read */
   int      ch;                 /* input character */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;

         data[n] = (ch & 0x80) ? ch - 0x100 : ch;
      }

   return n;
}


/* Read items of type UCHAR */

static int
EdrReadUchar(Uchar  *data,
             long   length,
             FILE   *file)
{
   long     n;          /* number of items read */
   int              ch;         /* input character */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;

         data[n] = ch;
      }

   return n;
}


/* Read items of type BOOL */

static int
EdrReadBool(Bool    *data,
            long    length,
            FILE    *file)
{
   long     n;          /* number of items read */
   int              ch;         /* input character */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;

         data[n] = (ch != 0);
      }

   return n;
}


/* Read items of type DOUBLE_COMPLEX */

static int
EdrReadDoubleComplex(DoubleComplex  *data,
                     long           length,
                     FILE           *file)
{
   long     n;          /* number of items read */
   double           x[2];       /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         if (EdrReadDouble(x, 2, file) != 2)
            break;

         data[n].real = x[0];
         data[n].imag = x[1];
      }

   return n;
}


/* Read items of type FLOAT_COMPLEX */

static int
EdrReadFloatComplex(FloatComplex    *data,
                    long            length,
                    FILE            *file)
{
   long     n;          /* number of items read */
   float            x[2];       /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         if (EdrReadFloat(x, 2, file) != 2)
            break;

         data[n].real = x[0];
         data[n].imag = x[1];
      }

   return n;
}


/* Read items of type LONG_COMPLEX */

static int
EdrReadLongComplex(LongComplex  *data,
                   long     length,
                   FILE     *file,
                   int      longlong)
{
   long     n;          /* number of items read */
   long     x[2];       /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         if (EdrReadLong(x, 2, file, longlong) != 2)
            break;

         data[n].real = x[0];
         data[n].imag = x[1];
      }

   return n;
}


/* Read items of type SHORT_COMPLEX */

static int
EdrReadShortComplex(ShortComplex    *data,
                    long            length,
                    FILE            *file)
{
   long     n;          /* number of items read */
   short            x[2];       /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         if (EdrReadShort(x, 2, file) != 2)
            break;

         data[n].real = x[0];
         data[n].imag = x[1];
      }

   return n;
}


/* Read items of type SCHAR_COMPLEX */

static int
EdrReadScharComplex(ScharComplex    *data,
                    long            length,
                    FILE            *file)
{
   long     n;          /* number of items read */
   Schar            x[2];       /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         if (EdrReadSchar(x, 2, file) != 2)
            break;

         data[n].real = x[0];
         data[n].imag = x[1];
      }

   return n;
}


/* Read items of type CHAR */

static int
EdrReadChar(char *data,
            long length,
            FILE *file)
{
   long    n;                   /* number of items read */
   int      ch;                 /* input character */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;

         data[n] = ch;
      }

   return n;
}


/* Read items of type WCHAR */

static int
EdrReadWchar(Wchar      *data,
             long       length,
             FILE       *file)
{
   long     n;          /* number of items read */
   unsigned int    item;        /* one input data item */
   int              ch;         /* input character */

   for (n = 0; n < length; n++)
      {
         if ((ch = getc(file)) == EOF)
            break;
         item = ch;

         if ((ch = getc(file)) == EOF)
            break;
         item = (item << 8) | ch;

         data[n] = item;
      }

   return n;
}


/*
 * *** FUNCTIONS FOR OUTPUT
 */

/*
 * Write "data" member of field to file in EDR format.
 * Return TRUE on success, FALSE on failure.
 */

static int
WriteEdrData(FieldSpec  *field,
             FILE       *file,
             int        longlong)
{
   long    length;              /* number of elements */

   if (file  == NULL || field == NULL || field->type == NO_TYPE)
      return FALSE;

   length = FieldLength(field);

   if (length != 0 && field->data == NULL)
      return FALSE;

   if (EdrWrite(field->data,
                field->type, length, file, longlong) != length)
      return FALSE;

   return TRUE;
}


/*
 * Write field specification to file in EDR format.
 * Return TRUE on success, FALSE on failure.
 */

static int
WriteEdrFieldSpec(FieldSpec *field,
                  FILE      *file,
                  int       longlong)
{
   int     rank;                /* number of dimensions */
   int     i;                   /* loop index */
   long    num_ax_names;        /* number of axis names */

   if (file == NULL || field == NULL)
      return FALSE;

   if (!WriteEdrString(field->name, file, longlong))
      return FALSE;

   if (EdrWriteShort(&field->type, 1, file) != 1)
      return FALSE;

   if (EdrWriteShort(&field->rank, 1, file) != 1)
      return FALSE;

   rank = field->rank;

   if (rank != 0 && field->dim == NULL)
      return FALSE;             /* Inconsistent rank and dimensions. */

   if (EdrWriteLong(field->dim, rank, file, longlong) != rank)
      return FALSE;

   if (!WriteEdrString(field->units, file, longlong))
      return FALSE;

   if (EdrWriteDouble(&field->scale, 1, file) != 1)
      return FALSE;

   if (EdrWriteDouble(&field->offset, 1, file) != 1)
      return FALSE;

   num_ax_names = (field->axis_names == NULL) ? 0 : rank;
   if (EdrWriteLong(&num_ax_names, 1, file, longlong) != 1)
      return FALSE;
   for (i = 0; i < num_ax_names; i++)
      if (!WriteEdrString(field->axis_names[i], file, longlong))
         return FALSE;

   if (EdrWriteShort(&field->occurrence, 1, file) != 1)
      return FALSE;

   if (field->type != NO_TYPE
       && field->occurrence != REQUIRED && field->occurrence != OPTIONAL)
      {
         if (!WriteEdrData(field, file, longlong))
            return FALSE;
      }

   if (!WriteEdrFieldList(field->subfields, file, longlong))
      return FALSE;

   return TRUE;
}


/*
 * Write a string to a file in portable binary format:  the string length
 * in EDR1 or EDR2 LONG format, followed by the characters of the string.
 * Return TRUE on success and FALSE in case of failure.
 */

static int
WriteEdrString(char     *string,
               FILE     *file,
               int      longlong)
{
   long    length;
   long    i;

   if (file == NULL)
      return FALSE;

   if (string == NULL)
      string = "";

   length = strlen(string);

   if (EdrWriteLong(&length, 1, file, longlong) != 1)
      return FALSE;             /* Couldn't write length. */

   for (i = 0; i < length && putc(string[i], file) != EOF; i++)
      { }

   return (i == length);
}


/*
 * Write items of data to a file in EDR1 or EDR2 portable binary format
 * from the block of storage indicated by the pointer "data".
 * The number of items is given by "length", and their data type
 * is indicated by the integer code "type".
 * Return the number of successfully written items.  (A value other
 * than "length" indicates an error).
 */

static int
EdrWrite(void   *data,
         int    type,
         long   length,
         FILE   *file,
         int    longlong)
{
   if (file == NULL || data == NULL || length <= 0)
      return 0;

   switch (type)
      {
      case ARRAY:
         return EdrWriteArray((Array *) data, length, file, longlong);
      case DOUBLE:
         return EdrWriteDouble((double *) data, length, file);
      case FLOAT:
         return EdrWriteFloat((float *) data, length, file);
      case LONG:
         return EdrWriteLong((long *) data, length, file, longlong);
      case ULONG:
         return EdrWriteUlong((Ulong *) data, length, file, longlong);
      case SHORT:
         return EdrWriteShort((short *) data, length, file);
      case USHORT:
         return EdrWriteUshort((Ushort *) data, length, file);
      case SCHAR:
         return EdrWriteSchar((Schar *) data, length, file);
      case UCHAR:
         return EdrWriteUchar((Uchar *) data, length, file);
      case BOOL:
         return EdrWriteBool((Bool *) data, length, file);
      case DOUBLE_COMPLEX:
         return EdrWriteDoubleComplex((DoubleComplex *) data, length, file);
      case FLOAT_COMPLEX:
         return EdrWriteFloatComplex((FloatComplex *) data, length, file);
      case LONG_COMPLEX:
         return EdrWriteLongComplex((LongComplex *) data,
                                    length, file, longlong);
      case SHORT_COMPLEX:
         return EdrWriteShortComplex((ShortComplex *) data, length, file);
      case SCHAR_COMPLEX:
         return EdrWriteScharComplex((ScharComplex *) data, length, file);
      case CHAR:
         return EdrWriteChar((char *) data, length, file);
      case WCHAR:
         return EdrWriteWchar((Wchar *) data, length, file);
      default:    /* Invalid code or NO_TYPE. */
         return 0;
      }
}


/*
 * Write to "file" the EDR1 or EDR2 representation of one item of type
 * ARRAY from the location indicated by "array".
 * Return TRUE for success, FALSE for failure.
 */

static int
WriteEdrArray(Array *array,
              FILE  *file,
              int   longlong)
{
   short   type;
   short   rank;
   long    *dim;
   void    *data;
   long    length;

   if (file == NULL || array == NULL)
      return FALSE;

   type = array->type;
   rank = array->rank;
   dim = array->dim;
   data = array->data;

   if (EdrWriteShort(&type, 1, file) != 1)
      return FALSE;

   if (EdrWriteShort(&rank, 1, file) != 1)
      return FALSE;

   if (rank > 0)
      {
         if (dim == NULL)
            return FALSE;

         if (EdrWriteLong(dim, rank, file, longlong) != rank)
            return FALSE;
      }

   length = LongProd(rank, dim);

   if (length > 0)
      {
         if (data == NULL)
            return FALSE;

         if (EdrWrite(data, type, length, file, longlong) != length)
            return FALSE;
      }

   return TRUE;
}


/*
 * Each of the following functions with names of the form
 *
 *      EdrWrite<type>
 *
 * takes three arguments:
 *      <type>  *data,
 *      long    length,
 *      FILE    *file,
 * and in some cases a fourth:
 *      int     longlong
 * The function writes "length" items of data of the indicated data type
 * in EDR portable binary format to the stream "file" from the block of
 * storage indicated by "data".  The fourth argument, "longlong", is
 * present when the EDR1 and EDR2 representations of the data type differ;
 * a value of 1 indicates EDR1, and 2 indicates EDR2.  The int return
 * value is the number of items successfully written, which will be less
 * than "length" in case of error.
 */

/* Write items of type ARRAY */

static int
EdrWriteArray(Array     *data,
              long      length,
              FILE      *file,
              int       longlong)
{
   long n;

   for (n = 0;
        n < length && WriteEdrArray(&data[n], file, longlong);
        n++)
      { }

   return n;
}


/* Write items of type DOUBLE */

static int
EdrWriteDouble(double   *data,
               long     length,
               FILE     *file)
{
   long     n;          /* number of items written */
   double           item;       /* one data item for output */
   int              sn;         /* set sign bit? */
   int              ex;         /* exponent */
   unsigned long   hi;          /* high-order fraction bits
                                   (28 plus one hidden) */
   unsigned long   lo;          /* 24 low-order fraction bits */
   double           hix, lox;   /* hi and lo as double */

   for (n = 0; n < length; n++)
      {
         item = data[n];

         if (!(item <= DBL_MAX))        /* Inf or NaN */
            /* Don't change to if (item > DBL_MAX)! */
            {
               if (item > DBL_MAX)      /* Inf */
                  {
                     ex = 0x7ff;
                     hi = 0x0000000;
                     lo = 0x000000;
                  }
               else             /* NaN */
                  {
                     ex = 0x7ff;
                     hi = 0xfffffff;
                     lo = 0xffffff;
                  }
            }
         else if (item < -DBL_MAX)      /* -Inf */
            {
               ex = 0xfff;
               hi = 0x0000000;
               lo = 0x000000;
            }
         else
            {
               sn = (item < 0.0);
               item = frexp((sn) ? -item : item, &ex);

               if (item == 0)
                  ex = -1021;
               else if (ex < -1021)
                  {
                     item = ldexp(item, 29 - (-1021 - ex));
                     ex = -1021;
                  }
               else
                  item = ldexp(item, 29);

               lox = modf(item, &hix);

               lo = (unsigned long) (0.5 + ldexp(lox, 24));
               if (lo >= 0x1000000) /* 2^24 */
                  {
                     lo -= 0x1000000; /* 2^24 */
                     hi = 1 + (unsigned long) hix;
                  }
               else
                  hi = (unsigned long) hix;

               if (hi >= 0x20000000) /* 2^29 */
                  {
                     hi = 0x10000000; /* 2^28 */
                     lo = 0;
                     ex += 1;
                  }
               else if (hi < 0x10000000 /* 2^28 */
                        && ex > -1021)
                  {
                     hi = 0x10000000; /* 2^28 */
                     lo = 0;
                  }

               if (ex > 1024)   /* overflow */
                  {
                     ex = 0x7ff;
                     hi = 0x0000000;
                     lo = 0x000000;
                  }
               else if (hi < 0x10000000) /* 2^28 *//* subnormal */
                  ex = 0x000;
               else             /* normalized */
                  {
                     ex += 1022;
                     hi &= 0xfffffff; /* mask off hidden bit */
                  }

               if (sn)
                  ex |= 0x800;
            }

         if (putc(ex >> 4, file) == EOF) /* sign, 7 bits of exponent */
            break;

         if (putc(((ex << 4) & 0xf0) | (hi >> 24),
                  file) == EOF) /* rest of exponent, 4 fraction bits */
            break;

         if (putc((hi >> 16) & 0xff, file) == EOF)
            break;

         if (putc((hi >> 8) & 0xff, file) == EOF)
            break;

         if (putc(hi & 0xff, file) == EOF)
            break;

         if (putc((lo >> 16) & 0xff, file) == EOF) /* start on lo */
            break;

         if (putc((lo >> 8) & 0xff, file) == EOF) /* start on lo */
            break;

         if (putc(lo & 0xff, file) == EOF) /* start on lo */
            break;
      }

   return n;
}


/* Write items of type FLOAT */

static int
EdrWriteFloat(float     *data,
              long      length,
              FILE      *file)
{
   long     n;          /* number of items written */
   float            item;       /* one data item for output */
   int              sn;         /* sign */
   int              ex;         /* exponent */
   unsigned long   fr;          /* 23 fraction bits (and hidden bit) */
   double           frx;        /* fr as double */

   for (n = 0; n < length; n++)
      {
         item = data[n];

         if (!(item <= FLT_MAX))
            /* Don't change to if (item > FLT_MAX)! */
            {
               if (item > DBL_MAX)      /* Inf */
                  {
                     ex = 0x0ff;
                     fr = 0x000000;
                  }
               else             /* NaN */
                  {
                     ex = 0x0ff;
                     fr = 0x7fffff;
                  }
            }
         else if (item < -FLT_MAX)      /* -Inf */
            {
               ex = 0x1ff;
               fr = 0x000000;
            }
         else
            {
               /*! Check for IEEE -0.0 */
               sn = (item < 0.0);
               frx = frexp((sn) ? -item : item, &ex);

               if (frx == 0)
                  ex = -125;
               else if (ex < -125)
                  {
                     frx = ldexp(frx, 24 - (-125 - ex));
                     ex = -125;
                  }
               else
                  frx = ldexp(frx, 24);

               fr = (unsigned long) (0.5 + frx);
               if (fr >= 0x1000000) /* 2^24 */
                  {
                     fr = 0x800000;     /* 2^23 */
                     ex += 1;
                  }
               else if (fr < 0x800000 /* 2^23 */
                        && ex > -125)
                  fr = 0x800000;        /* 2^23 */

               if (ex > 128)    /* overflow */
                  {
                     ex = 0x0ff;
                     fr = 0x000000;
                  }
               else if (fr < 0x800000) /* 2^23 *//* subnormal */
                  ex = 0x000;
               else             /* normalized */
                  {
                     ex += 126;
                     fr &= 0x7fffff;    /* mask off hidden bit */
                  }

               if (sn)
                  ex |= 0x100;
            }

         if (putc(ex >> 1, file) == EOF) /* sign, 7 bits of exponent */
            break;

         if (putc(((ex << 7) & 0x80) | (fr >> 16),
                  file) == EOF)  /* rest of exponent, 7 fraction bits */
            break;

         if (putc((fr >> 8) & 0xff, file) == EOF)
            break;

         if (putc(fr & 0xff, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type LONG */

static int
EdrWriteLong(long       *data,
             long       length,
             FILE       *file,
             int        longlong)
{
   long     n;          /* number of items written */
   long     item;       /* one data item for output */

   /* This clips if native long values fall outside range
    * representable with EDR.
    */
   switch (longlong)
      {
      case EDR1:
         {
            unsigned long   u;

            for (n = 0; n < length; n++)
               {

#if (LONG_MAX > 0x7fffffffL)    /* longs more than 32 bits */

                  item = data[n];

                  if (item > 0x7fffffffL)
                     {
                        u = 0x7fffffffL;
                        /* CLIPPING */
                     }
                  else if (item < -0x80000000L)
                     {
                        u = -0x80000000L;
                        /* CLIPPING */
                     }
                  else
                     u = item;

#else  /* 32-bit longs */

                  u = data[n];

#endif

                  if (putc((u >> 24) & 0xff, file) == EOF)
                     break;

                  if (putc((u >> 16) & 0xff, file) == EOF)
                     break;

                  if (putc((u >> 8) & 0xff, file) == EOF)
                     break;

                  if (putc(u & 0xff, file) == EOF)
                     break;
               }

            return n;
         }
         break;
      case EDR2:
         {
            unsigned long   hi, lo;

            for (n = 0; n < length; n++)
               {
                  item = data[n];

#if (LONG_MAX > 0x7fffffffL)    /* longs more than 32 bits */

                  lo = ((unsigned long) item) & 0xffffffffUL;

                  if (item >= 0)
                     hi = item >> 32;
                  else
                     hi = ~((unsigned long) (-1L - item) >> 32);

#else  /* 32-bit longs */

                  if (item >= 0)
                     hi = 0UL;
                  else
                     hi = 0xffffffffUL;

                  lo = (unsigned long) item;

#endif

                  if (putc((hi >> 24) & 0xff, file) == EOF)
                     break;

                  if (putc((hi >> 16) & 0xff, file) == EOF)
                     break;

                  if (putc((hi >> 8) & 0xff, file) == EOF)
                     break;

                  if (putc(hi & 0xff, file) == EOF)
                     break;

                  if (putc((lo >> 24) & 0xff, file) == EOF)
                     break;

                  if (putc((lo >> 16) & 0xff, file) == EOF)
                     break;

                  if (putc((lo >> 8) & 0xff, file) == EOF)
                     break;

                  if (putc(lo & 0xff, file) == EOF)
                     break;
               }

            return n;
         }
         break;
      default:
         return 0;
      }
}


/* Write items of type ULONG */

static int
EdrWriteUlong(Ulong *data,
              long length,
              FILE *file,
              int longlong)
{
   long     n;          /* number of items written */

   /* This clips if native unsigned long values fall outside range
    * representable with EDR.
    */
   switch (longlong)
      {
      case EDR1:
         {
            unsigned long   u;

            for (n = 0; n < length; n++)
               {
                  u = data[n];

#if (ULONG_MAX > 0xffffffffUL)  /* unsigned longs more than 32 bits */

                  if (u > 0xffffffffL)
                     {
                        u = 0xffffffffL;
                        /* CLIPPING */
                     }

#endif

                  if (putc((u >> 24) & 0xff, file) == EOF)
                     break;

                  if (putc((u >> 16) & 0xff, file) == EOF)
                     break;

                  if (putc((u >> 8) & 0xff, file) == EOF)
                     break;

                  if (putc(u & 0xff, file) == EOF)
                     break;
               }

            return n;
         }
         break;
      case EDR2:
         {
            unsigned long   hi, lo;

            for (n = 0; n < length; n++)
               {
                  lo = data[n];

#if (ULONG_MAX > 0xffffffffUL)  /* unsigned longs more than 32 bits */

                  hi = lo >> 32;
                  lo &= 0xffffffffUL;

#else  /* 32-bit unsigned longs */

                  hi = 0UL;
#endif

                  if (putc((hi >> 24) & 0xff, file) == EOF)
                     break;

                  if (putc((hi >> 16) & 0xff, file) == EOF)
                     break;

                  if (putc((hi >> 8) & 0xff, file) == EOF)
                     break;

                  if (putc(hi & 0xff, file) == EOF)
                     break;

                  if (putc((lo >> 24) & 0xff, file) == EOF)
                     break;

                  if (putc((lo >> 16) & 0xff, file) == EOF)
                     break;

                  if (putc((lo >> 8) & 0xff, file) == EOF)
                     break;

                  if (putc(lo & 0xff, file) == EOF)
                     break;
               }

            return n;
         }
         break;
      default:
         return 0;
      }
}


/* Write items of type SHORT */

static int
EdrWriteShort(short     *data,
              long      length,
              FILE      *file)
{
   long     n;          /* number of items written */
   long     item;       /* one data item for output */
   unsigned int    u;           /* item as unsigned int */

   /* This clips if native short values fall outside range
    * representable with EDR.
    */

   for (n = 0; n < length; n++)
      {
         item = data[n];

         if (item < - 0x8000L)
            u = 0x8000U;
         else if (item > 0x7fffL)
            u = 0x7fffU;
         else
            u = (unsigned int) item;

         if (putc((u >> 8) & 0xff, file) == EOF)
            break;

         if (putc(u & 0xff, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type USHORT */

static int
EdrWriteUshort(Ushort *data,
               long length,
               FILE *file)
{
   long     n;          /* number of items written */
   Ushort           u;          /* one data item for output */

   /* This clips if native short values fall outside range
    * representable with EDR.
    */

   for (n = 0; n < length; n++)
      {
         u = data[n];

         /* if (u > 0xffffU)
            u = 0xffffU; */

         if (putc((u >> 8) & 0xff, file) == EOF)
            break;

         if (putc(u & 0xff, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type SCHAR */

static int
EdrWriteSchar(Schar     *data,
              long      length,
              FILE      *file)
{
   long    n;                   /* number of items written */

   for (n = 0; n < length; n++)
      {
         if (putc(data[n], file) == EOF)
            break;
      }

   return n;
}


/* Write items of type UCHAR */

static int
EdrWriteUchar(Uchar     *data,
              long      length,
              FILE      *file)
{
   long    n;                   /* number of items written */

   for (n = 0; n < length; n++)
      {
         if (putc(data[n], file) == EOF)
            break;
      }

   return n;
}


/* Write items of type BOOL */

static int
EdrWriteBool(Bool   *data,
             long   length,
             FILE   *file)
{
   long    n;                   /* number of items written */

   for (n = 0; n < length; n++)
      {
         if (putc(data[n] != 0, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type DOUBLE_COMPLEX */

static int
EdrWriteDoubleComplex(DoubleComplex *data,
                      long          length,
                      FILE          *file)
{
   long    n;                   /* number of items written */

   double  x[2];                /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         x[0] = data[n].real;
         x[1] = data[n].imag;

         if (EdrWriteDouble(x, 2, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type FLOAT_COMPLEX */

static int
EdrWriteFloatComplex(FloatComplex   *data,
                     long           length,
                     FILE           *file)
{
   long    n;                   /* number of items written */
   float  x[2];         /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         x[0] = data[n].real;
         x[1] = data[n].imag;

         if (EdrWriteFloat(x, 2, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type LONG_COMPLEX */

static int
EdrWriteLongComplex(LongComplex *data,
                    long        length,
                    FILE        *file,
                    int         longlong)
{
   long    n;                   /* number of items written */
   long    x[2];                /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         x[0] = data[n].real;
         x[1] = data[n].imag;

         if (EdrWriteLong(x, 2, file, longlong) == EOF)
            break;
      }

   return n;
}


/* Write items of type SHORT_COMPLEX */

static int
EdrWriteShortComplex(ShortComplex   *data,
                     long           length,
                     FILE           *file)
{
   long    n;                   /* number of items written */
   short   x[2];                /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         x[0] = data[n].real;
         x[1] = data[n].imag;

         if (EdrWriteShort(x, 2, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type SCHAR_COMPLEX */

static int
EdrWriteScharComplex(ScharComplex   *data,
                     long           length,
                     FILE           *file)
{
   long    n;                   /* number of items written */
   Schar   x[2];                /* real and imaginary parts */

   for (n = 0; n < length; n++)
      {
         x[0] = data[n].real;
         x[1] = data[n].imag;

         if (EdrWriteSchar(x, 2, file) == EOF)
            break;
      }

   return n;
}


/* Write items of type CHAR */

static int
EdrWriteChar(char *data,
             long length,
             FILE *file)
{
   long    n;                   /* number of items written */

   for (n = 0; n < length; n++)
      {
         if (putc(data[n], file) == EOF)
            break;
      }

   return n;
}


/* Write items of type WCHAR */

static int
EdrWriteWchar(Wchar     *data,
              long      length,
              FILE      *file)
{
   long     n;          /* number of items written */
   Wchar            u;          /* one data item for output */

   for (n = 0; n < length; n++)
      {
         u = data[n];

         /* if (u > 0xffffU)
            u = 0xffffU; */

         if (putc((u >> 8) & 0xff, file) == EOF)
            break;

         if (putc(u & 0xff, file) == EOF)
            break;
      }

   return n;
}


