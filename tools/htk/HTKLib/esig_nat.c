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
 * native binary I/O.
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

static int          ReadNativeData(FieldSpec *field, FILE *file);
static int          NativeRead(void *data,
                               int type, long length, FILE *file);
static int          ReadNativeArray(Array *array, FILE *file);
static FieldSpec    *ReadNativeFieldSpec(FILE *file);
static int          ReadNativeString(char **string, FILE *file);

/* Functions for output */

static int          WriteNativeData(FieldSpec *field, FILE *file);
static int          NativeWrite(void *data,
                                int type, long length, FILE *file);
static int          WriteNativeArray(Array *array, FILE *file);
static int          WriteNativeFieldSpec(FieldSpec *field, FILE *file);
static int          WriteNativeString(char *string, FILE *file);

/*
 * PUBLIC FUNCTION DEFINITIONS
 * - NativeTypeSize
 * - NativeRecordSize
 * - ReadNativeFieldList
 * - ReadNativeRecord
 * - ReadNativeSamples
 * - WriteNativeFieldList
 * - WriteNativeRecord
 * - WriteNativeSamples
 */

/*
 * Size in bytes of native binary external representation of data type.
 */

long
NativeTypeSize(int type   /* numeric data-type code */ )
{
   if (type == ARRAY)
      return -1;  /* Variable-length external representation. */
   else
      return InternTypeSize(type);
}


/*
 * Return the size, in bytes, of one record in native binary format,
 * according to the field specs on a field list.
 * Return 0 on NULL input; -1 is a code for variable-length records.
 */

long
NativeRecordSize(FieldList list)
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
            * NativeTypeSize(fld_order[i]->type);

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
 * Read field list in native binary format from file.
 * Return TRUE on success, FALSE on failure.
 */

int
ReadNativeFieldList(FieldList *listp, /* output variable */
                    FILE      *file) /* input file */
{
   FieldList    list;
   long num_fields;
   long i;
   FieldSpec    *spec;


   if (fread(&num_fields, sizeof(long), 1, file) != 1)
      {
         DebugMsg(1, "ReadNativeFieldList: Couldn't get number of fields.");
         return FALSE;
      }

   list = NULL;

   for (i = 0; i < num_fields; i++)
      {
         spec = ReadNativeFieldSpec(file);
         if (spec == NULL || !AddField(&list, spec))
            {
               if (list != NULL)
                  FreeFieldList(list);
               DebugMsg(2, ((spec == NULL)
                            ? "ReadNativeFieldList: couldn't read field spec."
                            : ("ReadNativeFieldList: "
                               "couldn't add field spec to list.")));
               return FALSE;
            }
      }

   *listp = list;

   return TRUE;
}


/*
 * Read one record in native binary format from file into the "data"
 * members of the field specs on a NULL-terminated linear array
 * of REQUIRED and OPTIONAL fields, like those produced by TypeOrder
 * and FieldOrder.  Return TRUE on success, FALSE on failure.
 */

int
ReadNativeRecord(FieldSpec  **fields,
                 FILE       *file)
{
   long    i;
   long    nopt;
   Uchar   flags;

   if (file == NULL || fields == NULL)
      {
         DebugMsg(1, "ReadNativeRecord: NULL argument.");
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
                     if (fread(&flags, 1, 1, file) != 1)
                        {
                           DebugMsg(1, ("ReadNativeRecord: can't read "
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
            if (!ReadNativeData(fields[i], file))
               {
                  DebugMsg(1, "ReadNativeRecord: couldn't read field data.");
                  return FALSE;
               }
         }

   return TRUE;
}


/*
 * Read nrec one-field records in native binary format from file into
 * the array indicated by data.  The field is specified by fields,
 * a NULL-terminated linear array like those produced by TypeOrder
 * and FieldOrder and containing exactly one REQUIRED field.
 * Return the number of complete records read.
 */

long
ReadNativeSamples(void      *data,
                  long      nrec,
                  FieldSpec **fields,
                  FILE      *file)
{
   int          type;
   long length;

   if (data == NULL)
      {
         DebugMsg(1, "ReadNativeSamples: NULL data pointer.");
         return 0;
      }
   if (nrec < 0)
      {
         DebugMsg(1, ("ReadNativeSamples: "
                      "negative number of records specified."));
         return 0;
      }
   if (fields == NULL || fields[0] == NULL || fields[1] != NULL
       || fields[0]->occurrence != REQUIRED)
      {
         DebugMsg(1, "ReadNativeSamples: bad \"fields\" array.");
         return 0;
      }
   if (file == NULL)
      {
         DebugMsg(1, "ReadNativeSamples: NULL file pointer.");
         return 0;
      }

   type = fields[0]->type;

   if (type == NO_TYPE)
      return 0;

   length = FieldLength(fields[0]);

   if (nrec*length == 0)
      return nrec;

   return NativeRead(data, type, nrec*length, file) / length;
}


/*
 * Write field list to file in native binary format.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteNativeFieldList(FieldList list,
                     FILE      *file)
{
   long    num_fields;          /* number of fields */
   long    i;                   /* loop index */

   if (file == NULL)
      {
         DebugMsg(1, "WriteNativeFieldList: NULL file pointer.");
         return FALSE;
      }

   num_fields = FieldListLength(list);

   if (fwrite(&num_fields, sizeof(long), 1, file) != 1)
      {
         DebugMsg(1, "WriteNativeFieldList: couldn't write number of fields.");
         return FALSE;
      }

   for (i = 0; i < num_fields; i++)
      if (!WriteNativeFieldSpec(list[i], file))
         {
            DebugMsg(2, "WriteNativeFieldList: couldn't write field spec.");
            return FALSE;
         }

   return TRUE;         /* Success. */
}


/*
 * Write to file one record in native binary format, consisting of the
 * contenta of the "data" members of the field specs on a NULL-terminated
 * linear array of REQUIRED and OPTIONAL fields, like those produced by
 * TypeOrder and FieldOrder.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteNativeRecord(FieldSpec **fields,
                  FILE      *file)
{
   long    i;
   long    nopt;
   Uchar   flags;

   if (file == NULL || fields == NULL)
      {
         DebugMsg(1, "WriteNativeRecord: NULL argument.");
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
                     if (fwrite(&flags, 1, 1, file) != 1)
                        {
                           DebugMsg(1, ("WriteNativeRecord:  couldn't write "
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

         if (fwrite(&flags, 1, 1, file) != 1)
            {
               DebugMsg(1, ("WriteNativeRecord: couldn't write "
                            "\"presence\" flags for OPTIONAL fields."));
               return FALSE;
            }
      }

   for (i = 0; fields[i] != NULL; i++)
      {
         if (fields[i]->occurrence == REQUIRED || fields[i]->present)
            {
               if (!WriteNativeData(fields[i], file))
                  {
                     DebugMsg(1, "WriteNativeRecord: couldn't write field data.");
                     return FALSE;
                  }
            }
      }

   return TRUE;
}


/*
 * Write nrec one-field records in native binary format to file from the
 * array indicated by data.  The The field is specified by fields, a
 * NULL-terminated linear array like those produced by TypeOrder
 * and FieldOrder and containing exactly one REQUIRED field.
 * Return the number of complete records written.
 */

long
WriteNativeSamples(void     *data,
                   long     nrec,
                   FieldSpec **fields,
                   FILE     *file)
{
   int          type;
   long length;

   if (data == NULL)
      {
         DebugMsg(1, "WriteNativeSamples: NULL data pointer.");
         return 0;
      }
   if (nrec < 0)
      {
         DebugMsg(1, ("WriteNativeSamples: "
                      "negative number of records specified."));
         return 0;
      }
   if (fields == NULL || fields[0] == NULL || fields[1] != NULL
       || fields[0]->occurrence != REQUIRED)
      {
         DebugMsg(1, "WriteNativeSamples: bad \"fields\" array.");
         return 0;
      }
   if (file == NULL)
      {
         DebugMsg(1, "WriteNativeSamples: NULL file pointer.");
         return 0;
      }

   type = fields[0]->type;

   if (type == NO_TYPE)
      return 0;

   length = FieldLength(fields[0]);

   if (nrec*length == 0)
      return nrec;

   return NativeWrite(data, type, nrec*length, file) / length;
}


/*
 * LOCAL FUNCTION DEFINITIONS
 */

/*
 * *** FUNCTIONS FOR INPUT
 */

/*
 * Read "data" member of field from file in native binary format;
 * allocate if NULL.  Return TRUE on success, FALSE on failure.
 */

static int
ReadNativeData(FieldSpec *field,
               FILE *file)
{
   long    size;                /* size of element (bytes) */
   long    length;              /* number of elements */

   if (file  == NULL || field == NULL || field->type == NO_TYPE)
      {
         DebugMsg(1, "ReadNativeData: NULL argument or type NO_TYPE.");
         return FALSE;
      }

   size = InternTypeSize(field->type);
   length = FieldLength(field);

   if (field->data == NULL && length != 0)
      {
         field->data = malloc(length * size);
         if (field->data == NULL)
            {
               DebugMsg(1, "ReadNativeData: allocation failure.");
               return FALSE;
            }
      }

   return (NativeRead(field->data, field->type, length, file) == length);
}


/*
 * Read items of data from a file into a block of storage indicated
 * by a pointer "data".  The number of items is given by "length",
 * and their data type is indicated by the integer code "type".
 * Return the number of successfully read items.  (A value other
 * than "length" indicates an error).
 */

static int
NativeRead(void *data,
           int  type,
           long length,
           FILE *file)
{
   if (file == NULL || data == NULL || length <= 0)
      return 0;

   if (type == ARRAY)
      {
         Array  *array = (Array *) data;
         long   n;

         for (n = 0;
              n < length && ReadNativeArray(&array[n], file);
              n++)
            { }

         return n;
      }
   else
      {
         long   size;           /* size of element (bytes) */

         size = InternTypeSize(type);
         return fread(data, size, length, file);
      }
}


/*
 * Read from file the native-mode representation of one item of type
 * ARRAY.  Store the type, rank, dimensions, and data in the members of
 * the array structure, allocating the necessary space for dimensions
 * and data.  Return TRUE for success, FALSE for failure.
 */

static int
ReadNativeArray(Array *array,
                FILE *file)
{
   short   type, rank;
   long    *dim;
   long    length;
   long    size;
   void    *data;

   if (array == NULL || file == NULL)
      return FALSE;

   /* Read type, rank. */

   if (fread(&type, sizeof(short), 1, file) != 1)
      return FALSE;             /* Couldn't get type. */

   if (fread(&rank, sizeof(short), 1, file) != 1)
      return FALSE;             /* Couldn't get rank. */

   if (rank == 0)
      dim = NULL;
   else
      {
         dim = (long *) malloc(rank * sizeof(long));
         if (dim == NULL)
            return FALSE;       /* Allocation failure. */

         if (fread(dim, sizeof(long), rank, file) != rank)
            return FALSE;       /* Couldn't get dimensions. */
      }

   length = LongProd(rank, dim);
   size = InternTypeSize(type);

   if (length == 0)
      data = NULL;
   else
      {
         data = malloc(length*size);

         if (data == NULL)
            return FALSE;       /* Allocation failure. */

         if (NativeRead(data, type, length, file) != length)
            return FALSE;
      }

   array->type = type;
   array->rank = rank;
   array->dim = dim;
   array->data = data;

   return TRUE;
}


/*
 * Read field specification in native binary format from file
 * and return pointer to it; return NULL on failure.
 */

static FieldSpec *
ReadNativeFieldSpec(FILE *file)
{
   char *name;          /* field name */
   short        type;           /* data type code */
   short        rank;           /* number of dimensions */
   int          i;              /* loop index */
   FieldSpec    *field;         /* field spec being read */
   long num_ax_names;   /* number of axis names */

   /* Read name, type, rank. */

   if (!ReadNativeString(&name, file))
      return NULL;              /* Failure getting field name. */

   if (fread(&type, sizeof(short), 1, file) != 1)
      return NULL;              /* Couldn't get type. */

   if (fread(&rank, sizeof(short), 1, file) != 1)
      return NULL;              /* Couldn't get rank. */

   /* Allocate structure. */

   field = NewFieldSpec(type, rank);
   if (field == NULL)
      return NULL;              /* Couldn't create field spec. */

   field->name = name;

   /* Read dimensions. */

   if (rank != 0 && field->dim == NULL)
      return NULL;              /* Inconsistent rank and dimensions. */

   if (fread(field->dim, sizeof(long), rank, file) != rank)
      return NULL;              /* Couldn't get dimensions. */

   /* Read units, scale, offset, axis_names. */

   if (!ReadNativeString(&field->units, file))
      return NULL;

   if (fread(&field->scale, sizeof(double), 1, file) != 1)
      return NULL;              /* Couldn't get scale. */

   if (fread(&field->offset, sizeof(double), 1, file) != 1)
      return NULL;              /* Couldn't get offset. */

   if (fread(&num_ax_names, sizeof(long), 1, file) != 1)
      return NULL;              /* Couldn't get number of axis names. */

   if (num_ax_names > rank)
      return NULL;              /* Bad value for num_ax_names. */

   if (num_ax_names != 0)
      {
         field->axis_names = (char **) malloc(rank * sizeof(char *));
         if (field->axis_names == NULL)
            return NULL;        /* Allocation failure. */

         for (i = 0; i < num_ax_names; i++)
            ReadNativeString(&field->axis_names[i], file);

         for ( ; i < rank; i++)
            field->axis_names[i] = NULL;
      }

   /* Read occurrence class. */

   if (fread(&field->occurrence, sizeof(short), 1, file) != 1)
      return NULL;              /* Couldn't get occurrence code. */

   /* Read data if required. */

   if (type != NO_TYPE
       && field->occurrence != REQUIRED && field->occurrence != OPTIONAL)
      {
         if (!ReadNativeData(field, file))
            return NULL;        /* Failure reading data. */
      }

   /* Read subfield list. */

   if (!ReadNativeFieldList(&field->subfields, file))
      {
         FreeFieldSpec(field);
         return NULL;           /* Failure getting subfields. */
      }

   return field;                /* Success. */
}


/*
 * Read character string in native binary format from file and
 * assign a pointer to it via output variable.  Return TRUE on
 * success, FALSE on failure.
 */

static int
ReadNativeString(char **string, FILE *file)
{
   long    length;
   char    *str;

   if (fread(&length, sizeof(long), 1, file) != 1)
      return FALSE;             /* Couldn't get length. */

   if (length == 0)
      str = NULL;
   else
      {
         str = (char *) malloc(length + 1);
         if (str ==  NULL)
            return FALSE;

         if (fread(str, 1, length, file) != length)
            {
               free(str);
               return FALSE;
            }

         str[length] = '\0';
      }

   if (string != NULL)
      *string = str;

   return TRUE;
}


/*
 * *** FUNCTIONS FOR OUTPUT
 */

/*
 * Write "data" member of field to file in native binary format.
 * Return TRUE on success, FALSE on failure.
 */

static int
WriteNativeData(FieldSpec *field,
                FILE      *file)
{
   long    length;              /* number of elements */

   if (file  == NULL || field == NULL || field->type == NO_TYPE)
      return FALSE;

   length = FieldLength(field);

   if (length != 0 && field->data == NULL)
      return FALSE;

   return (NativeWrite(field->data, field->type, length, file) == length);
}


/*
 * Write items of data to a file in native binary format from
 * the block of storage indicated by the pointer "data".
 * The number of items is given by "length", and their data type
 * is indicated by the integer code "type".
 * Return the number of successfully written items.  (A value other
 * than "length" indicates an error).
 */

static int
NativeWrite(void    *data,
            int     type,
            long    length,
            FILE    *file)
{
   if (file == NULL || data == NULL || length <= 0)
      return 0;

   if (type == ARRAY)
      {
         Array  *array = (Array *) data;
         long   n;

         for (n = 0;
              n < length && WriteNativeArray(&array[n], file);
              n++)
            { }

         return n;
      }
   else
      {
         long   size;           /* size of element (bytes) */

         size = InternTypeSize(type);
         return fwrite(data, size, length, file);
      }
}


/*
 * Write to "file" the native binary representation of one item of type
 * ARRAY from the location indicated by "array".
 * Return TRUE for success, FALSE for failure.
 * *
 */

static int
WriteNativeArray(Array *array,
                 FILE  *file)
{
   short   type, rank;
   long    *dim;
   void    *data;
   long    length;

   if (array == NULL || file == NULL)
      return FALSE;

   type = array->type;
   rank = array->rank;
   dim = array->dim;
   data = array->data;

   if (fwrite(&type, sizeof(short), 1, file) != 1)
      return FALSE;

   if (fwrite(&rank, sizeof(short), 1, file) != 1)
      return FALSE;

   if (rank > 0)
      {
         if (dim == NULL)
            return FALSE;

         if (fwrite(dim, sizeof(long), rank, file) != rank)
            return FALSE;
      }

   length = LongProd(rank, dim);

   if (length > 0)
      {
         if (data == NULL)
            return FALSE;

         if (NativeWrite(data, type, length, file) != length)
            return FALSE;
      }

   return TRUE;
}


/*
 * Write field specification to file in native binary format.
 * Return TRUE on success, FALSE on failure.
 */

static int
WriteNativeFieldSpec(FieldSpec  *field,
                     FILE       *file)
{
   int     rank;                /* number of dimensions */
   int     i;                   /* loop index */
   long    num_ax_names;        /* number of axis names */

   if (file == NULL || field == NULL)
      return FALSE;

   if (!WriteNativeString(field->name, file))
      return FALSE;

   if (fwrite(&field->type, sizeof(short), 1, file) != 1)
      return FALSE;

   if (fwrite(&field->rank, sizeof(short), 1, file) != 1)
      return FALSE;

   rank = field->rank;

   if (rank != 0 && field->dim == NULL)
      return FALSE;             /* Inconsistent rank and dimensions. */

   if (fwrite(field->dim, sizeof(long), rank, file) != rank)
      return FALSE;

   if (!WriteNativeString(field->units, file))
      return FALSE;

   if (fwrite(&field->scale, sizeof(double), 1, file) != 1)
      return FALSE;

   if (fwrite(&field->offset, sizeof(double), 1, file) != 1)
      return FALSE;

   num_ax_names = (field->axis_names == NULL) ? 0 : rank;
   if (fwrite(&num_ax_names, sizeof(long), 1, file) != 1)
      return FALSE;
   for (i = 0; i < num_ax_names; i++)
      if (!WriteNativeString(field->axis_names[i], file))
         return FALSE;

   if (fwrite(&field->occurrence, sizeof(short), 1, file) != 1)
      return FALSE;

   if (field->type != NO_TYPE
       && field->occurrence != REQUIRED && field->occurrence != OPTIONAL)
      {
         if (!WriteNativeData(field, file))
            return FALSE;
      }

   if (!WriteNativeFieldList(field->subfields, file))
      return FALSE;

   return TRUE;
}


/*
 * Write character string in native binary format to a file.
 *  Return TRUE on success, FALSE on failure.
 */

static int
WriteNativeString(char *string,
                  FILE *file)
{
   long    length;

   if (file == NULL)
      return FALSE;

   if (string == NULL)
      string = "";

   length = strlen(string);

   if (fwrite(&length, sizeof(long), 1, file) != 1)
      return FALSE;             /* Couldn't write length. */

   if (fwrite(string, 1, length, file) != length)
      return FALSE;

   return TRUE;
}
