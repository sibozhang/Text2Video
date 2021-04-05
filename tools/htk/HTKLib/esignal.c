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
 * Utility; field specifications and lists; general I/O.
 *
 * Author:  Rod Johnson
 */


#include "esignal.h"

/*
 *  LOCAL FUNCTION DECLARATIONS
 */

/* Used by FieldListLength, FieldOrder, and TypeOrder. */

static int          FieldArrayLength(FieldSpec **fields);

/* Used by FindField */

static FieldSpec    *GetField(FieldList list, char *name);
static char         *FirstComponent(char *name);

/* Used by FieldOrder */

static FieldSpec    **SubFieldOrder(FieldList list, char *prefix);

/* Used by ReadPreamble */

static int          GetLine(char *buf, int len, FILE *file);
static int          GetLong(long *val, int len, FILE *file);

/* Used by GetFieldOrdering */

static int  LongVal(void *src, int type, long *dest);


/*
 * GLOBAL VARIABLES
 */

static short    Types[] =
{
   NO_TYPE,
   ARRAY,
   DOUBLE,
   FLOAT,
   LONG,
   ULONG,
   SHORT,
   USHORT,
   SCHAR,
   UCHAR,
   BOOL,
   DOUBLE_COMPLEX,
   FLOAT_COMPLEX,
   LONG_COMPLEX,
   SHORT_COMPLEX,
   SCHAR_COMPLEX,
   CHAR,
   WCHAR
};

int     DebugMsgLevel;
void    (*DebugMsgFunc)(char *msg);

/*
 * PUBLIC FUNCTION DEFINITIONS
 * *** Miscellaneous
 * *** Debug output
 * *** Types
 * *** Field specs and field lists
 * *** General input/output
 */

/*
 * *** MISCELLANEOUS
 * - StrDup
 * - LongProd
 */

/*
 * Replacement for non-ANSI function strdup.
 */

char *
StrDup(char *str)
{
   char    *cpy;

   if (str == NULL)
      return NULL;

   cpy = (char *) malloc(1 + strlen(str));

   if (cpy == NULL)
      return NULL;

   return strcpy(cpy, str);
}


/*
 * Product of array of longs (e.g. array length from dimensions).
 */

long
LongProd(int     n,
         long    *arr)
{
   int     i;
   long    prod;

   prod = 1;

   for (i = 0; i < n; i++)
      prod *= arr[i];

   return prod;
}


/*
 * *** DEBUG OUTPUT
 * - DebugPrint
 */

void
DebugPrint(char *msg)
{
   fprintf(stderr, "%s\n", msg);
}


/*
 * *** TYPES
 * - ValidType
 * - TypeName
 * - TypeCode
 * - InternTypeSize
 * - ExternTypeSize
 * - 
 */

/*
 * Does type code denote either NO_TYPE or supported data
 * type?
 */

int
ValidType(int type    /* numeric data-type code */ )
{
   switch (type)
      {
         /* All non-default cases fall through. */
      case NO_TYPE:
      case ARRAY:
      case DOUBLE:
      case FLOAT:
      case LONG:
      case ULONG:
      case SHORT:
      case USHORT:
      case SCHAR:
      case UCHAR:
      case BOOL:
      case DOUBLE_COMPLEX:
      case FLOAT_COMPLEX:
      case LONG_COMPLEX:
      case SHORT_COMPLEX:
      case SCHAR_COMPLEX:
      case CHAR:
      case WCHAR:
         return TRUE;
      default:
         return FALSE;
      }
}


/*
 * String containing name of data type.
 */

char *
TypeName(int type    /* numeric data-type code */ )
{
   switch (type)
      {
      case NO_TYPE:
         return "NO_TYPE";
      case ARRAY:
         return "ARRAY";
      case DOUBLE:
         return "DOUBLE";
      case FLOAT:
         return "FLOAT";
      case LONG:
         return "LONG";
      case ULONG:
         return "ULONG";
      case SHORT:
         return "SHORT";
      case USHORT:
         return "USHORT";
      case SCHAR:
         return "SCHAR";
      case UCHAR:
         return "UCHAR";
      case BOOL:
         return "BOOL";
      case DOUBLE_COMPLEX:
         return "DOUBLE_COMPLEX";
      case FLOAT_COMPLEX:
         return "FLOAT_COMPLEX";
      case LONG_COMPLEX:
         return "LONG_COMPLEX";
      case SHORT_COMPLEX:
         return "SHORT_COMPLEX";
      case SCHAR_COMPLEX:
         return "SCHAR_COMPLEX";
      case CHAR:
         return "CHAR";
      case WCHAR:
         return "WCHAR";
      default:    /* Invalid code. */
         return NULL;
      }
}


/*
 * Return data-type code for type with a given name.
 */

int
TypeCode(char *name)
{
   int     num_types = sizeof(Types)/sizeof(Types[0]);
   int     i;
   char    *code;

   if (name == NULL)
      return 0;

   for (i = 0; i < num_types; i++)
      {
         code = TypeName(Types[i]);
         if (code != NULL && strcmp(code, name) == 0)
            return Types[i];
      }

   return 0;
}


/*
 * Number of bytes of memory to allocate for an item of a given type.
 */

long
InternTypeSize(int type         /* numeric data_type code */ )
{
   switch (type)
      {
      case ARRAY:
         return sizeof(Array);
      case DOUBLE:
         return sizeof(double);
      case FLOAT:
         return sizeof(float);
      case LONG:
      case ULONG:
         return sizeof(long);
      case SHORT:
      case USHORT:
         return sizeof(short);
      case SCHAR:
         return sizeof(Schar);
      case UCHAR:
         return sizeof(Uchar);
      case BOOL:
         return sizeof(Bool);
      case DOUBLE_COMPLEX:
         return sizeof(DoubleComplex);
      case FLOAT_COMPLEX:
         return sizeof(FloatComplex);
      case LONG_COMPLEX:
         return sizeof(LongComplex);
      case SHORT_COMPLEX:
         return sizeof(ShortComplex);
      case SCHAR_COMPLEX:
         return sizeof(ScharComplex);
      case CHAR:
         return sizeof(char);
      case WCHAR:
         return sizeof(Wchar);
      default:    /* Invalid code or NO_TYPE. */
         return 0;
      }
}


/*
 * Number of bytes of external storage occupied by an item of a given type
 * in a file with a given format.
 */

long
ExternTypeSize(int type,
               int arch)
{
   switch (arch)
      {
      case EDR1:
         return EdrTypeSize(type, EDR1);
         break;
      case EDR2:
         return EdrTypeSize(type, EDR2);
         break;
      case NATIVE:
         return NativeTypeSize(type);
         break;
      case ASCII:
         return -1;
         break;
      default:
         return -1;
         break;
      }
}


/*
 * *** FIELD SPECS AND FIELD LISTS
 * - NewFieldSpec
 * - FreeFieldSpec
 * - FreeFieldList
 * - FreeAxisNames
 * - FieldLength
 * - FieldListLength
 * - FindField
 * - AddField
 * - AddSubfield
 * - FieldOrder
 * - TypeOrder
 */

/*
 * Create a new FieldSpec structure.
 */

FieldSpec *
NewFieldSpec(int    type,       /* numeric code for data type */
             int    rank)       /* number of dimensions */
{
   FieldSpec    *field;         /* new FieldSpec structure */

   /* Check for bad arguments. */

   if (!ValidType(type) || rank < 0)
      return NULL;

   /* Allocate new struct. */

   field = (FieldSpec *) malloc(sizeof(*field));
   if (field == NULL)
      return NULL;

   /* Fill in default values for members. */

   field->type = type;
   field->rank = rank;

   if (type == NO_TYPE || rank == 0)
      field->dim = NULL;
   else
      {
         /* Allocate dimension vector according to rank. */

         field->dim = (long *) malloc(rank * sizeof(long));
         if (field->dim == NULL)
            {
               free(field);
               return NULL;
            }
      }

   field->occurrence = REQUIRED;
   field->name = NULL;
   field->subfields = NULL;
   field->units = NULL;
   field->scale = 1.0;
   field->offset = 0.0;
   field->axis_names = NULL;
   field->data = NULL;
   field->present = TRUE;
   field->fullname = NULL;

   return field;
}


/*
 * Free storage for a field specification, not including the subfield
 * list.
 */

void
FreeFieldSpec(FieldSpec *spec)
{
   int     i;

   if (spec == NULL)
      return;

   if (spec->name != NULL)
      free(spec->name);

   if (spec->dim != NULL)
      free(spec->dim);

   if (spec->units != NULL)
      free(spec->units);

   if (spec->axis_names != NULL)
      {
         for (i = 0; i < spec->rank; i++)
            if (spec->axis_names[i] != NULL)
               free(spec->axis_names[i]);
         free(spec->axis_names);
      }

   if (spec->data != NULL)
      free(spec->data);

   if (spec->fullname != NULL)
      free(spec->fullname);

   free(spec);
}


/*
 * Free storage for a field list, including all field specifications,
 * and (recursively) their subfield lists.
 */

void
FreeFieldList(FieldList list)
{
   int     i;

   if (list == NULL)
      return;

   for (i = 0; list[i] != NULL; i++)
      {
         FreeFieldList(list[i]->subfields);
         FreeFieldSpec(list[i]);
      }

   free(list);
}


/*
 * Free all non-NULL elements of a string array and the array itself.
 */

void
FreeAxisNames(char  **axis_names,
              int   rank)
{
   int     i;

   if (axis_names != NULL)
      {
         for (i = 0; i < rank; i++)
            if (axis_names[i] != NULL)
               free(axis_names[i]);
         free(axis_names);
      }

}


/*
 * Number of array elements of a field.  Zero in case of error.
 */

long
FieldLength(FieldSpec *field)
{
   if (field == NULL
       || (field->rank != 0 && field->dim == NULL)
       || field->type == NO_TYPE)
      {
         return 0;
      }

   return LongProd(field->rank, field->dim);
}


/*
 * Number of top-level fields on a field list.
 */

int
FieldListLength(FieldList list)
{
   /* This depends on field lists being implemented as
    * NULL-terminated arrays of pointers. */

   return FieldArrayLength(list);
}


/*
 * Find field specification with given full name on given
 * field list (including tree of subfields).
 */

FieldSpec *
FindField(FieldList list,       /* field list */
          char       *name)     /* full name of field */
{
   char *prefix;        /* first component of name */
   char *tail;          /* rest of name after prefix */
   FieldSpec    *ancestor;      /* parent of named field spec
                                   (or parent of parent ...). */
   FieldSpec    *field;         /* named field spec */

   /* Check for bad or empty arguments. */

   if (list == NULL || *list == NULL || name == NULL)
      return NULL;

   /* Parse name. */

   tail = strchr(name, DOT);

   /* Handle simple case immediately or complex case by
      recursion. */

   if (tail == NULL)            /* Just one component. */
      return GetField(list, name);
   else                 /* Multi-component name. */
      {
         tail++;                        /* Skip over dot. */

         prefix = FirstComponent(name);
         if (prefix == NULL)
            return NULL;

         ancestor = GetField(list, prefix);
         if (ancestor == NULL)  /* Search failed. */
            field = NULL;
         else                   /* Descend into subfields. */
            field = FindField(ancestor->subfields, tail);

         free(prefix);
         return field;
      }
}


/*
 * Add a field at top level to a field list.
 */

int
AddField(FieldList   *list,     /* variable containing field list */
         FieldSpec   *field)    /* specification of field */
{
   int          n;              /* length of list */
   FieldList    new_list;       /* new field list */

   /* Check for bad argument. */

   if (field == NULL || list == NULL)
      return FALSE;

   /* Extend list with 1 new pointer plus terminating NULL. */

   if (*list == NULL)
      {
         n = 0;
         new_list = (FieldList) malloc(2 * sizeof(**list));
      }
   else
      {
         n = FieldListLength(*list);
         new_list = (FieldList) realloc(*list, (n+2) * sizeof(**list));
      }

   if (new_list == NULL)        /* Allocation failure. */
      return FALSE;

   new_list[n] = field;
   new_list[n+1] = NULL;
   *list = new_list;

   return TRUE;
}


/*
 * Add field as subfield of another field.
 */

int
AddSubfield(FieldSpec   *field,    /* field to aquire subfield */
            FieldSpec   *subfield) /* subfield to be added */
{
   /* Check for bad arguments. */

   if (field == NULL || subfield == NULL)
      return FALSE;             /* Failure. */

   /* Add subfield. */

   return AddField(&field->subfields, subfield);
}


/*
 * Make NULL-terminated linear array in field order of REQUIRED
 * and OPTIONAL fields (and subfields ...) of a field list.
 * Fill in "fullname" members.
 */

FieldSpec **
FieldOrder(FieldList list)
{
   return SubFieldOrder(list, NULL);
}


/*
 * Make NULL-terminated linear array in type order of REQUIRED
 * and OPTIONAL fields (and subfields ...) of a field list.
 */

FieldSpec **
TypeOrder(FieldList list)
{
   FieldSpec    **fld_order;    /* fields in field order */
   FieldSpec   **typ_order;     /* fields in type order */
   long len;            /* number of fields ... */
   long i, j;           /* list indices */

   fld_order = FieldOrder(list);

   if (fld_order == NULL)
      return NULL;

   len = FieldArrayLength(fld_order);

   typ_order = (FieldSpec **) malloc((len+1) * sizeof(*typ_order));
   if (typ_order == NULL)
      return NULL;              /* Allocation failure. */

   j = 0;

   for (i = 0; i < len; i++)
      if (fld_order[i]->type == ARRAY)
         typ_order[j++] = fld_order[i];
   for (i = 0; i < len; i++)
      if (fld_order[i]->type == DOUBLE)
         typ_order[j++] = fld_order[i];
   for (i = 0; i < len; i++)
      if (fld_order[i]->type == DOUBLE_COMPLEX)
         typ_order[j++] = fld_order[i];
   for (i = 0; i < len; i++)
      if (fld_order[i]->type == FLOAT)
         typ_order[j++] = fld_order[i];
   for (i = 0; i < len; i++)
      if (fld_order[i]->type == FLOAT_COMPLEX)
         typ_order[j++] = fld_order[i];
   for (i = 0; i < len; i++)
      switch (fld_order[i]->type)
         {
         case LONG: /* Fall through */
         case ULONG:
            typ_order[j++] = fld_order[i];
         }
   for (i = 0; i < len; i++)
      if (fld_order[i]->type == LONG_COMPLEX)
         typ_order[j++] = fld_order[i];
   for (i = 0; i < len; i++)
      switch (fld_order[i]->type)
         {
         case SHORT:
         case USHORT:
         case WCHAR:
            typ_order[j++] = fld_order[i];
         }
   for (i = 0; i < len; i++)
      if (fld_order[i]->type == SHORT_COMPLEX)
         typ_order[j++] = fld_order[i];
   for (i = 0; i < len; i++)
      switch (fld_order[i]->type)
         {
         case SCHAR: /* Fall through. */
         case UCHAR: /* Fall through. */
         case CHAR: /* Fall through. */
         case BOOL:
            typ_order[j++] = fld_order[i];
         }
   for (i = 0; i < len; i++)
      if (fld_order[i]->type == SCHAR_COMPLEX)
         typ_order[j++] = fld_order[i];

   typ_order[j] = NULL;

   free(fld_order);

   return typ_order;
}


/*
 * Output via "order" (if not NULL) the ordering---FIELD_ORDER or
 * TYPE_ORDER---of a field list.  The result is TYPE_ORDER, the
 * default, if there is no field named "fieldOrder", and it is
 * the value of that field if it is a GLOBAL scalar numeric field
 * with one of the two allowed values.  In either of those cases
 * return TRUE for success.  Return FALSE for failure in any other
 * case.
 */

int
GetFieldOrdering(FieldList  list,
                 int        *order)
{
   FieldSpec    *field;
   long ord;

   field = FindField(list, "fieldOrder");

   if (field == NULL)
      ord = TYPE_ORDER;
   else if (field->occurrence != GLOBAL
            || field->rank != 0
            || field->data == NULL
            || !LongVal(field->data, field->type, &ord)
            || (ord != FIELD_ORDER && ord != TYPE_ORDER))
      {
         DebugMsg(1, "Bad field \"fieldOrder\".");
         return FALSE;
      }

   if (order != NULL)
      *order = ord;

   return TRUE;
}


/*
 * Set "list" to indicate an ordering of "order", which should be
 * FIELD_ORDER or TYPE_ORDER.  If no field named "fieldOrder" is
 * present, and "order" is TYPE_ORDER (the default) no action is
 * needed.  If the field exists and is not GLOBAL, return an
 * indication of failure.  Otherwise create the field if necessary
 * (and possible) make it a SHORT scalar field, and assign "order"
 * as its value.  Return TRUE for success and FALSE for failure.
 */

int
SetFieldOrdering(FieldList  *list,
                 int        order)
{
   FieldSpec    *field;

   if (*list == NULL)
      {
         DebugMsg(1, "SetFieldOrdering: NULL field list.");
         return FALSE;
      }

   field = FindField(*list, "fieldOrder");

   if (field == NULL)
      {
         if (order == TYPE_ORDER)
            return TRUE;

         field = NewFieldSpec(SHORT, 0);
         if (field == NULL)
            {
               DebugMsg(1, "SetFieldOrdering: Couldn't create field spec.");
               return FALSE;
            }

         field->name = StrDup("fieldOrder");
         field->occurrence = GLOBAL;
         if (!AddField(list, field))
            {
               DebugMsg(1, "SetFieldOrdering: Couldn't add field spec.");
               return FALSE;
            }
      }
   else    /* field != NULL */
      {
         if (field->occurrence != GLOBAL)
            {
               DebugMsg(1, "SetFieldOrdering: non-GLOBAL field \"fieldOrder\".");
               return FALSE;
            }
         field->type = SHORT;
         field->rank = 0;
      }

   field->data = ((field->data == NULL)
                  ? malloc(sizeof(short))
                  : realloc(field->data, sizeof(short)));
   if (field->data == NULL)
      {
         DebugMsg(1, "SetFieldOrdering: couldn't (re)allocate data.");
         return FALSE;
      }

   *(short *) field->data = order;
   return TRUE;
}


/*
 * *** GENERAL INPUT/OUTPUT
 * - ReadPreamble
 * - ReadFieldList
 * - ReadHeader
 * - OpenIn
 * - WritePreamble
 * - WriteFieldList
 * - WriteHeader
 * - OpenOut
 * - RecordSize
 * - ReadRecord
 * - WriteRecord
 * - ReadSamples
 * - WriteSamples
 */

/*
 * Read preamble from file.  Information returned via pointer
 * arguments.  Return TRUE on success, FALSE on failure.
 */

int
ReadPreamble(char **version,    /* version (output) */
             char **arch,       /* architecture (output) */
             long *pre_size,    /* preamble size (output) */
             long *hdr_size,    /* header size (output) */
             long *rec_size,    /* record size (output) */
             FILE *file)        /* input file */
{
   char    buf[PREAM_MAX + 1]; /* input string + null */

   /* Check magic number, MAGIC. */

   if (!GetLine(buf, 8, file))
      return FALSE;

   if (strcmp(buf, MAGIC) != 0)
      return FALSE;

   if (!GetLine(buf, 8, file))
      return FALSE;
   if (version != NULL)
      *version = StrDup(buf);

   /* Get architecture. */

   if (!GetLine(buf, 8, file))
      return FALSE;
   if (arch != NULL)
      *arch = StrDup(buf);

   /* Get preamble size */

   if (!GetLong(pre_size, 8, file))
      return FALSE;

   /* Could check *pre_size here. */

   /* Get header size */

   if (!GetLong(hdr_size, 8, file))
      return FALSE;

   /* Get record size */

   if (!GetLong(rec_size, 8, file))
      return FALSE;

   return TRUE;         /* success */
}


/*
 * Read field list from a given file in a given format.
 */

int
ReadFieldList(FieldList *list,
              int arch,
              FILE *file)
{
   switch (arch)
      {
      case EDR1:
         return ReadEdrFieldList(list, file, EDR1);
         break;
      case EDR2:
         return ReadEdrFieldList(list, file, EDR2);
         break;
      case NATIVE:
         return ReadNativeFieldList(list, file);
         break;
      case ASCII:
         return ReadAsciiFieldList(list, file);
         break;
      default:
         return FALSE;
         break;
      }
}


/*
 * Read header from file.
 * Return value is field list (NULL on failure).
 * Information from preamble is returned via pointer arguments.
 */

FieldList
ReadHeader(char     **version,  /* version (output) */
           int      *arch,      /* architecture (output) */
           long     *pre_size,  /* preamble size  (output) */
           long     *hdr_size,  /* header size (output) */
           long     *rec_size,  /* record size (output) */
           FILE     *file)      /* input file */
{
   FieldList    list;
   char *architecture;

   if (!ReadPreamble(version,
                     &architecture, pre_size, hdr_size, rec_size, file))
      return NULL;      /* Bad preamble. */
   if (strcmp(architecture, ARCH) == 0)    /* native architecture */
      {
         if (arch != NULL)
            *arch = NATIVE;

         if (!ReadNativeFieldList(&list, file))
            return NULL;
      }
   else if (strcmp(architecture, "EDR1") == 0)
      {
         if (arch != NULL)
            *arch = EDR1;

         /*
          * On machines whose native architecture is EDR1, could call
          * ReadNativeFieldList here.
          */
         if (!ReadEdrFieldList(&list, file, EDR1))
            return NULL;
      }
   else if (strcmp(architecture, "EDR2") == 0)
      {
         if (arch != NULL)
            *arch = EDR2;
         /*
          * On machines whose native architecture is EDR2, could call
          * ReadNativeFieldList here.
          */
         if (!ReadEdrFieldList(&list, file, EDR2))
            return NULL;
      }
   else if (strcmp(architecture, "ASCII") == 0)
      {
         if (arch != NULL)
            *arch = ASCII;

         if (!ReadAsciiFieldList(&list, file))
            return NULL;
      }
   else
      {
         if (arch != NULL)
            *arch = UNKNOWN;

         return NULL;           /* Unsupported architecture. */
      }

   return list;
}


/*
 * Open file "filename" for reading, or use stdin if filename is "-".
 * If successful, call ReadHeader on the file.  Return NULL in case
 * of failure.
 */

FieldList
OpenIn(char     *filename,      /* name of input file */
       char     **version,      /* version (output) */
       int      *arch,          /* architecture (output) */
       long     *pre_size,      /* preamble size (output) */
       long     *hdr_size,      /* header size (output) */
       long     *rec_size,      /* record size (output) */
       FILE     **file)         /* input file (output) */
{
   FILE    *input;

   if (filename == NULL)
      return NULL;

   if (strcmp(filename, "-") == 0)
      input = stdin;
   else
      input = fopen(filename, "rb"); /* Binary file */

   if (input == NULL)
      return NULL;

   if (file != NULL)
      *file = input;

   return ReadHeader(version, arch, pre_size, hdr_size, rec_size, input);
}


/*
 * Write preamble to file given architecture, field-list size, and
 * record size.  (Preamble size is known to the function; header size is
 * field-list size plus preamble size.)  Return TRUE on success, FALSE on
 * failure.
 */

int
WritePreamble(char  *arch,
              long  fld_size,
              long  rec_size,
              FILE  *file)
{
   long    pre_size = 48;       /* Preamble size: total length of
                                 * strings to be printed, including
                                 * terminal '\n'.
                                 *  8   magic number
                                 *  8   version
                                 *  8   architecture
                                 *  8   preamble size
                                 *  8   header size
                                 *  8   record size
                                 * --
                                 * 48   total
                                 */

   if (fprintf(file, "%7s\n", MAGIC) != 8)
      return FALSE;

   if (fprintf(file, "%7s\n", VERSION) != 8)
      return FALSE;

   if (fprintf(file, "%7s\n", arch) != 8)
      return FALSE;

   if (fprintf(file, "%7ld\n", pre_size) != 8)
      return FALSE;

   /* Header size: preamble size plus field-list size. */

   if (fprintf(file, "%7ld\n", fld_size + pre_size) != 8)
      return FALSE;

   if (fprintf(file, "%7ld\n", rec_size) != 8)
      return FALSE;

   return TRUE;         /* success */
}


/*
 * Write field list to file in specified format (Edr1 or Edr2 portable
 * binary, native binary, or Ascii.  "annotate" is ignored unless
 * the format is ASCII, and then it means add annotations in "[]"
 * for readability.  Return TRUE on success, FALSE on failure.
 */

int
WriteFieldList(FieldList list,
               int arch,
               FILE *file,
               Annot *annotate)
{
   switch (arch)
      {
      case EDR1:
         return WriteEdrFieldList(list, file, EDR1);
         break;
      case EDR2:
         return WriteEdrFieldList(list, file, EDR2);
         break;
      case NATIVE:
         return WriteNativeFieldList(list, file);
         break;
      case ASCII:
         return WriteAsciiFieldList(list, file, annotate);
         break;
      default:
         return FALSE;
         break;
      }
}


/*
 * Write header to file, in format for given architecture, given field
 * list.  The field list is written to a temporary file so
 * that the header size can be determined when the preamble is written.
 * (This is only really necessary if the output cannot be rewound---
 * e.g. a pipe.)
 * The value of annotate is ignored unless arch is ASCII; then a
 * non-NULL value means add annotations in "[]" for readability.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteHeader(FieldList  list,
            int        arch,
            FILE       *file,
            Annot      *annotate)
{
   FILE *temp;          /* temporary file */
   int  okay;           /* success or failure? */
   long rec_size=0L;    /* record size */
   long fld_size;       /* field-list size */
   static char  buf[BUFSIZ];    /* buffer for file copy */
   char *architecture=NULL;  /* architecture name */
   int  n;              /* byte count for file copy */
   long tot;            /* byte count total for file copy */


   /* Make temp file. */

   temp = tmpfile();

   if (temp == NULL)
      return FALSE;             /* Failure to make temp file. */

   /* Write field list to temp file; get record size. */

   switch (arch)
      {
      case NATIVE:
         okay = WriteNativeFieldList(list, temp);
         architecture = ARCH;
         if (okay)
            rec_size = NativeRecordSize(list);
         break;
      case EDR1:
         /*
          * On machines whose native architecture is EDR1, could call
          * WriteNativeFieldList here.
          */
         okay = WriteEdrFieldList(list, temp, EDR1);
         architecture = "EDR1";
         if (okay)
            rec_size = EdrRecordSize(list, EDR1);
         break;
      case EDR2:
         /*
          * On machines whose native architecture is EDR2, could call
          * WriteNativeFieldList here.
          */
         okay = WriteEdrFieldList(list, temp, EDR2);
         architecture = "EDR2";
         if (okay)
            rec_size = EdrRecordSize(list, EDR2);
         break;
      case ASCII:
         okay = WriteAsciiFieldList(list, temp, annotate);
         architecture = "ASCII";
         if (okay)
            rec_size = -1;
         break;
      default:
         DebugMsg(1, "WriteHeader: unrecognized architecture code.");
         okay = FALSE;          /* Unsupported architecture. */
         break;
      }

   if (!okay)
      {
         fclose(temp);
         return FALSE;          /* Failure to write field list. */
      }

   /* Get field-list size */

   fld_size = ftell(temp);

   /* Write preamble. */

   if (!WritePreamble(architecture, fld_size, rec_size, file))
      return FALSE;             /* Failure to write preamble. */

   /* Copy field list from temp file. */

   rewind(temp);

   tot = 0;
   while ((n = fread(buf, 1, BUFSIZ, temp)) > 0)
      {
         fwrite(buf, 1, n, file);
         tot += n;
      }

   if (tot != fld_size)
      return FALSE;             /* I/O error or wrong fld_size. */

   return TRUE;         /* success */
}


/*
 * Open file "filename" for writing, or use stdout if filename is "-".
 * If successful, call WriteHeader on the file.  Return FALSE in case
 * of failure.
 */

int
OpenOut(char        *filename,
        FieldList   list,
        int         arch,
        FILE        **file,
        Annot       *annotate)
{
   FILE    *output;

   if (filename == NULL)
      {
         DebugMsg(1, "OpenOut: NULL file name.");
         return FALSE;
      }

   if (strcmp(filename, "-") == 0)
      output = stdout;
   else
      output = fopen(filename, "wb"); /* Binary file */

   if (output == NULL)
      {
         DebugMsg(1, "OpenOut: couldn't open file.");
         return FALSE;
      }

   if (file != NULL)
      *file = output;

   return WriteHeader(list, arch, output, annotate);
}


/*
 * Size in bytes of one record for a given format,
 * according to a given field list.
 */

long
RecordSize(FieldList list,
           int arch)
{
   switch (arch)
      {
      case EDR1:
         return EdrRecordSize(list, EDR1);
         break;
      case EDR2:
         return EdrRecordSize(list, EDR2);
         break;
      case NATIVE:
         return NativeRecordSize(list);
         break;
      case ASCII:
         return -1;
         break;
      default:
         return -1;
         break;
      }
}


/*
 * Read one record from a file, in the format for a given architecture,
 * into the "data" members of the field specs on a NULL-terminated
 * linear array of REQUIRED and OPTIONAL fields, like those produced
 * by TypeOrder and FieldOrder.  The order of the array must correspond
 * exactly, without omissions, to the actual order of fields in the file
 * (field order or type order).
 * Return TRUE on success, FALSE on failure.
 */

int
ReadRecord(FieldSpec    **fields,
           int          arch,
           FILE         *file)
{
   if (file == NULL)
      return FALSE;

   switch (arch)
      {
      case NATIVE:
         if (!ReadNativeRecord(fields, file))
            return FALSE;
         break;
      case EDR1:
         if (!ReadEdrRecord(fields, file, EDR1))
            return FALSE;
         /*
          * On machines whose native architecture is EDR1, could call
          * ReadNativeRecord here.
          */
         break;
      case EDR2:
         if (!ReadEdrRecord(fields, file, EDR2))
            return FALSE;
         /*
          * On machines whose native architecture is EDR2, could call
          * ReadNativeRecord here.
          */
         break;
      case ASCII:
         if (!ReadAsciiRecord(fields, file))
            return FALSE;
         break;
      default:
         return FALSE;          /* Unsupported architecture. */
         break;
      }

   return TRUE;
}


/*
 * Write one record to a file, in the format for a given architecture,
 * from the "data" members of the field specs on a NULL-terminated
 * linear array of REQUIRED and OPTIONAL fields, like those produced by
 * TypeOrder and FieldOrder.
 * If annotate != NULL, add annotations in "[]" for readability.
 * Return TRUE on success, FALSE on failure.
 */

int
WriteRecord(FieldSpec   **fields,
            int         arch,
            FILE        *file,
            Annot       *annotate)
{
   if (file == NULL)
      return FALSE;

   switch (arch)
      {
      case NATIVE:
         if (!WriteNativeRecord(fields, file))
            return FALSE;
         break;
      case EDR1:
         if (!WriteEdrRecord(fields, file, EDR1))
            return FALSE;
         /*
          * On machines whose native architecture is EDR1, could call
          * WriteNativeRecord here.
          */
         break;
      case EDR2:
         if (!WriteEdrRecord(fields, file, EDR2))
            return FALSE;
         /*
          * On machines whose native architecture is EDR2, could call
          * WriteNativeRecord here.
          */
         break;
      case ASCII:
         if (!WriteAsciiRecord(fields, file, annotate))
            return FALSE;
         break;
      default:
         return FALSE;          /* Unsupported architecture. */
         break;
      }

   return TRUE;
}


/*
 * Read "nrec" one-field records from "file", in the format given by
 * the integer code "arch", into the array indicated by "data".
 * The field is specified by "fields", a NULL-terminated linear array
 * like those produced by TypeOrder and FieldOrder and containing
 * exactly one REQUIRED field.  Return the number of complete records
 * read.
 */

long
ReadSamples(void        *data,
            long        nrec,
            FieldSpec   **fields,
            int         arch,
            FILE        *file)
{
   if (data == NULL)
      return 0;
   if (nrec <= 0)
      return 0;
   if (fields == NULL || fields[0] == NULL || fields[1] != NULL
       || fields[0]->occurrence != REQUIRED)
      return 0;
   if (file == NULL)
      return 0;

   switch (arch)
      {
      case NATIVE:
         return ReadNativeSamples(data, nrec, fields, file);
      case EDR1:
         return ReadEdrSamples(data, nrec, fields, file, EDR1);
      case EDR2:
         return ReadEdrSamples(data, nrec, fields, file, EDR2);
      case ASCII:
         return ReadAsciiSamples(data, nrec, fields, file);
      default:
         return 0;              /* Unsupported architecture. */
         break;
      }
}


/*
 * Write "nrec" one-field records to "file" in the format given by
 * the integer code "arch", from the array indicated by "data".
 * The field is specified by "fields", a NULL-terminated linear
 * array like those produced by TypeOrder and FieldOrder
 * and containing exactly one REQUIRED field.
 * Return the number of complete records written.
 */

long
WriteSamples(void       *data,
             long       nrec,
             FieldSpec  **fields,
             int        arch,
             FILE       *file,
             Annot      *annotate)
{
   if (data == NULL)
      return 0;
   if (nrec <= 0)
      return 0;
   if (fields == NULL || fields[0] == NULL || fields[1] != NULL
       || fields[0]->occurrence != REQUIRED)
      return 0;
   if (file == NULL)
      return 0;

   switch (arch)
      {
      case NATIVE:
         return WriteNativeSamples(data, nrec, fields, file);
      case EDR1:
         return WriteEdrSamples(data, nrec, fields, file, EDR1);
      case EDR2:
         return WriteEdrSamples(data, nrec, fields, file, EDR2);
      case ASCII:
         return WriteAsciiSamples(data, nrec, fields, file, annotate);
      default:
         return 0;              /* Unsupported architecture. */
         break;
      }
}


/*
 * Allocate sufficient storage for "nrec" one-field records.
 * The field is specified by "fields", a NULL-terminated linear
 * array like those produced by TypeOrder and FieldOrder
 * and containing exactly one REQUIRED field.
 * Return NULL in case of failure.
 */

void *
AllocSamples(long       nrec,
             FieldSpec  **fields)
{
   if (fields == NULL || fields[0] == NULL || fields[1] != NULL
       || fields[0]->occurrence != REQUIRED)
      return NULL;

   return malloc(nrec * FieldLength(fields[0])
                 * InternTypeSize(fields[0]->type));
}


/*
 * LOCAL FUNCTION DEFINITIONS
 */

/*
 * Number of fields in a NULL-terminated linear array.
 */

static int
FieldArrayLength(FieldSpec **fields)
{
   int      i;                  /* array index */

   if (fields == NULL)
      return 0;

   for (i = 0; fields[i] != NULL; i++)
      { }

   return i;
}


/*
 * Make NULL-terminated linear array in field order of REQUIRED
 * and OPTIONAL fields (and subfields ...) of a field list.
 * Fill in "fullname" members; if "prefix" is non-NULL, prefix it
 * to each "fullname" with a connecting dot (".").
 */

static FieldSpec **
SubFieldOrder(FieldList list, char *prefix)
{
   FieldSpec    **fields;       /* linear array of fields */
   FieldSpec    **old_fields;   /* prior value of fields */
   long nfld;           /* number of fields */
   long i, j;           /* list indices */
   FieldSpec    *fld;           /* top-level field */
   FieldSpec    **subfields;    /* linear array of subfields */
   long nsub;           /* number of subfields */
   long preflen = 0L;   /* length of prefix */
   char *fullname;      /* name including prefix & dot */


   if (list == NULL)
      return NULL;

   fields = NULL;
   nfld = 0;

   if (prefix != NULL)
      preflen = strlen(prefix);

   for (i = 0; list[i] != NULL; i++)
      {
         fld = list[i];

         if (prefix == NULL)
            fullname = StrDup(fld->name);
         else
            {
               fullname = (char *) malloc(preflen + strlen(fld->name) + 2);
               sprintf(fullname, "%s.%s", prefix, fld->name);
            }
         fld->fullname = fullname;

         if (fld->occurrence != VIRTUAL)
            {
               if (fld->occurrence != GLOBAL && fld->type != NO_TYPE)
                  /* REQUIRED or OPTIONAL */
                  {
                     old_fields = fields;
                     fields = (FieldSpec **)
                        ((fields == NULL)
                         ? malloc((nfld + 2) * sizeof(*fields))
                         : realloc(fields, (nfld + 2) * sizeof(*fields)));

                     if (fields == NULL) /* Allocation failure. */
                        {
                           free(old_fields);
                           return NULL;
                        }

                     /* Could use AddField here, depending on field lists being
                      * implemented as NULL-terminated arrays of pointers. E.g.:
                      *
                      * if (!AddField(&fields, fld))
                      * {
                      *     if (fields != NULL)
                      *     free(fields);
                      *     return NULL;
                      * }
                      */

                     fields[nfld] = fld;
                     nfld++;
                     fields[nfld] = NULL;
                  }

               subfields = SubFieldOrder(fld->subfields, fullname);

               if (subfields != NULL)
                  {
                     /* Append subfields to fields. */

                     nsub = FieldArrayLength(subfields);
                     old_fields = fields;
                     fields = (FieldSpec **)
                        ((fields == NULL)
                         ? malloc((nfld + nsub + 1) * sizeof(*fields))
                         : realloc(fields,
                                   (nfld + nsub + 1) * sizeof(*fields)));

                     if (fields == NULL) /* Allocation failure. */
                        {
                           free(old_fields);
                           free(subfields);
                           return NULL;
                        }

                     for (j = 0; j < nsub; j++, nfld++)
                        fields[nfld] = subfields[j];

                     fields[nfld] = NULL;

                     free(subfields);
                  }
            }
      }

   return fields;
}


/*
 * Find field specification with given name at top level on given
 * field list.
 */

static FieldSpec *
GetField(FieldList  list,       /* field list */
         char       *name)      /* component name */
{
   int      i;                  /* index into list */

   if (list == NULL || name == NULL)
      return NULL;

   /* Search NULL-terminated list for field spec with
      matching name. */

   for (i = 0;
        list[i] != NULL && strcmp(name, list[i]->name) != 0;
        i++)
      { }

   return list[i];
}


/*
 * Copy first component of field name: everything up to
 * first dot if any or whole string if none.
 */

static char *
FirstComponent(char *name)
{
   char    *tail;               /* pointer to first dot */
   long    len;         /* length of first component */
   int      terminator;         /* delimiting char: '.' or null */
   char    *head;               /* copy of first component */
   char    *p;                  /* pointer into copy */

   if (name == NULL)
      return NULL;

   /* Parse name. */

   tail = strchr(name, DOT);

   if (tail == NULL)            /* Just one component. */
      {
         len = strlen(name);
         terminator = '\0';
      }
   else                 /* More than one component. */
      {
         len = tail - name;
         terminator = DOT;
      }

   /* Allocate space for copy including final null. */

   head = (char *) malloc(1 + len);
   if (head == NULL)
      return NULL;

   /* Copy. */

   p = head;
   while (*name != terminator)
      *p++ = *name++;
   *p = '\0';

   return head;
}


/*
 * Read line of length len (including '\n') from file into buf.
 * Check length of line, presence of terminating newline,
 * and absence of trailing blanks.
 * Trim newline and leading blanks and supply terminating null.
 * Return TRUE on success, FALSE on failure.
 */

static int
GetLine(char    *buf,
        int     len,
        FILE    *file)
{
   int     i, j;

   fgets(buf, len+1, file);
   if (strlen(buf) != len || buf[len-1] != '\n')
      return FALSE;
   buf[len-1] = '\0';

   /* count leading blanks */
   for (j = 0; buf[j] == ' '; j++)
      { }

   if (j < len - 1 && buf[len-2] == ' ')
      return FALSE;             /* not right justified */

   if (j == 0)
      return TRUE;

   /* remove leading blanks */
   for (i = 0; buf[j] != '\0'; i++,j++)
      buf[i] = buf[j];

   buf[i] = '\0';

   return TRUE;         /* success */
}


/*
 * Read line of length len (including '\n') from file.  Check length of
 * line and presence of terminating newline.  Convert to long integer,
 * assign result through pointer val if non-NULL.  Check for garbage
 * characters following the constant.  Return TRUE on success, FALSE
 * on failure.
 */

static int
GetLong(long    *val,
        int     len,
        FILE    *file)
{
   char    *buf;
   char    *ptr;                /* end-of-scan pointer from strtol */
   long    value;               /* converted value */

   buf = (char *) malloc(len+1);   /* len + 1 for terminating null */
   if (buf == NULL)
      return FALSE;             /* Allocation failure. */

   /* Read line; check length. */

   fgets(buf, len+1, file);

   if (strlen(buf) != len || buf[len-1] != '\n')
      {
         free(buf);
         return FALSE;
      }

   /* Convert; check for bad format. */

   value = strtol(buf, &ptr, 10);

   if (ptr != buf + (len-1))
      {
         free(buf);
         return FALSE;
      }

   /* Clean up; return. */

   free(buf);

   if (val != NULL)
      *val = value;

   return TRUE;         /* success */
}


/*
 * If the value of the indicated type, located at "src" can be represented
 * as a long, convert to long, store the result at "dest", and return
 * TRUE.  Otherwise return FALSE.
 */

static int
LongVal(void *src, int type, long *dest)
{
   switch (type)
      {
      case BOOL:
         *dest = *(Bool *) src;
         break;
      case UCHAR:
         *dest = *(Uchar *) src;
         break;
      case CHAR:
         *dest = *(char *) src;
         break;
      case WCHAR:
         *dest = *(Wchar *) src;
         break;
      case SCHAR:
         *dest = *(Schar *) src;
         break;
      case SHORT:
         *dest = *(short *) src;
         break;
      case USHORT:
         *dest = *(Ushort *) src;
         break;
      case LONG:
         *dest = *(long *) src;
         break;
      case ULONG:
         if (*(Ulong *) src > LONG_MAX)
            return FALSE;
         *dest = *(Ulong *) src;
         break;
      case FLOAT:               /* Fall through */
      case DOUBLE:      /* Fall through */
      case FLOAT_COMPLEX:       /* Fall through */
      case DOUBLE_COMPLEX:
         {
            double  x=0.0;

            switch (type)
               {
               case FLOAT:
                  x = *(float *) src;
                  break;
               case DOUBLE:
                  x = *(double *) src;
                  break;
               case FLOAT_COMPLEX:
                  if (((FloatComplex *) src)->imag != 0.0)
                     return FALSE;
                  x = ((FloatComplex *) src)->real;
                  break;
               case DOUBLE_COMPLEX:
                  if (((DoubleComplex *) src)->imag != 0.0)
                     return FALSE;
                  x = ((DoubleComplex *) src)->real;
                  break;
               }
            if (x != floor(x) || x > LONG_MAX || x < LONG_MIN)
               return FALSE;
            *dest = (long) x;
         }
         break;
      case SCHAR_COMPLEX:
         if (((ScharComplex *) src)->imag != 0)
            return FALSE;
         *dest = ((ScharComplex *) src)->real;
         break;
      case SHORT_COMPLEX:
         if (((ShortComplex *) src)->imag != 0)
            return FALSE;
         *dest = ((ShortComplex *) src)->real;
         break;
      case LONG_COMPLEX:
         if (((LongComplex *) src)->imag != 0)
            return FALSE;
         *dest = ((LongComplex *) src)->real;
         break;
      default:
         return FALSE;
      }

   return TRUE;
}
