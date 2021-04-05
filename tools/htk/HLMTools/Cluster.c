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
/* author: Gareth Moore <glm20@eng.cam.ac.uk>                  */
/*                                                             */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*          1999-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*            Cluster.c: Cluster words into classes            */


char *Cluster_version = "!HVER!Cluster:   3.4.1 [CUED 12/03/09]";
char *Cluster_vc_id = "$Id: Cluster.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";

/* HTK/HLM libraries: */
#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"
#include "LWMap.h"
#include "LGBase.h"
#include "LModel.h"
#include "LCMap.h"

/* Uncomment the following line to run integrity checks on each iteration
   to ensure that:
   * The class counts all add up correctly
   * The maximum-likelihood values have all been updated correctly     */
/*#define INTEGRITY_CHECK */


/* -------------------------- Trace Flags ------------------------ */

#define T_TOP   00001              /* Basic tracing */
#define T_FILE  00002              /* Report major file operations */
#define T_EXTRA 00004              /* Extra tracing */
#define T_BOND    007              /* Undercover tracing */
#define T_MEM   00010              /* Trace memory usage */



/* Constants */
/* Size of blocks we grab from New() and then allocate internally
   (This is done to avoid grabbing around 100,000 small blocks!) #bytes */
#define block_grab_size        1048576
/* Cut-off point at which we decide not to use the internal block and
   just use New() - number of bigrams using a given word */
#define block_cut_off          (200*sizeof(bi_count))
/* Initial size of bigram read buffer (large enough to hold maximum number
   of bigrams featuring a single word in a single position) - can grow */
#define initial_bigram_buffer  10000
/* Granularity of growth of above buffer, if required */
#define bigram_buffer_grow     1000

/* Identifiers for word clustering sort orders */
#define SORT_WMAP 1
#define SORT_FREQ 2


/* Type definitions */

/* Bigram count */
typedef struct {
   UInt id;	  /* Word id */
   int  count;	  /* Bigram count */
}
bi_count;

/* All bigrams which start or end with a certain word */
typedef struct {
   bi_count *bi;   /* Array of counts */
   int       size; /* Number of bigrams with this word in */
}
bigrams;

typedef UInt unigram;   /* Occurrence count */


/* ---------------------- Global Variables ----------------------- */
/* DEFAULTS */

/* Global variables - defaults */
static int         N = 1000;                /* Default number of classes */
static Boolean     show_MLV=FALSE;          /* Show MLV after each change */
static char       *export_prefix="cluster"; /* Prefix of export filenames */
static Boolean     unk_sep = FALSE;         /* Keep unknown word in its own class? */
static Boolean     outCMapRaw = FALSE;      /* Output classes in raw mode */
static Boolean     inCMapRaw = FALSE;       /* Input classes in raw mode */

/* Global variables - others */
/* Used by core clusterer */
static int       **clCnt=NULL;              /* Array of arrays; index with count[c1][c2]
                                               (clCnt = 'class count') */
static int        *tmp_c1=NULL;             /* Temporary set of bigrams (1) */
static int        *tmp_c2=NULL;             /* Temporary set of bigrams (2) */
static int        *tmp_c3=NULL;             /* Temporary set of bigrams (3) */
static int        *tmp_c4=NULL;             /* Temporary set of bigrams (4) */
static int        *tmp_sum1=NULL;           /* Temporary word-class counts (1) */
static int        *tmp_sum2=NULL;           /* Temporary word-class counts (2) */
static int        *clSum=NULL;              /* Class unigram [classes]
                                               returns word unigram sum */
static int	  *clMemb=NULL;             /* Class membership [words]
                                               returns class given a word */
static int         GwGw, gGw, Gwg, gg;      /* Special-case class counts */
static double     *mlv;                     /* ML values involving class [N] */
static int        *bipair;                  /* Array of word bigrams (w,w) */
static int         sum_of_all_bigram_counts;/* Sum of all bigram counts */
static int         sum_of_all_uni_counts;   /* Sum of all unigram counts */
static int         curr_class;              /* Temporary value, saves passing */
static int         start_class = 2;         /* Which is the first 'real' class? */
static double      curr_MLV=0;              /* ...and its current value */
static int         W = 0;     		    /* Number of words */
static bigrams    *forward, *backward;      /* Forward and backward bigram tables */
static int         export_index=0;          /* What iteration is this? */
static FILE       *logfile=NULL;            /* Log progress to this file */
static char        tmp[256];                /* Scrap array */
static int         start_id=-1, end_id=-1;  /* Start and end word ids */
static int         unk_id=-1;               /* Unknown word token id */
static MemHeap     global_heap;             /* Claim fixed block memory from here */
static MemHeap     global_stack;            /* Claim other memory from here */
/* Used by uni/bigram storage */
static unigram     *uni;                    /* Unigram store */
static int          max_words;              /* Maximum number of words */
static bigrams     *forward=0, *backward;    /* Forward and backward bigram tables */
static void        *block=0;                /* First word of free memory we have */
static void        *block_end=0;            /* First byte after current block */
static UInt         last_word;              /* ID of last word (w,?) read in */
static int          store_idx;              /* Next index of second word in bigram */
static bi_count    *store;                  /* Store of current word w (w,*) pairs */
static int          curr_bistore_size;      /* Current size of bigram buffer store */
/* Front-end code */
static WordMap      wmap;                   /* HTK word map */
static MemHeap      imem;                   /* memory for input gram file set */
static MemHeap      imem2;                  /* memory for input gram file set (copy) */
static NGInputSet   inset;                  /* input gram file set */
static NGInputSet   inset2;                 /* input gram file set (copy) */
static char         sent_start[256];        /* sentence start word */
static char         sent_end[256];          /* sentence end word */
static char         unknown_w[256];         /* unknown word token */
static ConfParam   *cParm[MAXGLOBS];        /* configuration script parameters */
static int          nParm = 0;              /* total num params */
static int          trace = 0;              /* trace setting */
static UInt        *class_sort;	            /* Used to sort output alphabetically */
static Boolean      pipe_logfile;           /* HShell file handling - using pipe? */
static int          rec_freq = 1000;        /* Frequency we write recovery files (0 = off) */
static Boolean      verbose = FALSE;        /* Verbose file logging */
static Boolean      write_logfile = TRUE;   /* Write a log file during execution */
static int          sort_order = SORT_WMAP; /* Order words are considered in */
static int         *sort_uni;               /* Sort unigrams by count */
static Boolean     outCMapRawTrap = FALSE;  /* Has this been changed by config file? */
static Boolean     inCMapRawTrap = FALSE;   /* Has this been changed by config file? */

/* ---------------- Function Prototypes -------------------------- */

#ifdef INTEGRITY_CHECK
static void check_counts_sum(void);
static void max_likelihood_check(void);
#endif
static void max_likelihood_init(void);

/* Add a bigram */
void bigram_add(NGram ng, int count);

/* Call when all bigrams have been passed in */
void bigram_added_all(void);

/* Must be called before almost any other function in this file will work */
void bigram_init(int words);

/* Initialise this unigram storage module */
void unigram_init(int numb_words);

/* Add a unigram */
void unigram_add(NGram ng, int count);

/* Read a unigram */
UInt unigram_read(UInt id);

/* Set whether to show MLV or not (non-zero = on) */
void classes_showMLV(int on);

/* Set prefix for all output files */
void set_output_prefix(char *name);

/* Return the number of classes used by default */
int classes_get_default(void);

/* Set the number of classes used */
void classes_set_number(int numb);

/* Initialise this module - MUST have initialised bigrams first */
void classes_init(int numb_words);

/* Perform a given number of iterations of the clustering algorithm */
void cluster_words(int iterations);

/* Setup all class counts, given existing class word map */
void setup_all_counts(void);

/* Perform some initial clustering - currently just puts all in one class,
   except for given start, end and unknown (if -k passed) ids */
void initial_cluster(void);

/* Write out class sets (pass non-zero to write recovery file) */
void export_classes(int recovery);

/* Import existing HLM classmap */
void import_classmap(char *fname, int numb_words);

/* Recover from a given recovery file */
void do_recovery(char *fname, int words);

/* Write out p(word | class) probabilities */
void write_word_probs(char *filename);

/* Write out p(word | class) counts */
void write_word_counts(char *filename);

/* Specify whether to keep the unknown word in its own solo-member
   class or not (non-zero = keep separate) */
void classes_keep_unk_separate(int keep_separate);

/* Pass in start, end and unknown word ids */
void set_ids(int start_id, int end_id, int unk_id);

/* Report an error message to stderr */
void report_error(char *text);

/* TEMP? */
char *what_is_word(UInt id); /* In Cluster.c */

/* ---------------- Process Command Line ------------------------- */

/* See if any configuration parameters have been set for this tool */
void SetConfParms(void)
{
   char b[256];
   int  i;

   nParm = GetConfig("CLUSTER", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE", &i))      trace = i;
      if (GetConfStr(cParm,nParm,"STARTWORD", b))   strcpy(sent_start, b);
      if (GetConfStr(cParm,nParm,"ENDWORD", b))     strcpy(sent_end, b);
      if (GetConfStr(cParm,nParm,"UNKNOWNNAME", b)) strcpy(unknown_w, b);
      if (GetConfBool(cParm,nParm,"INCMAPRAW", &inCMapRaw)) {
         inCMapRawTrap = TRUE;
      }
      if (GetConfBool(cParm,nParm,"OUTCMAPRAW", &outCMapRaw)) {
         outCMapRawTrap = TRUE;
      }
   }
}


/* Provide skeleton help */
void ReportUsage(void)
{
   printf("\nUSAGE: Cluster [options] mapfile gramfile ...\n\n");
   printf(" Option                                       Default\n");
   printf(" -c n    use n classes                        %d\n", classes_get_default());
   printf(" -i n    perform n iterations                 1\n");
   printf(" -k      put unknown word in a separate class off\n");
   printf(" -l f    start from existing classmap 'f'     off\n");
   printf(" -m      add running ML values to logfile     %s\n", show_MLV?"on":"off");
   printf(" -n      do not produce any logfile output    %s\n", write_logfile?"off":"on");
   printf(" -o f    set prefix of output files           %s\n", export_prefix);
   printf(" -p f    write word|class probs to file 'f'   off\n");
   printf(" -q f    write word|class counts to file 'f'  off\n");
   printf(" -r n    write recovery file freq (0=off)     %d\n", rec_freq);
   printf(" -s t    specify sentence start word as 't'   %s\n", DEF_STARTWORD);
   printf(" -t t    specify sentence end word as 't'     %s\n", DEF_ENDWORD);
   printf(" -u t    specify unknown word token as 't'    %s\n", DEF_UNKNOWNNAME);
   printf(" -v      use verbose log file format          %s\n", verbose?"on":"off");
   printf(" -w t    specify word sort order - WMAP/FREQ  %s\n", sort_order==SORT_WMAP?"WMAP":"FREQ");
   printf(" -x f    continue from recovery file 'f'      off\n");
   printf(" Standard options:\n");
   PrintStdOpts("");
   printf("\n");
}


void check_file(FILE *file, char *fname, char *function)
{
   if (!file)
      HError(17011, "%s: Can't open file '%s'", function, fname);
}


/* --------------------- Import N-grams ----------------- */


/* LoadBiGrams: load in N-gram files, keeping only bigrams */
static void LoadBiGrams()
{
   UInt   ng[2];
   float  cnt; /* Occurrence count */
   int    added=0;

   if (trace & T_FILE) {
      printf("Loading bigrams from N-gram files\n");
   }
   OpenInputSet(&inset);
   if (trace & T_FILE) {
      printf("Opened input set of %d entries\n", inset.nFiles);
   }

   while (GetNextNGram(&inset, ng, &cnt, 2)) {
      /* ng stores ngram in format [0],[1]...[N]; count is separate */
      ng[0] = GetMEIndex(&wmap, ng[0]);
      ng[1] = GetMEIndex(&wmap, ng[1]);
      bigram_add(ng, (int) cnt);
      added++;
   }
   CloseInputSet(&inset);
   if (trace & T_FILE) {
      printf("Bigram load complete - %d bigrams imported\n", added);
   }
}


/* LoadUniGrams: load in N-gram files - we want unigrams */
static void LoadUniGrams()
{
   UInt   ng[1];
   float  cnt; /* Occurrence count */
   int    added=0;

   if (trace & T_FILE) {
      printf("Loading unigrams from N-gram files\n");
   }
   OpenInputSet(&inset2);

   while (GetNextNGram(&inset2, ng, &cnt, 1)) {
      /* ng stores ngram in format [0],[1]...[N]; count is separate */
      ng[0] = GetMEIndex(&wmap, ng[0]); /* convert into value indexed from 0 */
      unigram_add(ng, (int) cnt);
      added++;
   }
   CloseInputSet(&inset2);
   if (trace & T_FILE) {
      printf("Unigram load complete - %d unigrams imported\n", added);
   }
}


/* Return word text given an internal id */
char *what_is_word(UInt id)
{
   return wmap.id[id]->name;
}


/* Return a word id given a word */
UInt get_id_from_word(char *word)
{
   if (!(GetLabId(word, FALSE))) {
      HError(17050, "Word '%s' found in class map but not in word map", word);
   }
   return GetMEIndex(&wmap, (((MapEntry *)(GetLabId(word, FALSE)->aux))->ndx));
}


/* Class functions */

/* Set whether to show MLV or not */
void classes_showMLV(int on)
{
   show_MLV = on ? TRUE : FALSE;
}


/* Set prefix for all output files */
void set_output_prefix(char *name)
{
   if (clCnt) {
      HError(-17099, "set_output_prefix(): this function must be called before initialisation");
      /* No need to abort - it will just affect future files opened */
   }
   export_prefix = New(&global_stack, strlen(name)+1);
   strcpy(export_prefix, name);
}


/* Return the number of classes used by default (or currently set) */
int classes_get_default(void)
{
   return N;
}


/* Set the number of classes used */
void classes_set_number(int numb)
{
   if (clCnt) {
      HError(17099, "classes_set_number: must be called prior to initialisation");
   }

   if (numb<1) {
      HError(17099, "classes_set_number: number of classes must be 1 or more");
   }
   
   N = numb;
}


/* Initialise this module */
void classes_init(int numb_words)
{
   int i, j;

   if (W>0 && W!=numb_words) {
      /* Could only get here if code to do with loading classmap is broken */
      HError(17098, "classes_init: internal inconsistency - number of words has changed");
   }
   W = numb_words;

   /* Create empty storage table */
   if (trace & T_MEM) {
      printf("Allocating memory for %d classes\n", N);
   }
   class_sort = CNew(&global_stack, W * sizeof(UInt));
   clSum = CNew(&global_stack, N * sizeof(int));
   tmp_c1 = CNew(&global_stack, N * sizeof(int));
   tmp_c2 = CNew(&global_stack, N * sizeof(int));
   tmp_c3 = CNew(&global_stack, N * sizeof(int));
   tmp_c4 = CNew(&global_stack, N * sizeof(int));
   tmp_sum1 = CNew(&global_stack, N * sizeof(int));
   tmp_sum2 = CNew(&global_stack, N * sizeof(int));
   mlv = CNew(&global_stack, N * sizeof(double));
   sort_uni = CNew(&global_stack, W * sizeof(int));
   if (!clMemb)
      clMemb = CNew(&global_stack, W * sizeof(int)); /* May have been setup by existing map */
   clCnt = CNew(&global_stack, N * sizeof(int *));
   for (i=0; i<N; i++) {
      clCnt[i] = CNew(&global_stack, N * sizeof(int));
   }
   if (trace & T_MEM) {
      printf("Class memory allocated\n");
   }

   /* Create array of bigram (w,w) pair counts (ie. word followed by itself) */
   bipair = CNew(&global_stack, W * sizeof(int));
   for (i=0; i<W; i++) {
      bipair[i] = 0;
      for (j=0; j<forward[i].size; j++) {
         if (forward[i].bi[j].id == i) {
            bipair[i] = forward[i].bi[j].count;
            break;
         }
      }
   }
}


/* See what change results when word 'w' moved to class 'g' */
static void classes_change(UInt w, int g)
{
   register int i;

   /* tmp_c1[] stores the set of class counts C(G(w),*)
      tmp_c2[] stores the set of class counts C(*,G(w))
      tmp_c3[] stores the set of class counts C(g,*)
      tmp_c4[] stores the set of class counts C(*,g)
      tmp_sum1[] stores the bigram class counts C(w,*)
      tmp_sum2[] stores the bigram class counts C(*,w) */
   
   /* Loop over all classes */
   for (i=0; i<N; i++) {
      if (i!=curr_class && i!=g) {
         if (tmp_sum1[i]) {
            /* (G(w),gi) => -C(w,gi) */
            tmp_c1[i] = clCnt[curr_class][i] - tmp_sum1[i];
            /* (g,gi)    => +C(w,gi) */
            tmp_c3[i] = clCnt[g][i] + tmp_sum1[i];
         }
         else {
            tmp_c1[i] = clCnt[curr_class][i];
            tmp_c3[i] = clCnt[g][i];
         }
         if (tmp_sum2[i]) {
            /* (gi,G(w)) => -C(gi,w) */
            tmp_c2[i] = clCnt[i][curr_class] - tmp_sum2[i];
            /* (gi,g)    => +C(gi,w) */
            tmp_c4[i] = clCnt[i][g] + tmp_sum2[i];
         }
         else {
            tmp_c2[i] = clCnt[i][curr_class];
            tmp_c4[i] = clCnt[i][g];
         }
      }
   }
  
   /* Calculate correct values for class-to-or-from-only pairs */
   /* (G(w),G(w)) => -C(w,G(x)) - C(G(x),w) + C(w,w) */
   GwGw =   clCnt[curr_class][curr_class]
          - tmp_sum1[curr_class] - tmp_sum2[curr_class] + bipair[w];
   /* (G(w),g)    => -C(w,g) + C(G(x),w) - C(w,w) */
   Gwg  =   clCnt[curr_class][g]
          - tmp_sum1[g] + tmp_sum2[curr_class] - bipair[w];
   /* (g,G(w))    => -C(g,w) + C(w,G(x)) - C(w,w) */
   gGw  =   clCnt[g][curr_class]
          - tmp_sum2[g] + tmp_sum1[curr_class] - bipair[w];
   /* (g,g)       => +C(w,g) + C(g,w) + C(w,w) */
   gg   =   clCnt[g][g]
          + tmp_sum1[g] + tmp_sum2[g] + bipair[w];
}


/* Decide on a class to move word 'w' to. Returns class index. */
static int choose_class(UInt w)
{
   register int i;
   register int j;
   double d;             /* Change in optimisation value */
   int uniGx, unig;
   int best_class;
   double best_change;
   double start_value;
   
   best_class = curr_class;
   best_change = 0;

   /* Create set of forward bigram class counts, C(w,*) and C(*,w)
      (for * = any class) */
   for (i=0; i<N; i++) {
      tmp_sum1[i] = 0;
      tmp_sum2[i] = 0;
   }
   for (i=0; i<forward[w].size; i++) {
      tmp_sum1[clMemb[forward[w].bi[i].id]] += forward[w].bi[i].count;
   }
   for (i=0; i<backward[w].size; i++) {
      tmp_sum2[clMemb[backward[w].bi[i].id]] += backward[w].bi[i].count;
   }

  /* Try all classes */
   for (i=start_class; i<N; i++) {
      if (i==curr_class || uni[w]==0) {
         /* If we have no information about this word, or its a self-move, don't
            bother (self-move gives zero change) */
         continue;
      }

      d = 0;
      classes_change(w, i);

      /* Word has moved to class i, so see how this would change our
         optimisation equation */
      uniGx = clSum[curr_class] - uni[w];
      unig  = clSum[i] + uni[w];

      /* Counts involving original class and a new class */
      for (j=0; j<N; j++) {
         if ((j!=curr_class) && (j!=i)) {
            if (tmp_c1[j]) {
               d += ((double)tmp_c1[j]) * log(tmp_c1[j]);
            }
            if (tmp_c2[j]) {
               d += ((double)tmp_c2[j]) * log(tmp_c2[j]);
            }
            if (tmp_c3[j]) {
               d += ((double)tmp_c3[j]) * log(tmp_c3[j]);
            }
            if (tmp_c4[j]) {
               d += ((double)tmp_c4[j]) * log(tmp_c4[j]);
            }
         }
      }
      /* Unigram part of summation */
      if (uniGx) {
         d -= 2*(((double)uniGx)*log(uniGx));
      }
      if (unig) {
         d -= 2*(((double)unig)*log(unig));
      }

      /* Exceptions */
      if (GwGw) {
         d += ((double)GwGw) * log(GwGw);
      }
      if (Gwg) {
         d += ((double)Gwg) * log(Gwg);
      }
      if (gGw) {
         d += ((double)gGw) * log(gGw);
      }
      if (gg) {
         d += ((double)gg) * log(gg);
      }

      /* Now make 'd' into a difference: */
      start_value = mlv[curr_class] + mlv[i];
      /* Subtract off the two values we added twice by using mlv[] */
      if (clCnt[curr_class][i])
         start_value -= clCnt[curr_class][i]*log(clCnt[curr_class][i]);
      if (clCnt[i][curr_class])
         start_value -= clCnt[i][curr_class]*log(clCnt[i][curr_class]);
      /* And calculate 'd': */
      d = d - start_value;

      if (verbose && logfile) {
         fprintf(logfile, "...moving word %d to class %d from class %d gives %f change\n",
                 w, i, curr_class, d);
      }

      if (d>best_change) {
         /* Accuracy check - this is a bit of a dodgy hack for gcc 3 */
         sprintf(tmp, "%f", d); /* This will flush d from the higher-precision maths register */
         /* No doubt a much faster way of doing this, but never mind */
         if (d>best_change) {

            if (verbose && logfile) {
               fprintf(logfile, "   \\- which is better than the current best\n");
            }
            best_change = d;
            best_class = i;
         }
         else {
            HError(-17059, "Noticed a comparison accuracy difference [you may safely ignore this warning]");
         }
      }
   }

   if (show_MLV) {
      curr_MLV += best_change;
   }

   return best_class;
}


/* Define sort order for word unigrams */
static int freq_sort_order(int *in1, int *in2)
{
  return ( (uni[*in1] < uni[*in2]) | (-(uni[*in1] > uni[*in2])));
}


/* Perform one iteration of the clustering algorithm */
static void do_one_iteration(int w_period, int start_word)
{
   UInt w, j, w_index;
   int to;
   FILE *file;
   Boolean pipe_status;
   int total_warnings=0;

   for (w=0; w<W; w++) {
      sort_uni[w] = w;
   }
   if (sort_order == SORT_FREQ) {
      qsort(sort_uni, W, sizeof(int), (int (*) (const void *, const void *)) &freq_sort_order);
   }

   for (w_index=start_word; w_index<W; w_index++) {
      w = sort_uni[w_index];

      if (w_period && w%w_period==0) {
         /* Write recovery file */
         export_classes(1);
         sprintf(tmp, "%.150s.recovery", export_prefix);
         file = FOpen(tmp, NoOFilter, &pipe_status);
         check_file(file, tmp, "do_one_iteration");
         fprintf(file, "Clustering automatic recovery status file\n");
         fprintf(file, "Clustered up to (excluding) word: %d\n", w_index);
         fprintf(file, "Clusters are stored in: %.150s.recovery.cm\n", export_prefix);
         fprintf(file, "Keep unknown word token separate: %d\n", unk_sep?1:0);
         fprintf(file, "Sort order: %s\n", (sort_order==SORT_WMAP)?"WMAP":"FREQ");
         FClose(file, pipe_status);
      }

      if ((w==start_id) || (w==end_id) || (unk_sep && (w==unk_id))) {
         /* We don't want to move this special token, so skip it */
         continue;
      }

      if (uni[w]==0) {
         /* Word is in wordlist but not used, so warn */
         if (total_warnings<10) {
            HError(-17053, "Word '%s' is in word map but not in any gram files", what_is_word(w));
         }
         else if (total_warnings==10) {
            HError(-17053, "Suppressing further word 'x' not in gram file warnings");
         }
         total_warnings++;
         continue;
      }

      if (logfile) {
         if (verbose) {
            fprintf(logfile, "...deciding whether/where to move word %d (of %d - %2.2f%% done) [id=%d]\n",
                    w_index, W, ((float)w_index/(float)W)*100.0, w);
         }
         else {
            fprintf(logfile, "%d [%d] (%2.2f%%):\t", w_index, w, ((float)w_index/(float)W)*100.0);
         }
      }

      curr_class = clMemb[w]; /* Find out what class word is currently in */
      to = choose_class(w);   /* Work out where to move it to */

      if (curr_class != to) {
         if (logfile) {
            if (verbose) {
               fprintf(logfile, "...moving word id %d from class %d to class %d\n", w, curr_class, to);
            }
            else {
               fprintf(logfile, "-> %d\n", to);
            }
            fflush(logfile);
         }

         classes_change(w, to); /* Calculate new unigram and bigram values */

         /* Remove influence of these two classes from MLV values */
         for (j=0; j<N; j++) {
            if (j!=to && j!=curr_class) {
               if (clCnt[to][j])
                  mlv[j] -= ((double)clCnt[to][j]) * log(clCnt[to][j]);
               if (clCnt[j][to])
                  mlv[j] -= ((double)clCnt[j][to]) * log(clCnt[j][to]);
               if (clCnt[curr_class][j])
                  mlv[j] -= ((double)clCnt[curr_class][j]) * log(clCnt[curr_class][j]);
               if (clCnt[j][curr_class])
                  mlv[j] -= ((double)clCnt[j][curr_class]) * log(clCnt[j][curr_class]);
            }
         }

         /* Make change permanent */
         /* Class map */
         clMemb[w] = to;
         /* Class unigram counts */
         clSum[curr_class] -= uni[w];
         clSum[to] += uni[w];
         /* Class bigram counts */
         for (j=0; j<N; j++) {
            if ((j!=curr_class) && (j!=to)) {
               /* (Gw, *) */
               clCnt[curr_class][j] = tmp_c1[j];
               /* (*, Gw) */
               clCnt[j][curr_class] = tmp_c2[j];
               /* (g, *) */
               clCnt[to][j] = tmp_c3[j];
               /* (*, g) */
               clCnt[j][to] = tmp_c4[j];
            }
         }
         /* Exceptions */
         clCnt[curr_class][curr_class] = GwGw;
         clCnt[curr_class][to] = Gwg;
         clCnt[to][curr_class] = gGw;
         clCnt[to][to] = gg;

         /* Recalculate maximum-likelihood values involving this class */
         mlv[to] = 0;
         for (j=0; j<N; j++) {
            if (clCnt[to][j])
               mlv[to] += ((double)clCnt[to][j]) * log(clCnt[to][j]);
            if (to!=j) {
               if (clCnt[j][to])
                  mlv[to] += ((double)clCnt[j][to]) * log(clCnt[j][to]);
            }
         }
         if (clSum[to])
            mlv[to] -= 2*(((double)clSum[to]) * log(clSum[to]));

         mlv[curr_class] = 0;
         for (j=0; j<N; j++) {
            if (clCnt[curr_class][j])
               mlv[curr_class] += ((double)clCnt[curr_class][j]) * log(clCnt[curr_class][j]);
            if (curr_class!=j) {
               if (clCnt[j][curr_class])
                  mlv[curr_class] += ((double)clCnt[j][curr_class]) * log(clCnt[j][curr_class]);
            }
         }
         if (clSum[curr_class])
            mlv[curr_class] -= 2*(((double)clSum[curr_class]) * log(clSum[curr_class]));

         /* Update MLV values for other classes */
         for (j=0; j<N; j++)  {
            if (j!=to && j!=curr_class) {
               if (clCnt[to][j])
                  mlv[j] += ((double)clCnt[to][j]) * log(clCnt[to][j]);
               if (clCnt[j][to])
                  mlv[j] += ((double)clCnt[j][to]) * log(clCnt[j][to]);
               if (clCnt[curr_class][j])
                  mlv[j] += ((double)clCnt[curr_class][j]) * log(clCnt[curr_class][j]);
               if (clCnt[j][curr_class])
                  mlv[j] += ((double)clCnt[j][curr_class]) * log(clCnt[j][curr_class]);
            }
         }
      }
      else {
         if (logfile) {
            if (verbose) {
               fprintf(logfile, "...decided not to move word %d from class %d\n", w, curr_class);
            }
            else {
               fprintf(logfile, "--\n");
            }
         }
         fflush(stdout);
      }

      if (show_MLV && logfile) {
         fprintf(logfile, "   MLV = %f\n", curr_MLV);
      }

#ifdef INTEGRITY_CHECK
      /* Debug: Check our counts still sum correctly */
      check_counts_sum();

      /* Debug: Check our updated MLV counts */
      max_likelihood_check();
#endif
   }

   if (w_period) {
      /* Make sure recovery file reflects end of iteration */
      export_classes(1);
      sprintf(tmp, "%.150s.recovery", export_prefix);
      file = FOpen(tmp, NoOFilter, &pipe_status);
      check_file(file, tmp, "do_one_iteration");
      fprintf(file, "Clustering automatic recovery status file\n");
      fprintf(file, "Clustered up to (excluding) word: all\n");
      fprintf(file, "Clusters are stored in: %.150s.recovery.cm\n", export_prefix);
      fprintf(file, "Keep unknown word token separate: %d\n", unk_sep?1:0);
      fprintf(file, "Sort order: %s\n", (sort_order==SORT_WMAP)?"WMAP":"FREQ");
      FClose(file, pipe_status);
   }
   if (total_warnings>=10) {
      HError(-17053, "A total of %d words were found in the wordmap but not in the gram files", total_warnings);
   }
}


/* Recover from a given recovery file */
void do_recovery(char *fname, int words)
{
   FILE *file;
   char *ptr;
   int   from;
   Boolean pipe_status;
   
   file = FOpen(fname, NoFilter, &pipe_status);
   check_file(file, fname, "do_recovery");
   fgets(tmp, 256, file);
   if (strncmp(tmp, "Clustering automatic", 20)!=0) {
      HError(17013, "This is not a recovery status file");
   }
   fgets(tmp, 256, file);
   ptr = strchr(tmp, ':');
   if (!ptr) {
      HError(17013, "Failure to read current word point from status file");
   }
   ptr++;
   ptr += strspn(ptr, " \t");
   if (strncmp(ptr, "all", 3)==0) {
      from = -1;
   }
   else {
      from = atoi(ptr);
   }
   fgets(tmp, 256, file);
   ptr = strchr(tmp, ':');
   if (!ptr) {
      HError(17013, "Failure to read recovery class map file name from status file");
   }
   ptr++;
   ptr = strtok(ptr, " \t\n");
   import_classmap(ptr, words);

   fgets(tmp, 256, file);
   ptr = strchr(tmp, ':');
   if (!ptr) {
      HError(17013, "Failure to read recovery unknown word status from status file");
   }
   ptr++;
   ptr += strspn(ptr, " \t");
   unk_sep = (*ptr=='1');
   start_class = unk_sep?3:2;

   fgets(tmp, 256, file);
   ptr = strchr(tmp, ':');
   if (!ptr) {
      HError(17013, "Failure to read recovery word sort order status from status file");
   }
   ptr++;
   ptr += strspn(ptr, " \t");
   sort_order = (*ptr=='W')?SORT_WMAP:SORT_FREQ;
   FClose(file, pipe_status);

   if (trace & T_TOP) {
      printf("Continuing from recovered state\n");
   }

   classes_init(words);
   setup_all_counts();
  
   if (trace & T_TOP) {
      printf("Iterations that had been completed: %d\n", export_index);
   }
   export_index++;

   /* Open output log file */
   if (write_logfile) {
      sprintf(tmp, "%.150s.%d.log", export_prefix, export_index);
      logfile = FOpen(tmp, NoOFilter, &pipe_logfile);
      check_file(logfile, tmp, "do_recovery");
   }
   else
      logfile = NULL;

   if (from>=0) {
      do_one_iteration(rec_freq, from);
   }

   if (logfile)
      FClose(logfile, pipe_logfile);

   export_classes(0);
   if (trace & T_EXTRA) {
      printf("Completed iteration which started from recovered state\n");
      if (from == -1) {
         printf("   (no change since recovery state was stored at end of iteration)\n");
      }
   }
}


/* Initialise the values used when calculating the current value of the
   maximum likelihood equation used when clustering */
static void max_likelihood_init(void)
{
   int i, j;

   if (show_MLV)
      curr_MLV=0;

   /* We store all those values from the summation which involve a
      particular class in a value specifically for that class */
   for (i=0; i<N; i++) {
      mlv[i] = 0;
      for (j=0; j<N; j++) {
         if (clCnt[i][j]) {
            mlv[i] += ((double)clCnt[i][j]) * log(clCnt[i][j]);
            if (show_MLV) {
               curr_MLV += ((double)clCnt[i][j]) * log(clCnt[i][j]);
            }
         }
         if (i!=j) {
            if (clCnt[j][i])
               mlv[i] += ((double)clCnt[j][i]) * log(clCnt[j][i]);
         }
      }
      if (clSum[i]) {
         mlv[i] -= 2*(((double)clSum[i]) * log(clSum[i]));
         if (show_MLV) {
            curr_MLV -= 2*(((double)clSum[i]) * log(clSum[i]));
         }
      }
   }
}


#ifdef INTEGRITY_CHECK
/* Check the contents of the maximum likelihood running totals store */
static void max_likelihood_check(void)
{
   int i, j;
   double a;
   char s1[50], s2[50];

  /* We store all those values from the summation which involve a
     particular class in a value specifically for that class */
   for (i=0; i<N; i++) {
      a = 0;
      for (j=0; j<N; j++) {
         if (clCnt[i][j])
            a += ((double)clCnt[i][j]) * log(clCnt[i][j]);
         if (i!=j) {
            if (clCnt[j][i])
               a += ((double)clCnt[j][i]) * log(clCnt[j][i]);
         }
      }
      if (clSum[i])
         a -= 2*(((double)clSum[i]) * log(clSum[i]));

      /* Compare strings, to ignore minor precision differences */
      sprintf(s1, "%f", a);
      sprintf(s2, "%f", mlv[i]);
      if (strcmp(s1, s2)) {
         HError(17097, "max_likelihood_check: MLV for class %d is wrong - %f instead of %f", i, mlv[i], a);
      }
   }
}
#endif


/* Perform a given number of iterations of the clustering algorithm */
void cluster_words(int iterations)
{
   int i;

   for (i=0; i<iterations; i++) {
      /* Also keep a separate iteration count - we do this because it's
         possible to call cluster_words() multiple times from a host
         program, or to continue from an existing classmap */
      export_index++;
      
      if (trace & T_TOP) {
         printf("Beginning iteration %d\n", export_index);
      }
      
      /* Open output log file */
      if (write_logfile) {
         sprintf(tmp, "%.150s.%d.log", export_prefix, export_index);
         logfile = FOpen(tmp, NoOFilter, &pipe_logfile);
         check_file(logfile, tmp, "cluster_words");
      }

      do_one_iteration(rec_freq, 0);

      if (logfile)
         FClose(logfile, pipe_logfile);

      if (trace & T_TOP) {
         printf("Iteration complete\n");
      }
   }
}


/* Setup all class counts, given existing class word map */
void setup_all_counts(void)
{
   register int i, j;

   for (i=0; i<N; i++) {
      clSum[i] = 0;
      for (j=0; j<N; j++) {
         clCnt[i][j] = 0;
      }
   }

   for (i=0; i<W; i++) {
      /* Class unigram counts */
      clSum[clMemb[i]] += uni[i];

      /* Class bigram counts */
      for (j=0; j<forward[i].size; j++) {
         clCnt[clMemb[i]][clMemb[forward[i].bi[j].id]] += forward[i].bi[j].count;
      }
   }

   /* Now initialise the maximisation function class values */
   max_likelihood_init();
}


/* Perform some initial clustering - currently just puts all in one class */
void initial_cluster(void)
{
   register int i;

   for (i=0; i<W; i++) {
      if (unk_sep) {
         clMemb[i] = 3;     /* Put everything in class 3 */
      }
      else {
         clMemb[i] = 2;     /* Put everything in class 2 */
      }
   }

   clMemb[start_id] = 0;
   clMemb[end_id] = 1;
   if (unk_sep) {
      clMemb[unk_id] = 2;
   }

   /* Note that external class numbers are all +1 relative to internal (HLM can't cope
      with class 0 in a class map) */
   if (trace & T_EXTRA) {
      printf("Initial clustering performed: all words in class %d (total count=%d)\n", unk_sep?4:3, sum_of_all_bigram_counts);
      printf ("   (sentence start in class 1; sentence end in class 2%s)\n", unk_sep?"; unknown in class 3":"");
   }
}


/* Define sorting order of words alphabetically, given id */
int id_sort(UInt *in1, UInt *in2)
{
   return strcmp(what_is_word(*in1), what_is_word(*in2));
}


/* Write out a HLM class map file (pass non-zero to write recovery file) */
void export_classes(int recovery)
{
   FILE *out;
   int   i, j, index;
   Boolean pipe_status;

   /* %.150s limits the length of the filename prefix to 150 characters */
   if (recovery) {
      sprintf(tmp, "%.150s.recovery.cm", export_prefix);
   }
   else {
      sprintf(tmp, "%.150s.%d.cm", export_prefix, export_index);
   }

   out = FOpen(tmp, LCMapOFilter, &pipe_status);
   check_file(out, tmp, "export_classes");

   /* Write header */
   if (recovery) {
      fprintf(out, "Name=Classmap_%s_iteration%d\n", export_prefix, export_index-1);
      fprintf(out, "Entries=%d\n", N);
      fprintf(out, "Iterations=%d\n", export_index-1);
   }
   else {
      fprintf(out, "Name=Classmap_%s_iteration%d\n", export_prefix, export_index);
      fprintf(out, "Entries=%d\n", N);
      fprintf(out, "Iterations=%d\n", export_index);
   }
   if (outCMapRaw) {
      fprintf(out, "EscMode=Raw\n");
   }
   else {
      fprintf(out, "EscMode=HTK\n");
   }
   fprintf(out, "\\Classes\\\n");

   for (i=0; i<N; i++) {
      index = 0;
      for (j=0; j<W; j++) {
         if (clMemb[j] == i) {
            class_sort[index] = j;
            index++;
         }
      }
      qsort(class_sort, index, sizeof(UInt),
            (int (*) (const void *, const void *)) &id_sort);

      fprintf(out, "CLASS%d %d %d IN\n", i+1, i+1, index);

      if (outCMapRaw) {
         for (j=0; j<index; j++) {
            fprintf(out, " %s\n", what_is_word(class_sort[j]));
         }
      }
      else {
         for (j=0; j<index; j++) {
            fprintf(out, " %s\n", ReWriteString(what_is_word(class_sort[j]), NULL, ESCAPE_CHAR));
         }
      }
   }
   FClose(out, pipe_status);
}


#ifdef INTEGRITY_CHECK
/* Debugging: Do integrity check on counts - ensure they sum
   to the same value after each loop! */
void check_counts_sum(void)
{
   register int i, j;
   register int a, b;

   a = 0;
   b = 0;

   for (i=0; i<N; i++) {
      a += clSum[i];
      for (j=0; j<N; j++) {
         b += clCnt[i][j];
      }
   }

   if (a != sum_of_all_uni_counts) {
      HError(17096, "check_counts_sum: unigrams now sum to %d, not %d", a, sum_of_all_uni_counts);
   }

   if (b != sum_of_all_bigram_counts) {
      HError(17096, "check_counts_sum: bigrams now sum to %d, not %d", a, sum_of_all_bigram_counts);
   }

   if (a != b) {
      HError(17096, "check_counts_sum: uni and bi totals differ - %d v %d", a, b);
   }
}
#endif


/* Complain about a broken header */
static void invalid_header(void)
{
   HError(17013, "Classmap has broken header - missing '='");
}


/* Import a HLM classmap file. Currently ignores contents of 'EscMode' field. GLM Also non-IN?*/
void import_classmap(char *fname, int numb_words)
{
#define max_line_len 500 
   FILE     *file;               /* Input file handle */
   char      line[max_line_len]; /* Line read buffer */
   int       C;                  /* Current class index */
   int       size;               /* Size of current class */
   int       i;                  /* Loop counter */
   char     *ptr;                /* Text pointer */
   UInt      id;                 /* Word id */
   int       reassigned = 0;     /* Number of reassigned classes */
   int       unexpected = 0;     /* Number of unexpected lines trailing class descriptions */
   Boolean   pipe_status;

   if (trace & T_FILE) {
      printf("Importing classmap '%s'\n", fname);
   }

   W = numb_words;
   clMemb = CNew(&global_stack, W * sizeof(int));
  
   /* Set impossible classmap in order to do integrity check after import */
   for (i=0; i<W; i++) {
      clMemb[i] = -1;
   }
   
   N = 0;
   file = FOpen(fname, LCMapFilter, &pipe_status);
   check_file(file, fname, "import_classmap");
   while (fgets(line, max_line_len, file)) {
      if (strncmp(line, "Entries", 7)==0) {
         ptr = strchr(line, '=');
         if (!ptr) invalid_header();
         ptr++;
         ptr = strtok(ptr, " \t\n");
         N = atoi(ptr);
         if (trace & T_EXTRA) {
            printf("Number of classes = %d\n", N);
         }
      }
      else if (strncmp(line, "Iterations", 10)==0) {
         ptr = strchr(line, '=');
         if (!ptr) invalid_header();
         ptr++;
         ptr = strtok(ptr, " \t\n");
         export_index = atoi(ptr);
      }
      else if (strncmp(line, "EscMode", 7)==0) {
         ptr = strchr(line, '=');
         if (!ptr) invalid_header();
         ptr++;
         ptr = strtok(ptr, " \t\n");
         if (strcmp(ptr, "HTK")==0) {
            if (inCMapRawTrap && inCMapRaw) {
               HError(-17013, "Class map specifies HTK escaping on input but configuration file specifies Raw escaping -- using HTK escaping for input");
            }
            inCMapRaw = FALSE;
         }
         else if (strcmp(ptr, "Raw")==0) {
            if (inCMapRawTrap && !inCMapRaw) {
               HError(-17013, "Class map specifies Raw escaping on input but configuration file specifies HTK escaping -- using Raw escaping for input");
            }
            inCMapRaw = TRUE;
         }
         else {
            HError(17013, "Classmap has unknown escaping of type '%s'", ptr);
         }
         if (!outCMapRawTrap) {
            if (outCMapRaw != inCMapRaw) {
               HError(-17013, "Setting output class map escaping to same format as input class map (%s)", inCMapRaw?"Raw":"HTK");
            }
            outCMapRaw = inCMapRaw;  /* This is common sense */
         }
         if (inCMapRaw != outCMapRaw) {
            HError(-17013, "Input class map escaping and output class map escaping differ (this is not a problem -- this warning is to alert you in case you meant them to be the same)");
         }
      }
      else if (strncmp(line, "\\Classes\\", 9)==0) {
         break;
      }
   }
   if (feof(file)) {
      HError(17013, "Classmap file is corrupt/contains no classes!");
   }

   if (!N) {
      HError(17013, "Corrupt classmap header - must specify number of classes!");
   }

   if (trace & T_EXTRA) {
      printf("Iterations = %d\n", export_index);
   }

   C = 0;
   while (fgets(line, max_line_len, file)) {
      if (C>=N) {
         if (strstr(line, " IN")) {
            HError(17013, "More classes are described than are specified in the header!");
         }
         else {
            ptr = strtok(line, " \t\n");
            if (!ptr)
               continue;
            else {
               HError(-17013, "Warning: ignoring '%s' at end of classmap file", ptr);
            }
         }
      }

      if (strstr(line, " IN")) {
         /* Start of a new class */
         /* Make this class 'C' */
         strtok(line, " \t");
         ptr = strtok(NULL, " \t");
         if (!ptr) {
            HError(17013, "Failure reading class header %d in classmap (no id)", C);
         }
         if (atoi(ptr) != C+1) {
            /* We'll renumber this class */
            reassigned++;
         }
         ptr = strtok(NULL, " \t");
         if (!ptr) {
            HError(17013, "Failure reading class header %d in classmap (no size)", C);
         }
         size = atoi(ptr); /* Read number of words in class */

         for (i=0; i<size; i++) {
            fgets(line, max_line_len, file);
            ptr = strtok(line, " \t\n");
            if (!ptr) {
               /* Warn about the blank line */
               HError(-17013, "Found empty line inside class %d definition", C);
               i--;
               continue;
            }
            /* Unescape word if necessary */
            if (!inCMapRaw) {
               if (strlen(ptr)>255) {
                  HError(17013, "Cannot handle words longer than 255 characters when using HTK escaping (recompile with higher tmp[] buffer size in Cluster.c)");
               }
               ParseString(ptr, tmp);

               /* Put word in class */
               id = get_id_from_word(tmp);
            }
            else {
               /* Put word in class */
               id = get_id_from_word(ptr);
            }

            if (inCMapRaw && strcmp(what_is_word(id), ptr)!=0) {
               HError(17095, "import_classmap: word '%s' is id '%d'; id is '%s'!", ptr, id, what_is_word(id));
            }
            else if (!inCMapRaw && strcmp(what_is_word(id), tmp)!=0) {
               HError(17095, "import_classmap: word '%s' is id '%d'; id is '%s'!", tmp, id, what_is_word(id));
            }

            if (clMemb[id] != -1) {
               HError(17094, "Word '%s' occurs more than once in classmap!", ptr);
            }
            clMemb[id] = C;
         }
         C++;
      }
      else {
         /* Where is class header? It's gone missing! */
         if (strlen(line)>0) {
            if (strchr(line, '\n'))
               *strchr(line, '\n')='\0'; /* Strip linefeed */
            if (strlen(line)==0)
               continue;
            HError(-17013, "Unexpected line '%s' in classmap", line);
            unexpected++;
         }
         if (unexpected>9) {
            HError(17013, "Too many unexpected lines in classmap - aborting now");
         }
         /* Loop round to see if it's coming up next */
      }
   }

   if (C<N) {
      HError(17013, "Less classes are described than are specified in the header!");
   }

   if (trace & T_TOP) {
      if (reassigned) {
         if (reassigned>1) {
            printf("%d class ids were reassigned\n", reassigned);
         }
         else {
            printf("1 class id was reassigned\n");
         }
      }
      else {
         printf("No class ids were reassigned\n");
      }
   }

   /* Check all words were assigned */
   for (i=0; i<W; i++) {
      if (clMemb[i]==-1) {
         HError(17052,"import_classmap: Not all words were assigned to classes");
      }
   }

   FClose(file, pipe_status);
   if (trace & T_FILE) {
      printf("Class map import successful\n");
   }
}


/* Write out p(word | class) probabilities */
void write_word_probs(char *filename)
{
   FILE *out;
   int   i;   /* Loop counter */
   double probability;
   Boolean pipe_status;

   /* These files never use HTK escaping */
   
   out = FOpen(filename, NoOFilter, &pipe_status);
   check_file(out, filename, "write_word_probs");

   /* Write header */
   fprintf(out, "Word|Class probabilities\n");
   fprintf(out, "\n");
   fprintf(out, "Derived from: %s\n", export_prefix);
   fprintf(out, "Number of classes: %d\n", N);
   fprintf(out, "Number of words: %d\n", W);
   fprintf(out, "Iterations: %d\n", export_index);
   fprintf(out, "\n");
   fprintf(out, "%-15s\tClass name\tProbability (log)\n", "Word");

   for (i=0; i<W; i++) {
      if (uni[i]==0) uni[i]=1;
   }

   /* Use tmp_sum1[] to save having to allocate a new array (so can't call
      this from within a class change calculation, but this isn't a problem!)   */
   for (i=0; i<N; i++) {
      tmp_sum1[i] = 0;
   }
   for (i=0; i<W; i++) {
      tmp_sum1[clMemb[i]] += uni[i];
   }

   for (i=0; i<W; i++) {
      probability = (double)uni[i]/((double)tmp_sum1[clMemb[i]]);

      fprintf(out, "%-15s\tCLASS%-4d\t%f\n", what_is_word(i), clMemb[i]+1,
              LOG_NATURAL(probability));

      if (LOG_NATURAL(probability)<-90) {
         printf("prob is %f, discount is %f, uni is %d\n", LOG_NATURAL((double)uni[i]/((double)tmp_sum1[clMemb[i]])), mlv[clMemb[i]], uni[i]);
      }
   }
   FClose(out, pipe_status);
   if (trace & T_FILE) {
      printf("Wrote word|class probabilities to '%s'\n", filename);
   }
}


/* Write out p(word | class) counts */
void write_word_counts(char *filename)
{
   FILE *out;
   int   i;   /* Loop counter */
   Boolean pipe_status;

   /* These files never use HTK escaping */

   /* Open output file */
   out = FOpen(filename, NoOFilter, &pipe_status);
   check_file(out, filename, "write_word_counts");

   /* Write header */
   fprintf(out, "Word|Class counts\n");
   fprintf(out, "\n");
   fprintf(out, "Derived from: %s\n", export_prefix);
   fprintf(out, "Number of classes: %d\n", N);
   fprintf(out, "Number of words: %d\n", W);
   fprintf(out, "Iterations: %d\n", export_index);
   fprintf(out, "\n");
   fprintf(out, "%-15s\tClass name\tCount\n", "Word");
   
   for (i=0; i<W; i++) {
      fprintf(out, "%-15s\tCLASS%-4d\t%d\n", what_is_word(i), clMemb[i]+1, uni[i]);
   }

   FClose(out, pipe_status);
   if (trace & T_FILE) {
      printf("Wrote word|class counts to '%s'\n", filename);
   }
}


/* Specify whether to keep the unknown word in its own solo-member class or not */
void classes_keep_unk_separate(int keep_separate)
{
   unk_sep = (Boolean) keep_separate;
   start_class = unk_sep?3:2;
}


/* Pass in start, end and unknown word ids */
void set_ids(int start, int end, int unk)
{
   start_id = start;
   end_id = end;
   unk_id = unk;
}


/* This set of functions takes unigram counts, stores them, and
   then allows them to be retrieved. It simply allocates a count
   for each possible word id, since they are allocated in a
   continuous block.
*/

/* Initialise this unigram storage module */
void unigram_init(int words)
{
   max_words = words;
   uni = CNew(&global_stack, words * sizeof(unigram));
   sum_of_all_uni_counts = 0;
}


/* Add a unigram */
void unigram_add(NGram ng, int count)
{
   if (ng[0]>=max_words) {
      /* Something's gone wrong */
      HError(17093, "unigram_add: Found a word id higher than the base+number of words - word ids are expected to be allocated in an unbroken chunk\n[Current unigram is (%d); number of words is %d]", ng[0], max_words);
   }
   uni[ng[0]] += count;
   sum_of_all_uni_counts += count;
   return;
}


/* Read a unigram */
UInt unigram_read(UInt id)
{
#ifdef INTEGRITY_CHECK
      if ((id<0) || (id>=max_words)) {
         HError(17092, "unigram_read: attempt to read unigram outside bounds (%d; %d words)", id, max_words);
      }
#endif
   return uni[id];
}


/* This section contains functions to store a sequence of bigrams - they must
   be sequenced before passing to this code, since it relies on the input
   being sorted.
   Both forward and backward word to all bigrams look-up tables are built,
   so given either u or v from a bigram (u,v) then the set of all (u,*) or
   (*,v) can be found. */

/* Grab some space from our current local storage block */
static void *get_space(int size)
{
   static void* ptr;

   /* Test against our not-worth-using cut-off point */
   if (size>block_cut_off)
      return New(&global_stack, size);

   /* Use New() again if necessary to get a new block */
   if (((int)block+(int)size) >= (int)block_end) {
      block = New(&global_heap, block_grab_size);
      block_end = (void *) ((int)block+(int)block_grab_size);
   }

   /* Hand back the next free space */
   ptr = block;
   block = (void*) ((int) block + (int) size);     /* Next free byte */
   block = (void*) ((((int)block)+3) & (~(int)3)); /* Word-align */

   return ptr;
}


/* Add a bigram */
void bigram_add(NGram ng, int count)
{
   bi_count *ptr;
   int       space_used;

   if ((ng[0]>=max_words) || (ng[1]>=max_words)) {
      /* Something's gone wrong */
      HError(17093, "bigram_add: Found a word id higher than the base+number of words - all word ids are expected to be allocated in an unbroken chunk.\n[Current bigram is (%d,%d). Number of words is %d]", ng[0], ng[1], max_words);
   }

   /* Keep backward count */
   backward[ng[1]].size++;

   if (ng[0] == last_word) {
      /* Make sure there's room in the buffer */
      if (store_idx >= curr_bistore_size) {
         /* Expand bigram buffer store to cope */
         curr_bistore_size += bigram_buffer_grow;
         if (trace & T_MEM) {
            printf("Expanding bigram read buffer to %d entries\n", curr_bistore_size);
         }
         store = realloc(store, curr_bistore_size*sizeof(bi_count));
      }

      /* Store in buffer */
      store[store_idx].id = ng[1];
      store[store_idx].count = count;
      store_idx++;
      return;
   }

   /* Otherwise we must have just gone on to a new word, so keep the old
      details */
   forward[last_word].size = store_idx;
   space_used = store_idx*sizeof(bi_count);
   ptr = get_space(space_used);
   memcpy(ptr, store, space_used);
   forward[last_word].bi = ptr;

   /* And go on to the next entry */
   last_word = ng[0];
   store[0].id = ng[1];
   store[0].count = count;
   store_idx = 1;
}


/* Call when all bigrams have been passed in */
void bigram_added_all(void)
{
   bi_count *ptr;
   int       space_used;
   int       i, j, backward_id;

   /* Store last set of details */
   forward[last_word].size = store_idx;
   space_used = store_idx*sizeof(bi_count);
   ptr = get_space(space_used);
   memcpy(ptr, store, space_used);
   forward[last_word].bi = ptr;
   free(store);

   sum_of_all_bigram_counts = 0;

   /* Generate backward lookup table */
   if (trace & T_EXTRA) {
      printf("Building bigram backward lookup table...");
      fflush(stdout);
   }
   /* Allocate required storage space */
   for (i=0; i<max_words; i++) {
      backward[i].bi = get_space(backward[i].size * sizeof(bi_count));
      backward[i].size = 0; /* Reset to use as counter when building data */
   }

   /* Run through all forward data, copying into backward array */
   for (i=0; i<max_words; i++) {
      for (j=0; j<forward[i].size; j++) {
         backward_id = forward[i].bi[j].id;

         backward[backward_id].bi[backward[backward_id].size].id = i;
         backward[backward_id].bi[backward[backward_id].size].count
                                            = forward[i].bi[j].count;
         backward[backward_id].size++;
         sum_of_all_bigram_counts += forward[i].bi[j].count;
      }
   }
   if (trace & T_EXTRA) {
      printf(" done\n");
   }
}


/* Must be called before almost any other function in this file will work */
void bigram_init(int words) /* Pass ->used field from word-map */
{
   max_words = words;
   forward = CNew(&global_stack, words * sizeof(bigrams));
   backward = CNew(&global_stack, words * sizeof(bigrams));
   if (trace & T_MEM) {
      printf("Bigram store for %d words created\n", words);
   }
   last_word = 0;
   store_idx = 0;
   curr_bistore_size = initial_bigram_buffer;
   store = calloc(initial_bigram_buffer, sizeof(bi_count));
   if (trace & T_MEM) {
      printf("Bigram read buffer of %d entries created\n", initial_bigram_buffer);
   }
}


/* Main program control function */
int main(int argc, char *argv[])
{
   char *s;
   float weight;   /* used when loading gram files */
   char *filename; /* used when loading gram files */
   int   iterations=1, loop;
   char *init_cmap = NULL;
   char *recover_from = NULL;
   char *write_classprobs = NULL;
   char *write_classcounts = NULL;
   Boolean read_gram_files=FALSE; /* Has the user passed any gram files? */
   Boolean set_classes = FALSE, loaded_map = FALSE; /* Check for -c and -l */
   Boolean keep_unk_sep = FALSE; /* Was -k passed? */
   Boolean passed_unk = FALSE; /* Unknown word was passed in */
   int start_word_id, end_word_id, unknown_word_id;
   int numb_classes, min_classes;
   char *ptr, *ptr2; /* temp results */

   /* Initialise HTK/HLM modules */
   InitShell(argc, argv, Cluster_version, Cluster_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitLUtil();
   InitWMap();
   InitGBase();
   SetConfParms();

   /* Default start, end and unknown words */
   strcpy(sent_start, DEF_STARTWORD);
   strcpy(sent_end, DEF_ENDWORD);
   strcpy(unknown_w, DEF_UNKNOWNNAME);

   /* Default number of classes */
   numb_classes = classes_get_default();

   /* Parse command line */
   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0)
      Exit(EXIT_FAILURE);

   /* Create a global stack and heap */
   CreateHeap(&global_stack, "Clusterer stack", MSTAK, 1, 0.0, 8192, 8192);
   CreateHeap(&global_heap, "Clusterer heap", MHEAP, block_grab_size, 0.0, 1, 1);

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s) !=1 )
         HError(17019, "Cluster: Bad switch %s; must be single letter",s);
      switch(s[0]) {
         case 'c':
            if (NextArg()!=INTARG)
               HError(17019,"Cluster: number of categories expected for -c");
	    numb_classes = GetIntArg();
            classes_set_number(numb_classes);
	    set_classes = TRUE;
            break;
         case 'i':
            if (NextArg()!=INTARG)
               HError(17019,"Cluster: number of iterations expected for -i");
            iterations = GetIntArg();
            break;
          case 'r':
            if (NextArg()!=INTARG)
               HError(17019,"Cluster: recovery export frequency expected for -r");
            rec_freq = GetIntArg();
            break;
         case 'm':
	    classes_showMLV(1);
	    break;
         case 'o':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: output filename prefix expected for -o");
            set_output_prefix(GetStrArg());
            break;
         case 'p':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: output filename expected for -p");
            write_classprobs = GetStrArg();
            break;
         case 'q':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: output filename expected for -q");
            write_classcounts = GetStrArg();
            break;
         case 'l':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: output filename prefix expected for -l");
            init_cmap = GetStrArg();
	    loaded_map = TRUE;
            break;
         case 's':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: sentence start word expected for -s");
            strcpy(sent_start, GetStrArg());
            break;
         case 't':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: sentence end word expected for -t");
            strcpy(sent_end, GetStrArg());
            break;
         case 'u':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: unknown word token expected for -u");
            strcpy(unknown_w, GetStrArg());
	    passed_unk = TRUE;
            break;
         case 'x':
            if (NextArg()!=STRINGARG)
               HError(17019,"Cluster: recovery filename expected for -x");
            recover_from = GetStrArg();
            break;
         case 'w':
            if (NextArg()!=STRINGARG)
               HError(17019, "Cluster: wordmap sort order expected for -w");
            strcpy(tmp, GetStrArg());
            for (ptr=tmp; *ptr!=0; *ptr=toupper(*ptr), ptr++);
            if (strcmp(tmp, "WMAP")==0) {
               sort_order = SORT_WMAP;
            }
            else if (strcmp(tmp, "FREQ")==0) {
               sort_order = SORT_FREQ;
            }
            else {
               HError(17019, "Cluster: -w expects either WMAP or FREQ");
            }
            break;
         case 'k':
	    classes_keep_unk_separate(TRUE);
	    keep_unk_sep = TRUE;
            break;
         case 'v':
	    verbose = TRUE;
            break;
         case 'n':
	    write_logfile = !write_logfile;
            break;
         case 'T':
            trace = GetChkedInt(0,017,s); break;
         default:
            HError(17019,"Cluster: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(17019, "Cluster: word map file name expected");
   CreateWordMap(GetStrArg(), &wmap, 0);


   min_classes = 4 + (keep_unk_sep?1:0); /* Minimum number of classes */
   if (loaded_map && set_classes) {
      HError(-17019, "Ignoring -c option: when combined with -l the number of classes in the existing map must be used");
   }
   else if (numb_classes < min_classes) {
      HError(17019, "It doesn't make sense to specify less than %d classes -\n    %d classes are reserved, and you need at least 2 more", min_classes, min_classes-2);
   }

   /* See if start and end word occur in the data */
   if (!GetLabId(sent_start, FALSE)) {
      HError(17051, "Sentence start token '%s' not in word list");
   }
   if (!GetLabId(sent_end, FALSE)) {
      HError(17051, "Sentence end token '%s' not in word list");
   }
   /* We can't keep the unknown word in its own class if one wasn't passed */
   if (!GetLabId(unknown_w, FALSE) && keep_unk_sep) {
      HError(17051, "Unknown word token '%s' not in word list and -k passed", unknown_w);
   }

   /* And generate a sensible warning if necessary: */
   if (!GetLabId(unknown_w, FALSE) && passed_unk) {
      HError(-17051, "Unknown word token '%s' was explicitly given with -u, but does not occur in the word map", unknown_w);
   }

   start_word_id = GetMEIndex(&wmap, (((MapEntry *)(GetLabId(sent_start, FALSE)->aux))->ndx));
   end_word_id = GetMEIndex(&wmap, (((MapEntry *)(GetLabId(sent_end, FALSE)->aux))->ndx));

   if (keep_unk_sep) {
      unknown_word_id = GetMEIndex(&wmap, (((MapEntry *)(GetLabId(unknown_w, FALSE)->aux))->ndx));
   }
   else {
      unknown_word_id = 0;
   }

   set_ids(start_word_id, end_word_id, unknown_word_id);

   /* If we're doing no iterations we want to ignore the given filename
      prefix and use the one from the classmap - this way we'll write the
      correct information into the saved probabilities file header */
   if (iterations==0 && init_cmap) {
      ptr = strrchr(init_cmap, '.');
      if (ptr) {
         *ptr = '\0';
         ptr2 = strrchr(init_cmap, '.');
         if (ptr2) {
            *ptr2 = '\0';
            set_output_prefix(init_cmap);
            *ptr2 = '.';
         }
         else
            set_output_prefix(init_cmap);
         *ptr = '.';
      }
      else {
         set_output_prefix(init_cmap);
      }
   }

   if (trace & T_FILE) {
      printf("Wordmap loaded - %d words\n", wmap.used);
   }
   unigram_init(wmap.used);
   bigram_init(wmap.used);

   /* Add input gram files to input set */
   if (trace & T_TOP)
      printf("Preparing input gram set\n");
   CreateHeap(&imem, "inputset", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&imem2, "inputset2", MSTAK, 1, 0.0, 1000, 1000);
   CreateInputSet(&imem, &wmap, &inset);
   CreateInputSet(&imem2, &wmap, &inset2);
   weight = 1.0;
   while (NextArg() == STRINGARG || NextArg() == FLOATARG) {
      if (NextArg() == FLOATARG) {
         weight = GetFltArg();
      }
      if (weight==0.0 || weight<-10000.0 || weight>10000.0) {
         HError(17019, "Improbable gram file weight (%.4f)", weight);
      }
      if (NextArg()!=STRINGARG) {
         HError(17019,"Gram file name expected");
      }
      filename = GetStrArg();
      AddInputGFile(&inset, filename, weight);
      AddInputGFile(&inset2, filename, weight);
      read_gram_files = TRUE;

      if (trace & T_TOP)
         printf("Input gram file %s added (weight=%f)\n", filename, weight);
   }

   if (!read_gram_files) {
      HError(17019, "No gram files passed");
   }

   LoadBiGrams();
   LoadUniGrams();

   bigram_added_all();

   DeleteHeap(&imem);
   DeleteHeap(&imem2);

   if (init_cmap) {
      import_classmap(init_cmap, wmap.used);
   }
   else if (recover_from) {
      do_recovery(recover_from, wmap.used);
   }

   /* Allocate memory and compute bigram pair arrays */
   if (!recover_from) {
      classes_init(wmap.used);

      /* Perform default initial clustering */
      if (!init_cmap) {
         initial_cluster();
      }

      /* Calculate initial counts required */
      setup_all_counts();
   }

   /* Run clustering algorithm */
   for (loop=0; loop<iterations; loop++) {
      cluster_words(1);
      export_classes(0);
   }

   if (write_classprobs) {
      write_word_probs(write_classprobs);
   }
   if (write_classcounts) {
      write_word_counts(write_classcounts);
   }

   if (trace & T_TOP) {
      printf("Cluster completed successfully\n");
   }

   if (trace & T_MEM) {
      PrintAllHeapStats();
   }

   ResetHeap(&global_stack);
   ResetHeap(&global_heap);
   DeleteHeap(&global_stack);
   DeleteHeap(&global_heap);

   Exit(EXIT_SUCCESS);

   return EXIT_SUCCESS;
}
