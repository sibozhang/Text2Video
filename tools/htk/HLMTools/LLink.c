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
/* main authors: Valtcho Valtchev, Steve Young,                */
/*               Julian Odell, Gareth Moore                    */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*          1994-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*      File: LFile.c: link LM files to make a class LM        */
/* ----------------------------------------------------------- */

char *llink_version = "!HVER!LLink:   3.4.1 [CUED 12/03/09]";
char *llink_vc_id = "$Id: LLink.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"


/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001              /* Top Level tracing */

/* ---------------------- Global Variables ----------------------- */

  
/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LLink", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: LLink [options] word-classLMfile class-classLMfile outLMfile\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -c      Force model to say 'counts'          as per w|c file\n");
   printf(" -p      Force model to say 'probabilities'   as per w|c file\n");
   printf(" -s      Merge into one file, not three       off\n");
   PrintStdOpts("");
   printf("\n\n");
}


#define CFOpen(a,b,c) check_file(FOpen(a,b,c),a)
FILE *check_file(FILE *file, char *fname)
{
   if (!file)
      HError(17111, "Can't open file '%s'", fname);
   return file;
}


int main(int argc, char *argv[])
{
   char *s;
   Boolean pipe_status, ps2, ps3;
#define MLL 1024
   char type[25], lineBuf[MLL];
   FILE *file, *write, *ng;
   char *wcFile, *ccFile, *outFile;
   Boolean force_counts=FALSE, force_probs=FALSE, single_file=FALSE;
   Boolean read_ok=FALSE;
   int read;
   unsigned int *buffer;

   InitShell(argc,argv,llink_version,llink_vc_id);
   InitMem();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s) !=1 )
         HError(17119,"LLink: Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'c':
            force_counts = TRUE;
            break;
         case 'p':
            force_probs = TRUE;
            break;
         case 's':
            single_file = TRUE;
            break;
         case 'T':
            trace = GetChkedInt(0,077,s);
            break;
         default:
            HError(17119,"LLink: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(17119,"LLink: word|class LM filename expected");
   wcFile = GetStrArg();
   if (NextArg()!=STRINGARG)
      HError(17119,"LLink: class|class history LM filename expected");
   ccFile = GetStrArg();
   if (NextArg()!=STRINGARG)
      HError(17119,"LLink: output LM filename expected");
   outFile = GetStrArg();

   if (force_counts && force_probs) {
      HError(17119,"LLink: cannot force both probabilities and counts");
   }

   if (!single_file) {
      file = CFOpen(wcFile, LangModFilter, &pipe_status);
      fscanf(file, "Word|Class %20s", type);
      if (strcmp(type, "probabilities") && strcmp(type, "counts")) {
         HError(17119, "Word|Class file is of an unknown type");
      }
      FClose(file, pipe_status);

      /* Also check the other file opens */
      file = CFOpen(ccFile, LangModFilter, &pipe_status);
      /* Pipe unfortunately is returned by FOpen even if not found,
         so should really try and read a byte to make sure it is a file */
      FClose(file, pipe_status);

      if (force_counts) {
         strcpy(type, "counts");
      }
      else if (force_probs) {
         strcpy(type, "probabilities");
      }

      file = CFOpen(outFile, LangModOFilter, &pipe_status);
      fprintf(file, "Class-based LM\n");
      fprintf(file, "Word|Class %s: %s\n", type, wcFile);
      fprintf(file, "Class|Class counts: %s\n", ccFile);
      FClose(file, pipe_status);
   }
   else {
      /* Single file */
#define BUF_SIZE 1024*4096
      buffer = New(&gstack, sizeof(unsigned char) * BUF_SIZE);

      write = CFOpen(outFile, LangModOFilter, &pipe_status);
      fprintf(write, "CLASS MODEL\n");

      file = CFOpen(wcFile, LangModFilter, &ps2);
      fscanf(file, "Word|Class %20s\n", type);
      if (strcmp(type, "probabilities") && strcmp(type, "counts")) {
         HError(17119, "Word|Class file is of an unknown type");
      }
      if (force_counts) {
         strcpy(type, "counts");
      }
      else if (force_probs) {
         strcpy(type, "probabilities");
      }
      fprintf(write, "Word|Class %s\n\n", type);

      while (fgets(lineBuf, MLL, file)) {
         if (strncmp(lineBuf, "Word", 4)==0) {
            read_ok = TRUE;
            break;
         }
         fputs(lineBuf, write);
      }
      if (!read_ok) HError(17119, "Word|Class file is not in expected format (no terminator)");
      fprintf(write, "Class n-gram counts follow; word|class component is at end of file.\n");

      /* Now copy entire class n-gram component across */
      read_ok = FALSE;
      ng = CFOpen(ccFile, LangModFilter, &ps3);
      while ((read = fread(buffer, sizeof(unsigned char), BUF_SIZE, ng))) {
         read_ok = TRUE;
         fwrite(buffer, sizeof(unsigned char), read, write);
      }
      FClose(ng, ps3);
      if (!read_ok) HError(17119, "No data found for class n-gram counts");

      /* And finally write main body of the first, word|class, file */
      while (fgets(lineBuf, MLL, file)) {
         fputs(lineBuf, write);
      }
      FClose(file, pipe_status);
      FClose(write, ps3);

      Dispose(&gstack, buffer);
   }

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
} 




