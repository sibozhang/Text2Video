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
/*      File: LNewMap.c: create an empty word map file         */
/* ----------------------------------------------------------- */

char *lnewmap_version = "!HVER!LNewMap:   3.4.1 [CUED 12/03/09]";
char *lnewmap_vc_id = "$Id: LNewMap.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

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

   nParm = GetConfig("LNewMap", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: LNewMap [options] name mapfn\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -e esc  Set escape mode to 'esc'             RAW\n");
   printf(" -f fld  Add field 'fld' to Fields header     ID\n");
   PrintStdOpts("");
   printf("\n\n");
}
   
int main(int argc, char *argv[])
{
   char *s;
   Boolean pipe_status;
   FILE *file;
   char *wmFile, *wmName;
   char *escape="RAW";
   char fields[1024];

   InitShell(argc,argv,lnewmap_version,lnewmap_vc_id);
   InitMem();
   strcpy(fields, "ID");

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s) !=1 )
         HError(17219,"LNewMap: Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'e':
	    if (NextArg() != STRINGARG)
	      HError(17219,"LNewMap: String expected for -e option");
            escape = GetStrArg();
            break;
         case 'f':
	    if (NextArg() != STRINGARG)
	      HError(17219,"LNewMap: String expected for -f option");
            strcat(fields, ",");
            strcat(fields, GetStrArg());
            if (strlen(fields)>1000) HError(17299, "LNewMap: Too many fields");
            break;
         case 'T':
            trace = GetChkedInt(0,077,s);
            break;
         default:
            HError(17219,"LNewMap: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(17219,"LNewMap: word map 'Name' entry expected");
   wmName = GetStrArg();
   if (NextArg()!=STRINGARG)
      HError(17219,"LNewMap: word map file name expected");
   wmFile = GetStrArg();

   file = FOpen(wmFile, NoOFilter, &pipe_status);
   if (!file) HError(17211, "Can't open file '%s'", wmFile);
   fprintf(file, "Name    = %s\n", wmName);
   fprintf(file, "SeqNo   = 0\n");
   fprintf(file, "Entries = 0\n");
   fprintf(file, "EscMode = %s\n", escape);
   fprintf(file, "Fields  = %s\n", fields);
   fprintf(file, "\\Words\\\n");
   FClose(file, pipe_status);

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
} 




