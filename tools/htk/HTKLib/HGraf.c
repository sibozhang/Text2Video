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
/*         File: HGraf.X.c:  HGraf for X-Windows               */
/* ----------------------------------------------------------- */

char *hgraf_version = "!HVER!HGraf(X):   3.4.1 [CUED 12/03/09]";
char *hgraf_vc_id = "$Id: HGraf.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

/*
   This is the X Windows implementation of HGraf.  It is server
   independent except for two areas.
   
   a) a set of fonts in sizes 8,9,10,12,14,15,16,19,20,24 should
      be provided by your server.  This distribution attempts to
      load these using the pattern *-Medium-R-Normal-*.  If your
      X server does not provide fonts matching this pattern you
      will need to modify the FontNm initialiser below.
      
   b) the set of X colours used for the 16 HGraf colours may need
      to be modified to give the best results on your machine.
      See the XColArray initialiser below.
      
   It may be necessary to modify this source and/or change compiler
   options in order to compile this version of HGraf.  For example,
   under HP-UX 8.07 you would need to add the following immediately
   after this comment
   
      #define _HPUX_SOURCE

   The include search path also needs to be altered by using
   
       -I/usr/include/X11R4 
       
   compiler option
      
*/

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HGraf.h"

/*
   Enable use of timer to add hysteresis to button response.
   This should work for most UNIX systems but the location of
   <time.h> may vary from system to system.

   #define USE_TIMER
*/

#ifdef USE_TIMER
#include <sys/time.h>
#define _BSD_SIGNALS
#include <signal.h>
#endif
 
#include <X11/Xlib.h>      /* the X11 stuff makes string.h also available */
#include <X11/Xutil.h>
#include <X11/Xos.h>
#define XK_MISCELLANY      /* use miscellaneous keysym defs */
#include <X11/keysymdef.h>


#define MAX_GC 4  /* we need 1 GC for each transfer mode since changing
                     the transfer mode in the current GC is unreliable on
                     some X-servers that we have tested */

static MemHeap btnHeap;      /* heap for HButton structures */

/* Global X Stuff */
static Display       *theDisp;                           
static Window        rootW, theWindow;                   
static int           theScreen;                          
static unsigned int  ncells, dispWIDE, dispHIGH, dispDEEP;
static Colormap      theCmap;
static GC            theGC;
static GC            gcs[MAX_GC];
static unsigned long black, white;
static Visual        *theVisual;
static XEvent        report;
static XSizeHints    hints;
static Boolean       winCreated = FALSE;

static int colours[MAX_COLOURS];
static int greys[MAX_GREYS];
 
static char  *XColArray[] = {
   "white","yellow","orange","red",
   "plum", "purple","blue","lightblue", 
   "darkgreen","palegreen", "brown","tan", 
   "lightgray", "gray","darkslategray", "black" 
};

static int   TFuncX11[4] = {
   GXcopy, GXor, GXxor, GXinvert
};

/* --------------------------- Initialisation ---------------------- */

static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;
static int trace = 0;                   /* Just for consistency */

/* EXPORT->InitGraf: initialise memory and configuration parameters */
void InitGraf(void)
{
   int i;
   
   Register(hgraf_version,hgraf_vc_id);
   nParm = GetConfig("HGRAF", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

/*------------------- Font Handling Routines -----------------------*/

#define NO_OF_FONTS 10
#define FONTS_AVAILABLE 10

static char *FontNm[NO_OF_FONTS] = {
   "*-Medium-R-Normal-*-8-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-9-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-10-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-12-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-14-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-15-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-16-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-19-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-20-*-*-*-*-*-*-*",
   "*-Medium-R-Normal-*-24-*-*-*-*-*-*-*" };

static int FontSize[NO_OF_FONTS] = {8, 9, 10, 12, 14, 15, 16, 19, 20, 24};

static XFontStruct  *DefaultFont, *CurrentFont, *FontInfo[NO_OF_FONTS];

#define FONT1 "-*-lucida-medium-r-*-*-12-*"
#define FONT2 "-*-helvetica-medium-r-*-*-12-*"
#define FONT3 "6x13"

/* InstallFonts: install the HGraf font set */
static void InstallFonts(void)
{
   int  i;

   /* load user fonts */
   for (i=0; i < FONTS_AVAILABLE; i++) 
      if ((FontInfo[i] = XLoadQueryFont(theDisp, FontNm[i])) == NULL) 
         HError(-6870, "InstallFonts: Cannot load font %s", FontNm[i]);
   /* load the default font */
   if ((DefaultFont = XLoadQueryFont(theDisp, FONT1))==NULL  && 
       (DefaultFont = XLoadQueryFont(theDisp, FONT2))==NULL  && 
       (DefaultFont = XLoadQueryFont(theDisp, FONT3))==NULL) 
      HError(6870, "InstallFonts: Cannot load default font");
}

/* ----------------------------- Event Handling --------------------------------- */

/* DecodeKeyPress: decode the given keypress into char+modifier */
static void DecodeKeyPress(XKeyEvent *xkev, HEventRec *hev)
{
   char buf[20];
   int n;
   KeySym key;
   XComposeStatus compose;
   
   n = XLookupString(xkev,buf,20,&key,&compose);
   hev->c = buf[0];
   switch (key) {
   case XK_Shift_L:
   case XK_Shift_R:
      hev->ktype = SHIFTKEY;
      break;
   case XK_Control_L:
   case XK_Control_R:
      hev->ktype = CONTROLKEY;
      break;
   case XK_Meta_L:
   case XK_Meta_R:
   case XK_Alt_L:
   case XK_Alt_R:
      hev->ktype = COMMANDKEY;
      break;
   case XK_Return:
   case XK_KP_Enter:
      hev->ktype = ENTERKEY;
      break;
   case XK_Escape:
      hev->ktype = ESCKEY;
      break;
   case XK_BackSpace:
   case XK_Delete:
      hev->ktype = DELKEY;
      break;  
   default:
      hev->ktype = NORMALKEY;
   }
}

/* EXPORT->HGetEvent: return next relevant event in event queue */
HEventRec HGetEvent(Boolean anyEvent, void (*action)(void))
{
   XEvent xev;
   HEventRec hev;
   Boolean found,dummy;

   XFlush(theDisp); found = FALSE;
   do {
      if(XEventsQueued(theDisp, QueuedAfterFlush) > 0 || action==NULL){ 
         XNextEvent(theDisp, &xev);
         found = TRUE;
         if (xev.xany.window==theWindow || anyEvent){
            switch (xev.type) {
            case ButtonPress: 
               hev.event = HMOUSEDOWN;
               hev.x = xev.xbutton.x;
               hev.y = xev.xbutton.y;
               break;
            case ButtonRelease:
               hev.event = HMOUSEUP;
               hev.x = xev.xbutton.x;
               hev.y = xev.xbutton.y;
               break;
            case MotionNotify:
               hev.event = HMOUSEMOVE;
               hev.x = xev.xmotion.x;
               hev.y = xev.xmotion.y;
               break;
            case KeyPress:
               hev.event = HKEYPRESS;
               hev.x = xev.xkey.x;
               hev.y = xev.xkey.y;
               DecodeKeyPress(&(xev.xkey), &hev);
               break;
            case KeyRelease:
               hev.event = HKEYRELEASE;
               hev.x = xev.xkey.x;
               hev.y = xev.xkey.y;
               DecodeKeyPress(&(xev.xkey), &hev);
               break;
            case Expose:
               if (xev.xexpose.count==0)
                  hev.event = HREDRAW;
               else
                  found = FALSE;
               break;
            default:
               found = FALSE;
            }
         }
      } else if (action!=NULL){
         (*action)();
         XFlush(theDisp);
         /* execute a round-robin command to make sure that */
         /* client doesnt get too far ahead of the server */
         dummy = HMousePos(&hev.x,&hev.y);
      }
   } while (!found);
   return hev; 
}

/* EXPORT->HEventsPending: Return number of events pending */
int HEventsPending(void)
{
   return XEventsQueued(theDisp, QueuedAfterFlush);
}

/* EXPORT->HMousePos: return mouse pos in x, y, returns TRUE if 
   the pointer is on the window */
Boolean HMousePos(int *x, int *y)
{
   Window root,child;
   int rx,ry;
   unsigned int keys;

   return (Boolean)
      XQueryPointer(theDisp, theWindow, &root, &child, &rx, &ry, x, y, &keys);
}

/* EXPORT: IsInRect: return TRUE iff (x,y) is in the rectangle (x0,y0,x1,y1) */ 
Boolean IsInRect(int x, int y, int x0, int y0, int x1, int y1)
{
   return (x >= x0 && x<=x1 && y >= y0 && y <= y1);
}


/* ------------------------- Colour Handling ------------------------------ */

#define GSTP 4

static void InstallColours(void)
{
   int pixVal=0;
   int c, f, ggap, steps[GSTP] = {8, 4, 2, 1};
   XColor greyDef, whiteDef, blackDef, colourDef;
   short RGBval, step;

   /* initialise the grey levels/colours depending on the number of planes */
   for (c = 0; c < MAX_COLOURS; c++){
      /* get colour from the X11 database */
      if (!XParseColor(theDisp, theCmap, XColArray[c], &colourDef))
         HError(6870,"InstallColours: Colour name %s not in X11 database", 
                XColArray[c]);
      if (!XAllocColor(theDisp, theCmap, &colourDef))
         HError(-6870,"InstallColours: Cannot allocate colour %s", XColArray[c]);
      else
         pixVal = colourDef.pixel;
      colours[c] = pixVal;
   }
   if (dispDEEP == 1){
      /* map all grey levels onto b/w */
      for (c = 0; c < MAX_GREYS/2; c++)
         greys[c] = white;
      for (c = MAX_GREYS/2; c < MAX_GREYS; c++)
         greys[c] = black;
   } else {
      /* then the grey levels */
      whiteDef.pixel = white; XQueryColor(theDisp, theCmap, &whiteDef);
      blackDef.pixel = black; XQueryColor(theDisp, theCmap, &blackDef);
      ggap = ((int)(whiteDef.red - blackDef.red))/MAX_GREYS;
      for (f = 0; f < GSTP; f++){
         step = steps[f]*ggap;
         for (c = 0; c < (MAX_GREYS/steps[f]); c++){
            RGBval = whiteDef.red - c*step; 
            greyDef.red = RGBval;
            greyDef.green = RGBval;
            greyDef.blue  = RGBval;
            if (!XAllocColor(theDisp, theCmap, &greyDef))
               HError(-6870, "InstallColours: Cannot allocate grey level %d", 
                      c*steps[f]);
            else
               pixVal = greyDef.pixel;
            greys[c] = pixVal;
         } 
      }
   }
}

/* EXPORT-> HSetColour: Set current colour to c */
void HSetColour(HColour c)
{
   XferMode  xf;

   for (xf = GCOPY; xf < GINVERT; xf=(XferMode) (xf+1))   /* change all GCs except GINVERT*/
      XSetForeground(theDisp, gcs[(int) xf], colours[(int) c]);
}

/* EXPORT-> HSetGrey: Set current colour to grey level g */
void HSetGrey(int g)
{
   XferMode  xf;

   for (xf = GCOPY; xf < GINVERT; xf=(XferMode) (xf+1))   /* change all GCs except GINVERT*/
      XSetForeground(theDisp, gcs[(int) xf], greys[g]);
}

/* ------------------------ Drawing Primitives ---------------------------- */

/* CheckCorners: make sure (x0,y0) is north-west of (x1,y1) */
static void CheckCorners(int *x0, int *y0, int *x1, int *y1)
{
   int a,b,c,d;
   
   if (*x0<*x1) {a=*x0; c=*x1;} else {a=*x1; c=*x0;}
   if (*y0<*y1) {b=*y0; d=*y1;} else {b=*y1; d=*y0;}
   *x0=a; *y0=b; *x1=c; *y1=d;
}

/* EXPORT-> HDrawLines: Draw multiple lines */
void HDrawLines(HPoint *points, int n)
{
   XDrawLines(theDisp, theWindow, theGC, (XPoint *) points, n, CoordModeOrigin);
}

/* EXPORT-> HDrawRectangle: draw a rectangle */
void HDrawRectangle(int x0, int y0, int x1, int y1)
{
   CheckCorners(&x0,&y0,&x1,&y1);
   XDrawRectangle(theDisp, theWindow, theGC, x0, y0, x1 - x0, y1 - y0);
}

/* EXPORT-> HFillRectangle: fill a rectangle */
void HFillRectangle(int x0, int y0, int x1, int y1)
{
   CheckCorners(&x0,&y0,&x1,&y1);
   XFillRectangle(theDisp, theWindow, theGC, x0, y0, x1 - x0, y1 - y0);
}

/* EXPORT-> HDrawLines: Draw multiple lines */
void HDrawLine(int x0, int y0, int x1, int y1)
{
   XDrawLine(theDisp, theWindow, theGC, x0, y0, x1, y1);
}

/* EXPORT-> HFillPolygon: fill a convex polygon */
void HFillPolygon(HPoint *points, int n)
{
   XFillPolygon(theDisp, theWindow, theGC, (XPoint *) points, n, Convex, CoordModeOrigin);
}

/* EXPORT-> HDrawArc: Draw arc from stAngle thru arcAngle degrees */
void HDrawArc(int x0, int y0, int x1, int y1, int stAngle, int arcAngle)
{
   unsigned int rw, rh;

   CheckCorners(&x0,&y0,&x1,&y1);
   /* calculate width and height */
   rw = abs(x1 - x0); rh = abs(y1 - y0);
   /* the angles are signed integers in 64ths of a degree */
   stAngle *=64; arcAngle*=64;
   XDrawArc(theDisp, theWindow, theGC, x0, y0, rw, rh, stAngle, arcAngle);
}

/* EXPORT-> HFillArc: Draw filled arc from stAngle thru arcAngle degrees */
void HFillArc(int x0,int y0,int x1,int y1,int stAngle,int arcAngle)
{
   unsigned int rw, rh;
   
   CheckCorners(&x0,&y0,&x1,&y1);
   /* calculate width and height */
   rw = abs(x1 - x0); rh = abs(y1 - y0);
   /* the angles are signed integers in 64ths of a degree */
   stAngle *=64; arcAngle*=64;
   XFillArc(theDisp, theWindow, theGC, x0, y0, rw, rh, stAngle, arcAngle);
}

/* EXPORT-> HPrintf: works as printf on the graphics window at (x,y) */
void HPrintf(int x, int y, char *format, ...)
{
   va_list arg;
   char s[256];
   
   va_start(arg, format);
   vsprintf(s, format, arg);
   XDrawString(theDisp, theWindow, theGC, x, y, s, strlen(s));
}

/* EXPORT-> copy rectangular area of the drawable */
void HCopyArea(int srcx, int srcy, int width, int height, int destx, int desty)
{
   XCopyArea(theDisp, theWindow, theWindow, theGC, srcx, srcy, width, 
             height, destx, desty);
}

/* EXPORT-> HPlotVector: plot vector v in given rectangle */
void HPlotVector(int x0, int y0, int x1, int y1, Vector v, int st, int en, float ymax, float ymin)
{
   float yScale, yOffset, xInc, x;
   int   xOld, yOld, ix, iy, i;
   
   if (st >= en || st < 1 || en > VectorSize(v))
      HError(6815, "HPlotVector: Plot indices %d -> %d out of range", st, en);
   x = (x1 - x0 - 1); xInc = x/(en - st);
   yScale  = (y1 - y0)/(ymin - ymax);
   yOffset = y0 - ymax*yScale;
   x = x0; xOld = (int) x; yOld = (int) (v[st]*yScale + yOffset);
   for (i = st+1; i <= en; i++){
      x += xInc; ix = (int) x;
      iy = (int) (v[i]*yScale + yOffset);
      XDrawLine(theDisp, theWindow, theGC, xOld, yOld, ix, iy);
      xOld = ix; yOld = iy;
   } 
}

/* ----------------------------- Settings --------------------------------- */

/* EXPORT-> HSetFontSize: Set font size in points, 0 selects the default font */
void HSetFontSize(int size)
{  
   int i, bestf, d, min_d;
   XferMode  xf;

   if (size==0)
      CurrentFont = DefaultFont;
   else {
      min_d = INT_MAX; bestf = 0;
      for (i=0; i < FONTS_AVAILABLE; i++) {
         d=abs(FontSize[i] - size);
         if (d < min_d) {
            min_d = d; bestf = i;
         }
      }
      CurrentFont = FontInfo[bestf]; 
   }
   for (xf = GCOPY; xf <= GINVERT; xf=(XferMode) (xf+1))   /* change all GCs */
      XSetFont(theDisp, gcs[(int) xf], CurrentFont->fid); 
}

/* EXPORT-> HSetLineWidth: set the line width */
void HSetLineWidth(int w)
{
   XferMode xf;
   
   for (xf = GCOPY; xf <= GINVERT; xf=(XferMode) (xf+1))   /* change all GCs */
      XSetLineAttributes(theDisp,gcs[(int)xf],w,LineSolid,JoinRound,FillSolid);
}

/* EXPORT-> HSetXMode: Set current transfer mode */
void HSetXMode(XferMode m)
{ 
   theGC = gcs[(int) m];
}

/* EXPORT-> CentreX: return position at which the the h-center of str will be at x */
int CentreX(int x, char *str)
{
   return (x - XTextWidth(CurrentFont, str, strlen(str))/2);
}

/* EXPORT-> CentreY: return position at which the the v-center of str will be at y */
int CentreY(int y, char *str)
{
   return (y - ((CurrentFont->ascent + CurrentFont->descent)/2) + CurrentFont->ascent);
}

/* EXPORT HTextWidth: return the width of s in pixels */
int HTextWidth(char *str)
{
   return XTextWidth(CurrentFont, str, strlen(str));
}

/* EXPORT HTextHeight: return the height of s in pixels */
int HTextHeight(char *str)
{
   return CurrentFont->ascent + CurrentFont->descent;
}


/* --------------------------- Misc Routines -----------------------------*/

/* EXPORT->HDrawImage: draw grey scale image stored in p */
void HDrawImage(unsigned char *p, int x, int y, int width, int height)
{
   static XImage *xi = NULL;
   static unsigned char *mem = NULL;
   unsigned char *pix;
   int i, j;
   
   if (mem != p){
      if (xi != NULL) 
         XDestroyImage(xi);
      xi = XGetImage(theDisp,theWindow,x,y,width,height,AllPlanes,XYPixmap);
      pix = mem = p;
      for (j = 0; j < height; j++)
         for (i = 0; i < width; i++)
            XPutPixel(xi, i, j, greys[(int) (*pix++)]);
   }
   XPutImage(theDisp, theWindow, theGC, xi, 0, 0, x, y, width, height);
}

/* EXPORT->HFlush: flush any pending draw operations */
void HFlush(void)
{
   XFlush(theDisp);
}

/* EXPORT-> HSpoolGraf: start saving an image of window in fname */
void HSpoolGraf(char *fname)
{
}


/* EXPORT->HDumpGraf: dump a pixel image of current display into fname */
void HDumpGraf(char *fname)
{
}

/* ------------------ Timer routines  ------------------ */

#ifdef USE_TIMER
static Boolean timerDone;

static void AlarmOn(void)
{
   timerDone = TRUE;
}

/* Timer: wait for n milliseconds */
static void Timer(int n)
{
   long   usec;
   struct itimerval it;
   
   usec = (long) n * 1000;
   memset(&it, 0, sizeof(it));
   if (usec>=1000000L) {  /* more than 1 second */
      it.it_value.tv_sec = usec / 1000000L;
      usec %= 1000000L;
   }
   it.it_value.tv_usec = usec; timerDone=FALSE;
   signal(SIGALRM, AlarmOn);
   setitimer(ITIMER_REAL, &it, NULL);
   while (1) {
      sigblock(sigmask(SIGALRM)); 
      if (timerDone) break;       
      sigpause(0);                
   }
   sigblock(0);
   signal(SIGALRM, SIG_DFL);
}
#endif

/* ---------------------- Button facility ---------------------- */

#define BTN_WAIT    200        /* 200 milliseconds minimum button press */
#define BTN_LINE_WIDTH 1       /* the line width for button drawing */

/* EXPORT->CreateHButton: create a button object with the specified parameters */
HButton *CreateHButton(HButton *btnlst, ButtonId btnid, int x, int y, int w, 
                       int h, char *str, HColour fg, HColour bg, void (*action)(void))
{
   HButton *btn, *btnptr;
   
   btn = (HButton *) New(&btnHeap, sizeof(HButton));
   /* initialise the fields of the structure */
   btn->id = btnid;
   btn->x = x;   btn->y = y;   btn->w = w;   btn->h = h;
   btn->str = str;
   btn->fg = fg;   btn->bg = bg;
   btn->lit = FALSE; btn->active = TRUE; btn->toggle = FALSE;
   btn->next = NULL;
   /* append it to the end of the list if the list already exists */
   if (btnlst!=NULL){
      btnptr = btnlst;
      while (btnptr->next != NULL) btnptr = btnptr->next;
      btnptr->next = btn;
   }
   btn->action = action;
   /* return ptr to the newly created button */
   return btn;
}

/* EXPORT->RedrawHButton: readraw a single button object */
void RedrawHButton(HButton *btn)
{
   int pad = 2;
   int x, y, w, h, r, s, pos;
   HPoint poly[9], shad[4];
   char sbuf[256], nullchar = '\0';
   
   x = btn->x;   y=btn->y;   w=btn->w;   h=btn->h;   r=3; s=1;

   /* set up the polygon */
   poly[0].x = x;         poly[0].y = y+r;
   poly[1].x = x;         poly[1].y = y+h-r;
   poly[2].x = x+r;       poly[2].y = y+h;
   poly[3].x = x+w-r;     poly[3].y = y+h;
   poly[4].x = x+w;       poly[4].y = y+h-r;
   poly[5].x = x+w;       poly[5].y = y+r;
   poly[6].x = x+w-r;     poly[6].y = y;
   poly[7].x = x+r;       poly[7].y = y;
   poly[8].x = x;         poly[8].y = y+r;
   /* set up the extra lines for the shadow */
   shad[0].x = x+r+s;     shad[0].y = y+h+s;
   shad[1].x = x+w-r+s;   shad[1].y = y+h+s;
   shad[2].x = x+w+s;     shad[2].y = y+h-r+s;
   shad[3].x = x+w+s;     shad[3].y = y+r+s;

   if (btn->lit) 
      HSetColour(btn->fg);
   else 
      HSetColour(btn->bg);
   HFillPolygon(poly, 9);
   HSetColour(btn->fg);
   HDrawLines(poly, 9);
   HDrawLines(shad, 4);
   if (btn->active)
      if (btn->lit)
         HSetColour(btn->bg);
      else 
         HSetColour(btn->fg);
   else
      HSetGrey(30);
   strcpy(sbuf, btn->str);
   pos = strlen(sbuf); 
   while(HTextWidth(sbuf) > (w - 2*pad)) 
      sbuf[--pos]=nullchar;
   HPrintf(CentreX(x+w/2, sbuf), CentreY(y+h/2, sbuf), "%s", sbuf);
}


/* EXPORT->RedrawHButtonList: redraw the whole list of buttons */
void RedrawHButtonList(HButton *btnlst)
{
   HButton *btnptr;

   HSetLineWidth(BTN_LINE_WIDTH);
   for (btnptr=btnlst; btnptr!=NULL; btnptr=btnptr->next)
      RedrawHButton(btnptr);
}

/* EXPORT->FindButton: find button given name */
HButton *FindButton(HButton *btnlst, ButtonId key)
{
   HButton *btnptr;

   for (btnptr=btnlst; btnptr!=NULL; btnptr=btnptr->next)
      if (btnptr->id==key)
         return btnptr;
   return NULL;
}

/* EXPORT->SetActive: set active field in button list */
void SetActive(HButton *btnlst, Boolean active)
{
   HButton *btnptr;

   for (btnptr=btnlst; btnptr!=NULL; btnptr=btnptr->next)
      btnptr->active = active;
}

/* EXPORT->CheckButtonList: find within which button the point(x,y) is */
HButton *CheckButtonList(HButton *btnlst, int x, int y)
{
   HButton *btn;
   
   for (btn=btnlst; btn!=NULL; btn=btn->next)
      if (IsInRect(x, y, btn->x, btn->y, btn->x + btn->w, btn->y + btn->h) && btn->active)
         return btn;
   return NULL;
}

/* EXPORT->SetButtonLit: show button press */
void SetButtonLit(HButton *btn, Boolean lit)
{
   if (btn->lit != lit){
      btn->lit = lit;
      RedrawHButton(btn);
   }
}

/* EXPORT->TrackButtons: tracks the buttons until the mouse button is released */
ButtonId TrackButtons(HButton *btnlist, HEventRec hev)
{
   HButton *pressed, *released;
   Boolean done;

   pressed = CheckButtonList(btnlist, hev.x, hev.y);
   if (pressed != NULL){
      SetButtonLit(pressed, TRUE);
      done = FALSE;
#ifdef USE_TIMER
      HFlush(); Timer(BTN_WAIT);
#endif
      do {
         hev = HGetEvent(TRUE, pressed->action);
         done = (hev.event==HMOUSEUP);
      } while (!done);
      released = CheckButtonList(btnlist, hev.x, hev.y);
      SetButtonLit(pressed, FALSE);
      if ( pressed == released)
         return pressed->id;
   }
   return 0;
}

/* ------------------ Initialisation and Termination -----------------------*/

/* InitGCs: each transfer mode is implemented as a different graphics context in X11   */
/*          the GINVERT mode inverts only the b/w plane and is implemented using GXxor */
static void InitGCs(void)
{
   XferMode   xf;
   XGCValues  values;
   unsigned   long mask;
   
   mask = GCLineWidth | GCFunction | GCForeground;
   for (xf = GCOPY; xf < GINVERT; xf=(XferMode) (xf+1)){
      values.line_width = 0;
      values.function = TFuncX11[(int) xf];
      values.foreground = black;
      gcs[(int) xf] = XCreateGC(theDisp, theWindow, mask, &values);
   }
   mask = GCLineWidth | GCFunction | GCPlaneMask | GCForeground;
   values.line_width = 0;
   values.function = GXxor;
   values.foreground = ~0;
   values.plane_mask = black ^ white;
   gcs[(int) GINVERT] = XCreateGC(theDisp, theWindow, mask, &values);
}

static void InitGlobals(void)
{
   theScreen = DefaultScreen(theDisp);
   theCmap   = DefaultColormap(theDisp, theScreen);
   rootW     = RootWindow(theDisp, theScreen);
   theGC     = DefaultGC(theDisp, theScreen);
   theVisual = DefaultVisual(theDisp, theScreen);
   ncells    = DisplayCells(theDisp, theScreen);
   dispWIDE  = DisplayWidth(theDisp, theScreen);
   dispHIGH  = DisplayHeight(theDisp, theScreen);
   dispDEEP  = DisplayPlanes(theDisp, theScreen);
   white     = WhitePixel(theDisp,theScreen);
   black     = BlackPixel(theDisp,theScreen);
}

/* EXPORT-> MakeXGraf: Connect to the X-server and init globals */
void MakeXGraf(char *wname, int x, int y, int w, int h, int bw)
{
   char sbuf[256], *hgraf = "HGraf";
   Window window, parent;
   XSetWindowAttributes setwinattr;
   unsigned long vmask;
   
   if (winCreated)
      HError(6870, "MakeXGraf: Attempt to recreate the graphics window");
   if ((theDisp = XOpenDisplay(NULL)) == NULL)
      HError(6870, "MakeXGraf: cannot connect to X server %s", XDisplayName(NULL));
   InitGlobals();
   InstallFonts();
   InstallColours();
   parent = RootWindow(theDisp, theScreen);
   window = XCreateSimpleWindow(theDisp, parent, x, y, w, h, bw, black, white );
   /* allow for backing up the contents of the window */
   vmask = CWBackingStore;  setwinattr.backing_store = WhenMapped;
   XChangeWindowAttributes(theDisp, window, vmask, &setwinattr);
   /* set the size hints for the window manager */
   hints.flags = PPosition | PSize | PMaxSize | PMinSize;
   hints.y = y;              hints.x = x;
   hints.width  = w;         hints.height = h;
   hints.min_width  = w;     hints.min_height = h;
   hints.max_width  = w;     hints.max_height = h;
   /* compose the name of the window */
   strcpy(sbuf, hgraf);  strcat(sbuf, ": ");  strcat(sbuf, wname);
   XSetStandardProperties(theDisp, window, sbuf, hgraf, None, NULL, 0, &hints);
   /* select events to receive */
   XSelectInput(theDisp, window, ExposureMask | KeyPressMask | ButtonPressMask | 
                ButtonReleaseMask | PointerMotionHintMask | PointerMotionMask);
   XMapWindow(theDisp, theWindow = window);
   InitGCs(); 
   HSetXMode(GCOPY); HSetFontSize(0); HSetLineWidth(0); HSetColour(BLACK);
   /* wait for the first expose event - cannot draw before it has arrived */
   do 
      XNextEvent(theDisp, &report); 
   while (report.type != Expose);
   XSendEvent(theDisp,window,False,ExposureMask,&report);
   /* Create heap for buttons */
   CreateHeap(&btnHeap, "Button heap", MHEAP, sizeof(HButton), 1.0, 100, 100);
   winCreated = TRUE;
}

/* EXPORT->TermHGraf: Terminate Graphics (also called via at_exit) */
void TermHGraf()
{
   if (theDisp != NULL)
      XCloseDisplay(theDisp);
   DeleteHeap(&btnHeap);
}


/* ------------------------ End of HGraf.X.c ------------------------- */

