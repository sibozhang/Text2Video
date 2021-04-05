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
/*         File: HGraf.win32.c:  HGraf for Windows NT          */
/* ----------------------------------------------------------- */
/* Win32 port Peter Silsbee                                    */

char *hgraf_version = "!HVER!HGraf(NT):   3.4.1 [CUED 12/03/09]";
char *hgraf_vc_id = "$Id: HGraf_WIN32.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

/* define CAPTURE_ALT to allow application to respond to Alt-key presses. */
/* "Normal" windows application behavior is to allow the system to handle it. */
#define CAPTURE_ALT

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HGraf.h"

#include <windows.h>
#include <ctype.h>
#include <stdio.h>
#include <memory.h>

#define DEF_FONTSIZE 12           /* default font size */
#define MAX_POINT    64           /* max number of points for polygons */

static MemHeap btnHeap;           /* heap for HButton structures */

/* -------------------- Global Windows variables: ------------------------*/

static HWND theWindow = NULL;     /* A handle to the graphics window */
static HDC memDC = NULL;          /* A handle to the memory device context */
static HDC DCSaved;               /* to store memDC when saving Metafile */
static HBITMAP  theBitmap;        /* Internal representation of window contents */

/* -------------------- Objects that draw ----------------------- */

static HBRUSH theBrush = NULL;      /* Used to fill solid areas */
static HPEN thePen     = NULL;      /* Used to draw lines */
static HPEN thinPen    = NULL;      /* Always has width 1, needed for outlining filled shapes */
static HFONT theFont   = NULL;      /* Current font for text output */

/* ------------ stuff that application keeps track of ------------*/

static Boolean winCreated    = FALSE;               /* prevent duplicate windows */
static Boolean WritingToMeta = FALSE;               /* TRUE if spooling to a metafile */
static unsigned char colours[MAX_COLOURS][3];    /* r,g,b */
static unsigned char greys[MAX_GREYS];
static int   dispDEEP,dispWIDE,dispHIGH;
static RECT  ClientRect;
static POINT winPoints[MAX_POINT];

static int LineWidth = 1;
static COLORREF CurrentColour = RGB(0,0,0);
static POINT MousePos;   /* updated when a WM_MOUSEMOVE occurs */
                         /* Win32 does not support direct querying of the mouse position */
static char *FONTNAME = "Helvetica";

#ifdef CAPTURE_ALT
enum _AltState {ALT_UP,ALT_DOWN}; /* keep track of Alt key */
typedef enum _AltState AltState;
static AltState AltKeyState = ALT_UP;
#endif

#ifdef WIN32
LRESULT CALLBACK HGWinFunc(HWND WindowHandle, unsigned int msg, 
                           WPARAM wParam, LPARAM lParam);
/*   Handle messages from Windows */

KeyType HGetKeyType(char c);
/*      Utility routine to help decode key presses */
#endif


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

/* EXPORT->HGetEvent: return next relevant event in event queue */
HEventRec HGetEvent(Boolean anyEvent, void (*action)(void))
{
   HEventRec r={0,0,0,0,0};
   static KeyType SavedKeyType;
   MSG msg;
   Boolean hasEvent = FALSE;
   HWND WindowHandle = (anyEvent) ? NULL : theWindow;

   do {
      if (action==NULL) {
         GetMessage(&msg,WindowHandle,0,0);
         hasEvent=TRUE;
      } else {
         hasEvent=PeekMessage(&msg,WindowHandle,0,0,PM_REMOVE);
      }
      if (hasEvent) {
         TranslateMessage(&msg);
         switch (msg.message) {
         case WM_LBUTTONDOWN:
            r.event = HMOUSEDOWN;
            r.x = MousePos.x = LOWORD(msg.lParam);
            r.y = MousePos.y = HIWORD(msg.lParam);
            SetCapture(theWindow);
            break;
         case WM_LBUTTONUP:
            r.event = HMOUSEUP;
            r.x = MousePos.x = LOWORD(msg.lParam);
            r.y = MousePos.y = HIWORD(msg.lParam);
            ReleaseCapture();
            break;
         case WM_MOUSEMOVE:
            r.event = HMOUSEMOVE;
            r.x = MousePos.x = LOWORD(msg.lParam);
            r.y = MousePos.y = HIWORD(msg.lParam);
            break;
#ifdef CAPTURE_ALT     /* alt key events are normally intended for the system */
         case WM_SYSKEYDOWN:
            r.c = (unsigned char) msg.wParam;
            if (r.c == VK_MENU) {
               r.ktype = COMMANDKEY;
               r.event = HKEYPRESS;
               r.x = MousePos.x;
               r.y = MousePos.y;
               AltKeyState = ALT_DOWN;
               break;
            } /* else fall through to regular keydown */
#endif
         case WM_KEYDOWN:
            r.event = HKEYPRESS;
            r.x = MousePos.x;
            r.y = MousePos.y;
            r.c = (unsigned char) msg.wParam;
            SavedKeyType = r.ktype = HGetKeyType(r.c);
            if ((r.c != VK_DELETE) &&
                (r.ktype != CONTROLKEY) &&
                (r.ktype != COMMANDKEY) &&
                (r.ktype != SHIFTKEY))
               hasEvent = FALSE; /* other keys will be processed by WM_CHAR */
            /* message which should arrive shortly */
            break;
#ifdef CAPTURE_ALT
         case WM_SYSCHAR:
#endif
         case WM_CHAR:
            r.event = HKEYPRESS;
            r.x = MousePos.x;
            r.y = MousePos.y;
            r.c = (unsigned char) msg.wParam;
            r.ktype = SavedKeyType; //HGetKeyType(r.c);
            break;
#ifdef CAPTURE_ALT
         case WM_SYSKEYUP:
            r.c = (unsigned char) msg.wParam;
            if (r.c == VK_MENU) {
               r.ktype = COMMANDKEY;
               r.event = HKEYRELEASE;
               r.x = MousePos.x;
               r.y = MousePos.y;
               AltKeyState = ALT_UP;
               break;
            } /* else fall through to regular keydown */
#endif
         case WM_KEYUP:
            r.event = HKEYRELEASE;
            r.x = MousePos.x;
            r.y = MousePos.y;
            r.c = (unsigned char) msg.wParam;
            r.ktype = HGetKeyType(r.c);
            break;
         case WM_PAINT:
            DispatchMessage(&msg);    /* force Win32 to remove message from   */
            /* queue even though application is dispatching this message.     */
            /* Applications should actually be able to ignore HREDRAW events. */
            /* Fall through to next messages */
         case WM_SIZING:
         case WM_MOVING:
         case WM_EXITSIZEMOVE:
            r.event = HREDRAW;
            break;
         default: 
            hasEvent = FALSE;
            DispatchMessage(&msg); /* Win32 should handle other messages */
         }
      }
      else if (action != NULL) {
         (*action)();
      }
   } while (!hasEvent);
   return r;
}

/* EXPORT->HEventsPending: Return number of events pending           */
/* This doesn't seem to be supported in Win32. It is possible        */
/* to see if the queue is empty, but there is no way to see if       */
/* there is just one event or if there are many. This function       */
/* should probably return a Boolean value. Currently it returns 1    */
/* if there are one or more events pending, and 0 if there are none. */
int HEventsPending(void)
{
   MSG msg;

   if (PeekMessage(&msg,theWindow,0,0,PM_NOREMOVE))
      return 1;
   return 0;
}

/* EXPORT->HMousePos: return mouse pos in x, y, returns TRUE if the pointer 
   is on the window */
/* Win32: We only get mouse position information when (a) there is an event  */
/* when the mouse is positioned over the window, or (b) when we are capturing */
/* the mouse. We only capture the mouse when the mouse button is depressed.   */
/* Thus, the mouse position determined from this function may not be up to    */
/* date. This shouldn't be a problem since the "focus" is not determined by   */
/* mouse position in Win32. That is, keyboard events that take place when the */
/* mouse is outside the window still really do belong to our window, if the   */
/* event has been directed to our application.                                */

Boolean HMousePos(int *x, int *y)
{
   *x = MousePos.x;
   *y = MousePos.y;
   return (Boolean) IsInRect(*x,*y,ClientRect.left,ClientRect.top,
                             ClientRect.right,ClientRect.bottom);
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
   int c;
   int step;
   LOGPALETTE *pal;
   HPALETTE hpal,OldPal;
   HDC dc;
   /* WHITE */
   colours[0][0] = 255; colours[0][1] = 255; colours[0][2] = 255;
   /* YELLOW */
   colours[1][0] = 255; colours[1][1] = 255; colours[1][2] = 0;
   /* ORANGE */
   colours[2][0] = 255; colours[2][1] = 128; colours[2][2] = 0;
   /* RED */
   colours[3][0] = 255; colours[3][1] = 0;   colours[3][2] = 0;
   /* MAUVE */
   colours[4][0] = 196; colours[4][1] = 100; colours[4][2] = 255;
   /* PURPLE */
   colours[5][0] = 128; colours[5][1] = 0;   colours[5][2] = 128;
   /* DARK_BLUE */
   colours[6][0] = 0;   colours[6][1] = 0;   colours[6][2] = 196;
   /* LIGHT_BLUE (CYAN) */
   colours[7][0] = 0;   colours[7][1] = 255; colours[7][2] = 255;
   /* DARK_GREEN */
   colours[8][0] = 0;   colours[8][1] = 128; colours[8][2] = 0;
   /* LIGHT_GREEN */
   colours[9][0] = 0;   colours[9][1] = 255; colours[9][2] = 0;
   /* DARK_BROWN */
   colours[10][0] = 128;colours[10][1] = 64; colours[10][2] = 64;
   /* LIGHT_BROWN */
   colours[11][0] = 196;colours[11][1] = 140;colours[11][2] = 140;
   /* LIGHT_GREY */
   colours[12][0] = 196;colours[12][1] = 196;colours[12][2] = 196;
   /* GREY */
   colours[13][0] = 128;colours[13][1] = 128;colours[13][2] = 128;
   /* DARK_GREY */
   colours[14][0] = 64; colours[14][1] = 64; colours[14][2] = 64;
   /* BLACK */
   colours[15][0] = 0;  colours[15][1] = 0;  colours[15][2] = 0;
     
   step = 256/MAX_GREYS;
   for (c = 0; c < MAX_GREYS; c++, pixVal+= step){
      greys[c] = pixVal;
   } 
     
   /* if display is 8 bits or less, create a palette containing 
      the best match to our desired colors. If it is 16 bits or
      greater, matching will be no problem. */
   if (dispDEEP <= 8){
      pal = (LOGPALETTE *) 
         New(&gcheap,2*sizeof(WORD) + (MAX_GREYS + MAX_COLOURS)*sizeof(PALETTEENTRY));
      pal->palVersion = 0x300;
      pal->palNumEntries = MAX_GREYS + MAX_COLOURS;
          
      /* most important colors should be first in list. Black, White, 
         the rest of the colors, then the greys. */

      pal->palPalEntry[0].peRed = pal->palPalEntry[0].peGreen = 
         pal->palPalEntry[0].peBlue = 0;
      pal->palPalEntry[0].peFlags = 0;
      pal->palPalEntry[1].peRed = pal->palPalEntry[1].peGreen = 
         pal->palPalEntry[1].peBlue = 255;
      pal->palPalEntry[1].peFlags = 0;
      for (c=2;c<MAX_COLOURS;c++) {
         pal->palPalEntry[c].peRed = colours[c-1][0];
         pal->palPalEntry[c].peGreen = colours[c-1][1];
         pal->palPalEntry[c].peBlue = colours[c-1][2];
         pal->palPalEntry[c].peFlags = 0;
      } 
          
      for (c=0;c<MAX_GREYS;c++) {
         pal->palPalEntry[MAX_COLOURS+c].peRed = greys[c];
         pal->palPalEntry[MAX_COLOURS+c].peGreen = greys[c];
         pal->palPalEntry[MAX_COLOURS+c].peBlue = greys[c];
         pal->palPalEntry[MAX_COLOURS+c].peFlags = 0;
      }
      hpal = CreatePalette(pal);
          
      OldPal = SelectPalette(memDC,hpal,FALSE);
      RealizePalette(memDC);
      SelectPalette(memDC,OldPal,FALSE);
          
      dc = GetDC(theWindow);
      OldPal = SelectPalette(dc,hpal,FALSE);
      RealizePalette(dc);
      SelectPalette(dc,OldPal,FALSE);
      ReleaseDC(theWindow,dc);
      DeleteObject(hpal);
   } /* if (dispDEEP <= 8) */
}

/* EXPORT-> HSetColour: Set current colour to c */
void HSetColour(HColour c)
{
   CurrentColour = RGB(colours[c][0],colours[c][1],colours[c][2]);
     
   if (theBrush) DeleteObject(theBrush);
   theBrush = CreateSolidBrush(CurrentColour);
     
   if (thePen) DeleteObject(thePen);
   thePen = CreatePen(PS_SOLID,LineWidth,CurrentColour);
     
   if (thinPen) DeleteObject(thinPen);
   thinPen = CreatePen(PS_SOLID,1,CurrentColour);
}

/* EXPORT-> HSetGrey: Set current colour to grey level g */
void HSetGrey(int g)
{
   CurrentColour = RGB(greys[g],greys[g],greys[g]);
     
   if (theBrush) DeleteObject(theBrush);
   theBrush = CreateSolidBrush(CurrentColour);
     
   if (thePen) DeleteObject(thePen);
   thePen = CreatePen(PS_SOLID,LineWidth,CurrentColour);
     
   if (thinPen) DeleteObject(thinPen);
   thinPen = CreatePen(PS_SOLID,1,CurrentColour);
}

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
   int i;
   HDC dc;
   HGDIOBJ oldObject;

   if (n>MAX_POINT)
      HError(6815, "HDrawLines: can only specify up to %d points",MAX_POINT);
   for(i=0; i<n; i++) {
      winPoints[i].x=points[i].x;
      winPoints[i].y=points[i].y;
   }
   dc = GetDC(theWindow);
   oldObject = SelectObject(memDC,thePen);
   Polyline(memDC, winPoints, n);
   SelectObject(memDC,oldObject);
     
   oldObject = SelectObject(dc,thePen);
   Polyline(dc, winPoints, n);
   SelectObject(dc,oldObject);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> HDrawRectangle: draw a rectangle */
void HDrawRectangle(int x0, int y0, int x1, int y1)
{
   POINT points[5];
   HGDIOBJ oldObject = SelectObject(memDC,thePen);
   HDC dc = GetDC(theWindow);
     
   CheckCorners(&x0,&y0,&x1,&y1);
   points[0].x = x0; points[0].y = y0;
   points[1].x = x0; points[1].y = y1;
   points[2].x = x1; points[2].y = y1;
   points[3].x = x1; points[3].y = y0;
   points[4].x = x0; points[4].y = y0;
     
   Polyline(memDC, points, 5);
   SelectObject(memDC,oldObject);
     
   oldObject = SelectObject(dc,thePen);
   Polyline(dc, points, 5);
   SelectObject(dc,oldObject);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> HFillRectangle: fill a rectangle */
void HFillRectangle(int x0, int y0, int x1, int y1)
{
   HDC dc = GetDC(theWindow);
   HGDIOBJ oldBrush = SelectObject(memDC,theBrush);
   HGDIOBJ oldPen = SelectObject(memDC,thinPen);
     
   CheckCorners(&x0,&y0,&x1,&y1);
     
   Rectangle(memDC,x0,y0,x1,y1);
     
   SelectObject(memDC,oldBrush);
   SelectObject(memDC,oldPen);
     
   oldBrush = SelectObject(dc,theBrush);
   oldPen = SelectObject(dc,thinPen);
   Rectangle(dc,x0,y0,x1,y1);
   SelectObject(dc,oldBrush);
   SelectObject(dc,oldPen);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> HDrawLine: Draw one line */
void HDrawLine(int x0, int y0, int x1, int y1)
{
   HDC dc = GetDC(theWindow);
   HGDIOBJ oldObject = SelectObject(memDC,thePen);
     
   MoveToEx(memDC,x0,y0,NULL);
   LineTo(memDC,x1,y1);

   SelectObject(memDC,oldObject);
     
   oldObject = SelectObject(dc,thePen);
   MoveToEx(dc,x0,y0,NULL);
   LineTo(dc,x1,y1);
   SelectObject(dc,oldObject);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> HFillPolygon: fill a convex polygon */
void HFillPolygon(HPoint *points, int n)
{
   int i;
   HDC dc;
   HGDIOBJ oldPen;
   HGDIOBJ oldBrush;

   if (n>MAX_POINT)
      HError(6815, "HFillPolygon: can only specify up to %d points",MAX_POINT);
   for(i=0; i<n; i++) {
      winPoints[i].x=points[i].x;
      winPoints[i].y=points[i].y;
   }
   dc = GetDC(theWindow);
   oldPen = SelectObject(memDC,thinPen);
   oldBrush = SelectObject(memDC,theBrush);
     
   Polygon(memDC,winPoints,n);
     
   SelectObject(memDC,oldBrush);
   SelectObject(memDC,oldPen);
     
   oldBrush = SelectObject(dc,theBrush);
   oldPen = SelectObject(dc,thinPen);
   Polygon(dc,winPoints,n);
   SelectObject(dc,oldBrush);
   SelectObject(dc,oldPen);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> HDrawArc: Draw arc from stAngle thru arcAngle degrees */
void HDrawArc(int x0, int y0, int x1, int y1, int stAngle, int arcAngle)
{
   int Center_x = (x0+x1)/2;
   int Center_y = (y0+y1)/2;
   int StartArc_x, StartArc_y;
   int EndArc_x, EndArc_y;
   int radius; /* major axis */
   double startAngle, endAngle,convrt = PI/180; /* degrees to radians */
   HGDIOBJ oldObject = SelectObject(memDC,thePen);
   HDC dc = GetDC(theWindow);
     
   CheckCorners(&x0,&y0,&x1,&y1);
     
   startAngle = stAngle *convrt; 
   endAngle=(arcAngle+stAngle)*convrt;
     
   radius = (((x1-x0) > (y1-y0)) ? x1-x0 : y1-y0)/2;
   StartArc_x = Center_x + (int) (radius * cos((double) startAngle));
   StartArc_y = Center_y - (int) (radius * sin((double) startAngle));
   EndArc_x = Center_x + (int) (radius * cos((double) endAngle));
   EndArc_y = Center_y - (int) (radius * sin((double) endAngle));
     
   Arc(memDC,x0,y0,x1,y1,StartArc_x,StartArc_y,EndArc_x,EndArc_y);
   SelectObject(memDC,oldObject);
     
   oldObject = SelectObject(dc,thePen);
   Arc(dc,x0,y0,x1,y1,StartArc_x,StartArc_y,EndArc_x,EndArc_y);
   SelectObject(dc,oldObject);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> HFillArc: Draw filled arc from stAngle thru arcAngle degrees */
void HFillArc(int x0,int y0,int x1,int y1,int stAngle,int arcAngle)
{
   int radius;
   int Center_x = (x0+x1)/2;
   int Center_y = (y0+y1)/2;
   int StartArc_x,StartArc_y;
   int EndArc_x,EndArc_y;
   HGDIOBJ oldBrush = SelectObject(memDC,theBrush);
   HGDIOBJ oldPen = SelectObject(memDC,thinPen);
   HDC dc = GetDC(theWindow);
   double startAngle, endAngle,convrt = PI/180; /* degrees to radians */
     
   CheckCorners(&x0,&y0,&x1,&y1);
     
   /* calculate point locations */
     
   startAngle = stAngle*convrt; 
   endAngle = (stAngle+arcAngle)*convrt;
     
   radius = (((x1-x0) > (y1-y0)) ? x1-x0 : y1-y0)/2;
   StartArc_x = Center_x + (int) (radius * cos((double) startAngle));
   StartArc_y = Center_y - (int) (radius * sin((double) startAngle));
   EndArc_x = Center_x + (int) (radius * cos((double) endAngle));
   EndArc_y = Center_y - (int) (radius * sin((double) endAngle));
     
   Pie(memDC,x0,y0,x1,y1,StartArc_x,StartArc_y,EndArc_x,EndArc_y);
     
   SelectObject(memDC,oldBrush);
   SelectObject(memDC,oldPen);
     
   oldBrush = SelectObject(dc,theBrush);
   oldPen = SelectObject(dc,thinPen);
   Pie(dc,x0,y0,x1,y1,StartArc_x,StartArc_y,EndArc_x,EndArc_y);
   SelectObject(dc,oldBrush);
   SelectObject(dc,oldPen);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> HPrintf: works as printf on the graphics window at (x,y) */
void HPrintf(int x, int y, char *format, ...)
{ 
   va_list arg;
   char s[256];
   HGDIOBJ oldObject = SelectObject(memDC,theFont);
   HDC dc = GetDC(theWindow);
     
   SetTextColor(memDC,CurrentColour);   
   va_start(arg, format);
   vsprintf(s, format, arg);
     
   TextOut(memDC,x,y,s,strlen(s)); 
   SelectObject(memDC,oldObject);
     
   oldObject = SelectObject(dc,theFont);
   TextOut(dc,x,y,s,strlen(s)); 
   SelectObject(dc,oldObject);
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> copy rectangular area of the drawable */
void HCopyArea(int srcx, int srcy, int width, int height, int destx, int desty)
{
   HDC dc = GetDC(theWindow);
   BitBlt(memDC,destx,desty,width,height,memDC,srcx,srcy,SRCCOPY);
   BitBlt(dc,destx,desty,width,height,memDC,srcx,srcy,SRCCOPY);
   ReleaseDC(theWindow,dc);
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
   x = x0; xOld = x; yOld = v[st]*yScale + yOffset;
   for (i = st+1; i <= en; i++){
      x += xInc; ix = x;
      iy = v[i]*yScale + yOffset;
      HDrawLine(xOld,yOld,ix,iy);
      xOld = ix; yOld = iy;
   } 
}

/* ----------------------------- Global Settings ------------------------------- */

/* EXPORT-> HSetFontSize: Set font size in points, 0 selects the default font */
void HSetFontSize(int size)
{  
   int FontSize = (size > 0) ? size : DEF_FONTSIZE;
     
   if (theFont) DeleteObject(theFont);
   theFont = CreateFont(FontSize,
                        0,0,0,FW_NORMAL,
                        0,0,0,ANSI_CHARSET,
                        OUT_DEFAULT_PRECIS,
                        CLIP_DEFAULT_PRECIS,
                        DEFAULT_QUALITY,
                        DEFAULT_PITCH | FF_SWISS,
                        FONTNAME); 
}

/* EXPORT-> HSetLineWidth: set the line width */
void HSetLineWidth(int w)
{
   LineWidth = w;
     
   if (thePen) DeleteObject(thePen);
   thePen = CreatePen(PS_SOLID,LineWidth,CurrentColour);
}

/* EXPORT-> HSetXMode: Set current transfer mode */
void HSetXMode(XferMode m)
{ 
   HDC dc = GetDC(theWindow);
   switch(m) {
   case GCOPY:
      SetROP2(memDC,R2_COPYPEN);
      SetROP2(dc,R2_COPYPEN);
      break;
   case GOR:
      SetROP2(memDC,R2_MERGEPEN);
      SetROP2(dc,R2_MERGEPEN);
      break;
   case GXOR:
      SetROP2(memDC,R2_XORPEN);
      SetROP2(dc,R2_XORPEN);
      break;
   case GINVERT: 
      SetROP2(memDC,R2_NOT);
      SetROP2(dc,R2_NOT);
      break;
   default: /* GCOPY */
      SetROP2(memDC,R2_COPYPEN);
      SetROP2(dc,R2_COPYPEN);
      break;
   }
   ReleaseDC(theWindow,dc);
}

/* EXPORT-> CentreX: return position at which the the h-center of str will be at x */
int CentreX(int x, char *str)
{
   SIZE size;
     
   GetTextExtentPoint32(memDC,str,strlen(str),&size);
   return  (x-size.cx/2);
}

/* EXPORT-> CentreY: return position at which the the v-center of str will be at y */
int CentreY(int y, char *str)
{
   HDC dc;
   HGDIOBJ obj;
   TEXTMETRIC tm;
   int pos;

   dc = GetDC(theWindow);
   obj = SelectObject(dc,theFont);
   GetTextMetrics(dc,&tm);
   pos = (y - ((tm.tmAscent + tm.tmDescent)/2) + tm.tmAscent);
   SelectObject(dc,obj);
   return pos;
}

/* EXPORT HTextWidth: return the width of s in pixels */
int HTextWidth(char *str)
{
   SIZE size;
     
   GetTextExtentPoint32(memDC,str,strlen(str),&size);
   return  (size.cx);
}

/* EXPORT HTextHeight: return the height of s in pixels */
int HTextHeight(char *str)
{
   SIZE size;
     
   GetTextExtentPoint32(memDC,str,strlen(str),&size);
   return  (size.cy);
}

/* --------------------------- Misc/Button Routines -----------------------------*/

/* EXPORT->HDrawImage: draw grey scale image stored in p */
void HDrawImage(unsigned char *p, int x, int y, int width, int height)
{
   HDC tdc = GetDC(theWindow);
   HDC dc = CreateCompatibleDC(memDC);
   HBITMAP bm = CreateCompatibleBitmap(tdc,width,height);
   HGDIOBJ OldObject;

   char *data = New(&gcheap,sizeof(BITMAPINFOHEADER) + 
                    sizeof(RGBQUAD)*MAX_GREYS);
   BITMAPINFOHEADER *BitmapHeader = (BITMAPINFOHEADER *) data;
   RGBQUAD *ColorTable = (RGBQUAD *) (data + sizeof(BITMAPINFOHEADER));
   BITMAPINFO *Info = (BITMAPINFO *) data;

   int i,j;

   /* if the length of the scan line is not a */
   /* multiple of four, the bitmap must be reshaped. */
   /* SetDIBits() expects scan lines to start on word boundaries. */

   int ScanLineLen = 4*(1+(width-1)/4);  
   unsigned char *reshaped = NULL;       

   BitmapHeader->biSize = sizeof(BITMAPINFOHEADER);
   BitmapHeader->biWidth = width;
   BitmapHeader->biHeight = -height;
   BitmapHeader->biPlanes = 1;
   BitmapHeader->biBitCount = 8;
   BitmapHeader->biCompression = 0;
   BitmapHeader->biSizeImage = 0;
   BitmapHeader->biXPelsPerMeter = 0;
   BitmapHeader->biYPelsPerMeter = 0;
   BitmapHeader->biClrUsed = MAX_GREYS;
   BitmapHeader->biClrImportant = MAX_GREYS;
   for (i=0;i<MAX_GREYS;i++) {
      ColorTable[i].rgbRed =
         ColorTable[i].rgbBlue =
         ColorTable[i].rgbGreen = greys[i];
      ColorTable[i].rgbReserved = 0;
   }

   if (ScanLineLen != width) {
      reshaped = (unsigned char *) New(&gcheap,height*ScanLineLen);
      for (i=0;i<height;i++) {
         for (j=0;j<width;j++) {
            reshaped[i*ScanLineLen+j] = p[i*width+j];
         }
      }
      SetDIBits(memDC,bm,0,height,reshaped,Info,DIB_RGB_COLORS);
      Dispose(&gcheap,reshaped);
   }
   else {
      SetDIBits(memDC,bm,0,height,p,Info,DIB_RGB_COLORS);
   }

   OldObject = SelectObject(dc,bm);
   BitBlt(memDC,x,y,width,height,dc,0,0,SRCCOPY);
   if (WritingToMeta) { /* bitmap source location differs */
      BitBlt(tdc,x,y,width,height,dc,x,y,SRCCOPY);
   }
   else {
      BitBlt(tdc,x,y,width,height,dc,0,0,SRCCOPY);
   }

   DeleteDC(dc);
   DeleteObject(bm);
   ReleaseDC(theWindow,tdc);
   Dispose(&gcheap,data);
}

/* EXPORT->HFlush: flush any pending draw operations */
void HFlush(void)
{
}

/* EXPORT-> HSpoolGraf: start saving an image of window in fname */
/* must be balanced by a call to HEndSpoolGraf() */
void HSpoolGraf(char *fname)
{
   int wmm,hmm; /* width and height in millimeters */
   int wpx,hpx; /* width and height in pixels */
   char *description = "Created by HGraf";
   RECT r;
   HDC dc = GetDC(theWindow); 
   int er;
     
   wmm = GetDeviceCaps(dc, HORZSIZE); 
   hmm = GetDeviceCaps(dc, VERTSIZE); 
   wpx = GetDeviceCaps(dc, HORZRES); 
   hpx = GetDeviceCaps(dc, VERTRES); 
     
   r.left = (ClientRect.left * wmm * 100)/wpx; 
   r.top = (ClientRect.top * hmm * 100)/hpx; 
   r.right = (ClientRect.right * wmm * 100)/wpx; 
   r.bottom = (ClientRect.bottom * hmm * 100)/hpx; 
     
   DCSaved = memDC;
   memDC = CreateEnhMetaFile(dc,fname,&r,description); 
   er = GetLastError();
   ReleaseDC(theWindow,dc);
   WritingToMeta = TRUE;
}

/* EXPORT-> HEndSpoolGraf: close file opened in HSpoolGraf() */
/* It is recommended to redraw the window after this call.   */
void HEndSpoolGraf()
{
   WritingToMeta = FALSE;
   CloseEnhMetaFile(memDC);
   memDC = DCSaved;
}


/* EXPORT->HDumpGraf: dump a BMP image of current display into fname */
void HDumpGraf(char *fname)
{
   BITMAPFILEHEADER FileHeader;
   BITMAPINFOHEADER BitmapHeader;
   BITMAPINFO *Info;
   int ColorTableSize;
   int ImageSize;
   FILE *fp;
   char *img;
   HDC dc = GetDC(theWindow);
   HBITMAP temp = CreateCompatibleBitmap(memDC,1,1);
     
   SelectObject(memDC,temp);
     
   /* retrieve information about the bitmap */
   BitmapHeader.biSize = sizeof(BITMAPINFOHEADER);
   BitmapHeader.biBitCount = 0;
   GetDIBits(memDC,theBitmap,0,0,NULL,&BitmapHeader,BI_RGB);
     
   switch (BitmapHeader.biCompression) {
   case BI_RGB:
      if (BitmapHeader.biBitCount > 8) {
         ColorTableSize = 0;
      }
      else {
         ColorTableSize = BitmapHeader.biClrUsed*sizeof(RGBQUAD);
      }
      break;
   case BI_RLE8:
   case BI_RLE4:
      ColorTableSize = BitmapHeader.biClrUsed*sizeof(RGBQUAD); 
      break;
   case BI_BITFIELDS:
      ColorTableSize = 3*sizeof(DWORD);
   }
     
   Info = (BITMAPINFO *) New(&gcheap,sizeof(BITMAPINFOHEADER) + ColorTableSize);
   memcpy(Info,&BitmapHeader,sizeof(BITMAPINFOHEADER));
     
   ImageSize = BitmapHeader.biSizeImage;
   img = New(&gcheap,ImageSize);
     
   GetDIBits(memDC,theBitmap,0,ClientRect.bottom,img,Info,BI_RGB);
     
   FileHeader.bfType = 0x4d42;  /* 'BM' */
   FileHeader.bfSize = sizeof(BITMAPINFOHEADER) + sizeof(BITMAPFILEHEADER) + 
      ImageSize + ColorTableSize;
   FileHeader.bfReserved1 = FileHeader.bfReserved2 = 0;
   FileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + ColorTableSize; 
     
   fp = fopen(fname,"wb");
   fwrite(&FileHeader,1,sizeof(BITMAPFILEHEADER),fp);
   fwrite(Info,1,sizeof(BITMAPINFOHEADER) + ColorTableSize,fp);
   fwrite(img,1,ImageSize,fp);
   fclose(fp);

   SelectObject(memDC,theBitmap);
   DeleteObject(temp);
   Dispose(&gcheap,Info);
   Dispose(&gcheap,img);
}

#define BTN_WAIT    200        /* 200 milliseconds minimum button press */
#define BTN_LINE_WIDTH 1       /* the line width for button drawing */

/* EXPORT->CreateHButton: create a button object with the specified parameters */
HButton *CreateHButton(HButton *btnlst, ButtonId btnid, int x, int y, int w, 
                       int h, char *str, HColour fg, HColour bg, void (*action)(void))
{
   HButton *btn, *btnptr;
   
   btn = New(&btnHeap, sizeof(HButton));
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

static void InitGlobals(void)
{
   dispWIDE  = GetSystemMetrics(SM_CXSCREEN);
   dispHIGH  = GetSystemMetrics(SM_CYSCREEN);
   dispDEEP  = GetDeviceCaps(memDC,BITSPIXEL); 
}


/* EXPORT-> MakeXGraf: Create and open window, initialization */
void MakeXGraf(char *wname, int x, int y, int w, int h, int bw)
     /* WIN32: bw is ignored. */
{
   WNDCLASS WindowClass;
   char sbuf[256], *hgraf = "HGraf";
   HDC dc;
     
   if (winCreated)
      HError(6870, "MakeXGraf: Attempt to recreate the graphics window");
     
   WindowClass.hInstance = GetModuleHandle(NULL);
   WindowClass.lpszClassName = hgraf;
   WindowClass.lpfnWndProc = HGWinFunc;
   WindowClass.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
   WindowClass.hIcon = NULL;
   WindowClass.hCursor = LoadCursor(NULL,IDC_ARROW);
   WindowClass.lpszMenuName = NULL;
   WindowClass.cbClsExtra = 0;
   WindowClass.cbWndExtra = 0;
   WindowClass.hbrBackground = (HBRUSH) GetStockObject(WHITE_BRUSH);
     
   RegisterClass(&WindowClass);
     
   strcpy(sbuf, hgraf);  strcat(sbuf, ": ");  strcat(sbuf, wname);
     
   theWindow = 
      CreateWindow(hgraf, sbuf, WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                   x,y, w,h, HWND_DESKTOP, NULL,
                   WindowClass.hInstance,       NULL);
     
   /* adjust window size so that the client rectangle is the size requested */
   /* by the caller of MakeXGraf() --- Win32 interprets w and h as the dimensions */
   /* of the overall window. */
   GetClientRect(theWindow,&ClientRect);
   MoveWindow(theWindow,x,y,w+w-ClientRect.right,h+h-ClientRect.bottom,TRUE); 
   GetClientRect(theWindow,&ClientRect);
     
   /* Obtain and initialize device contexts */
   dc = GetDC(theWindow);
   memDC = CreateCompatibleDC(dc);
   SetArcDirection(memDC,AD_COUNTERCLOCKWISE);
   SetArcDirection(dc,AD_COUNTERCLOCKWISE);
   SetPolyFillMode(memDC,WINDING);
   SetPolyFillMode(memDC,WINDING);
   SetTextAlign(memDC,TA_BASELINE | TA_LEFT);
   SetTextAlign(dc,TA_BASELINE | TA_LEFT);
   SetBkMode(memDC,TRANSPARENT);
   SetBkMode(dc,TRANSPARENT);
   theBitmap = CreateCompatibleBitmap(dc,w,h);
   SelectObject(memDC,theBitmap);
   ReleaseDC(theWindow,dc);
   CreateHeap(&btnHeap, "Button heap", MHEAP, sizeof(HButton), 1.0, 100, 100);
     
   InitGlobals();
   InstallColours();
     
   winCreated = TRUE;
   HSetColour(WHITE);
   HFillRectangle(0,0,ClientRect.right,ClientRect.bottom);
}

/* EXPORT->TermHGraf: Terminate Graphics (also called via at_exit) */
void TermHGraf()
{
   if (theBrush) DeleteObject(theBrush);
   if (thePen) DeleteObject(thePen);
   if (thinPen) DeleteObject(thinPen);
   if (theFont) DeleteObject(theFont);
   if (theBitmap) DeleteObject(theBitmap);
  
   DeleteHeap(&btnHeap);
   DestroyWindow(theWindow);
}

/********** Win32 specific functions  */

/* Called by the window manager */
LRESULT CALLBACK HGWinFunc(HWND WindowHandle, unsigned int msg, WPARAM wParam, LPARAM lParam)
{
   HDC dc;
   PAINTSTRUCT ps;
     
   switch (msg) {
   case WM_SIZING: /* for some reason we have to repaint when the window moves */
   case WM_MOVING:
      InvalidateRect(theWindow,&ClientRect,FALSE);
      return TRUE;
   case WM_EXITSIZEMOVE:
      InvalidateRect(theWindow,&ClientRect,FALSE);
      return 0;
   case WM_PAINT:
      dc = BeginPaint(theWindow,&ps);
      BitBlt(dc,0,0,ClientRect.right,ClientRect.bottom,memDC,0,0,SRCCOPY);
      EndPaint(theWindow,&ps);
      return 0;
   default:
      return DefWindowProc(WindowHandle, msg, wParam, lParam);
   }
}

KeyType HGetKeyType(char c)
{
   switch ((int) c) {
   case VK_ESCAPE:
      return ESCKEY;
   case VK_DELETE:
   case VK_BACK:
      return DELKEY; 
   case VK_RETURN:
      return ENTERKEY; 
   case VK_CONTROL:
      return CONTROLKEY; 
   case VK_MENU:
      return COMMANDKEY; 
   case VK_SHIFT:
      return SHIFTKEY; 
   default:
      return NORMALKEY;
   }
}


/* ------------------------ End of HGraf.win32.c ------------------------- */
