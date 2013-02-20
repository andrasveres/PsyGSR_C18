//************************************************************************
// KS108.H
// NeX
// 03.12.2006
//************************************************************************

//************************************************************************
// DEFINES
//************************************************************************

// ANDRIS

#define     DI                  LATBbits.LATB2
#define     DI_TRIS             TRISBbits.TRISB2

#define 	CSA					LATCbits.LATC6
#define 	CSA_TRIS			TRISCbits.TRISC6

#define 	CSB					LATCbits.LATC7
#define 	CSB_TRIS			TRISCbits.TRISC7

// ANDRIS
#define 	RW					LATBbits.LATB4
#define 	RW_TRIS				TRISBbits.TRISB4

#define 	E					LATBbits.LATB3
#define 	E_TRIS				TRISBbits.TRISB3

#define 	DATA_DIR_IN()		TRISD=0xFF
#define 	DATA_DIR_OUT()	  	TRISD=0
#define 	WR_DATA(a)			LATD=(a)		
#define 	RD_DATA()			PORTD		
	
#define 	DATA				0
#define 	COMMAND				1

#define		OUT					0
#define		IN					1
//************************************************************************
// PROTOTYPES
//************************************************************************
extern void InitDisplay(void);
extern unsigned char BusyCheck(void);
extern void PageSelect(unsigned char page);

void ChipSelect(unsigned char chip);
void ClearDisplay(void);
void RowSelect(unsigned char row);
void StartLine(unsigned char line);
void WriteChar(unsigned char ch, unsigned char rev);
void WriteByte(unsigned char byt);
void Plot(unsigned char rx, unsigned char ry);
void Unplot(unsigned char rx, unsigned char ry);
void Line(int x0, int y0, int x1, int y1);
