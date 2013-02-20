
#include <p18cxxx.h>
#include "ks108.h"

extern const rom char Font7x5[0x80*5]; 
extern const rom char Font8x8[];

void Andris_Delay1us(void)
{
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
}

void Delay100us(unsigned char time)
{
 unsigned char i, j;

 for(i=0; i<time; i++)
  {
      // ANDRIS: This was reduced to make lcd faster
      // now it is 10us actually
      for(j=0; j<10; j++) Andris_Delay1us();
  }
}

void Delay1ms(unsigned int time) {
   unsigned int i;

   for(i=0; i<time; i++) {
      Delay100us(10);
   }
}



//************************************************************************
// void InitDisplay(void)
//************************************************************************
void InitDisplay(void)
{
unsigned char i, j, k;
	DATA_DIR_OUT();
	WR_DATA(0x3F);
	for(i = 0; i < 3; i ++)
	{
		ChipSelect(i);
		E = 1;
		Delay100us(1);
		E = 0;
	}
	ClearDisplay();
	ChipSelect(0);
	PageSelect(0);
	RowSelect(0);
	StartLine(0);

	for(i = 0; i < 3; i ++)
	{
       ChipSelect(i);
 	   StartLine(0);
    }   
}

//************************************************************************
// void WriteByte(unsigned char byt)
//************************************************************************
void WriteByte(unsigned char byt)
{
	WR_DATA(byt);
	E = 1;
	Delay100us(1);
	E = 0;
}


//************************************************************************
// void WriteByte(unsigned char byt)
//************************************************************************
unsigned char ReadByte(void)
{
unsigned char byt;
	DATA_DIR_IN();
	RW = 1;
	DI = 1;

	Delay100us(1);
	E = 1;
	Delay100us(1);
	E = 0;
	Delay100us(1);

	byt = RD_DATA();

	DATA_DIR_OUT();
}
/*
unsigned char ReadByte(void)
{
unsigned char byt;
	RW = 1;
	DI = 0;
	DATA_DIR_IN();
	E = 1;
	byt = RD_DATA();
	Delay100us(1);
	E = 0;
	DATA_DIR_OUT();
}
*/
//************************************************************************
// unsigned char BusyCheck(void)
//************************************************************************
unsigned char BusyCheck(void)
{
unsigned char by;
	DATA_DIR_IN();
	RW = 1;
	DI = 1;
	Delay100us(1);
	by = RD_DATA();
	DI = 0;
	by &= 0x80;
	RW = 0;
	DATA_DIR_OUT();
	return by;
}

//************************************************************************
// void ChipSelect(unsigned char page)
//************************************************************************
void ChipSelect(unsigned char chip)
{
	switch(chip)
	{
		case 0:
		{
			CSA = 0;
			CSB = 0;
		} break;
		case 1:
		{
			CSA = 0;
			CSB = 1;
		} break;
		case 2:
		{
			CSA = 1;
			CSB = 0;
		} break;
	}
}

//************************************************************************
// void PageSelect(unsigned char page)
//************************************************************************
void PageSelect(unsigned char page)
{
	RW = 0;
	DI = 0;
	WR_DATA(0xB8 | page);
	E = 1;
	Delay100us(2);
	E = 0;
//	ReadByte();
}

//************************************************************************
// void RowSelect -- Number of pixels to the right
//************************************************************************
void RowSelect(unsigned char row)
{
	RW = 0;
	DI = 0;
	WR_DATA(0x40 | row);
	E = 1;
	Delay100us(2);
	E = 0;
//	ReadByte();
}

//************************************************************************
// void StartLine(unsigned char page)
//************************************************************************
void StartLine(unsigned char line)
{
	RW = 0;
	DI = 0;
	WR_DATA(0xC0 | line);
	E = 1;
	Delay100us(2);
	E = 0;
//	ReadByte();
}

//************************************************************************
// void WriteChar(unsigned char ch)
//************************************************************************
void ClearDisplay(void)
{
unsigned char i, j, k;
	for(i = 0; i < 3; i ++)
	{
		ChipSelect(i);
		for(j = 0; j <8; j++)
		{
			PageSelect(j);
			RowSelect(0);
			WR_DATA(0);
			DI = 1;
			for(k = 0; k < 0x40; k++)
			{
				E = 1;
				Delay100us(1);
				E = 0;
			}
		}
	}
}

//************************************************************************
// void WriteChar(unsigned char ch)
//************************************************************************

// ANDRIS: 7x5-re cserelve
void WriteChar(unsigned char ch, unsigned char rev)
{
   unsigned char i, j;
   rom char *chr;
   int u;

   u = ch;
   u *= 5;

	DI = 1;
	RW = 0;
//	if(ch > 0x5E)
//		ch--;
	chr = Font7x5 + u;
 
	for(i = 0; i < 5; i++)
	{
//	j = *chr;
	if(rev)
		WriteByte(~(*chr++));
	else
		WriteByte(*chr++);
	}
	WriteByte(0);
	
}


/*
void WriteChar(unsigned char ch, unsigned char rev)
{
unsigned char i, j;
rom char *chr;
	DI = 1;
	RW = 0;
//	if(ch > 0x5E)
//		ch--;
	chr = Font8x8 + ch*8;
	for(i = 0; i < 8; i++)
	{
//	j = *chr;
	if(rev)
		WriteByte(~(*chr++));
	else
		WriteByte(*chr++);
	}
//	WriteByte(0);
	
}
*/
//************************************************************************
// void WriteCharAt(unsigned char xpos, unsigned char ypos, unsigned char ch, unsigned char rev)
//************************************************************************
void WriteCharAt(unsigned char xpos, unsigned char ypos, unsigned char ch, unsigned char rev)
{
unsigned char i, j;
rom char *chr;
	if(xpos > 23 | ypos > 7)
		return;
	StartLine(0);
	ChipSelect(xpos/8);
	PageSelect(ypos);
	RowSelect(xpos * 8);
	DI = 1;
	RW = 0;
//	if(ch > 0x5E)
//		ch--;
	chr = Font7x5 + ch*5;
	for(i = 0; i < 5; i++)
	{
//	j = *chr;
	if(rev)
		WriteByte(~(*chr++));
	else
		WriteByte(*chr++);
	}
}
/*
void WriteCharAt(unsigned char xpos, unsigned char ypos, unsigned char ch, unsigned char rev)
{
unsigned char i, j;
rom char *chr;
	if(xpos > 23 | ypos > 7)
		return;
	StartLine(0);
	ChipSelect(xpos/8);
	PageSelect(ypos);
	RowSelect(xpos * 8);
	DI = 1;
	RW = 0;
//	if(ch > 0x5E)
//		ch--;
	chr = Font8x8 + ch*8;
	for(i = 0; i < 8; i++)
	{
//	j = *chr;
	if(rev)
		WriteByte(~(*chr++));
	else
		WriteByte(*chr++);
	}
}
*/
//************************************************************************
// void WriteROMString(unsigned char str[])
//************************************************************************
void WriteROMString(const rom unsigned char *str, unsigned char xpos, unsigned char ypos, unsigned char rev)
{
static unsigned char x, y, z, u;
	x = xpos;
	y = ypos;
	StartLine(0);
	while(*str)
	{
		if(x > 0x17 || y > 7) return;
		z = x/8;
		ChipSelect(z);
		PageSelect(y);
		RowSelect((x - z*8)*8);
		WriteChar(*str, rev);
		x++;
		str++;
	}
}

//************************************************************************
// void WriteROMString(unsigned char str[])
//************************************************************************
void WriteRAMString(unsigned char str[])
{
//unsigned char *string;
	while(*str)
	{
		WriteChar(*str, 0);
		str++;
	}
}

//************************************************************************
// unsigned char Button(void)
//************************************************************************

void Plot(unsigned char rx, unsigned char ry)
{
   unsigned char data;
   //lcd_waitbusy();

   // select the correct side
   //StartLine(0);

   if (rx & 64)
      ChipSelect(1);
   else
      ChipSelect(2);

   PageSelect( ry >> 3);
   RowSelect( rx );

   data = ReadByte(); // dummy read needed here
   data = ReadByte();

   RowSelect( rx );

   DI = 1;
   RW = 0;
   WriteByte (data | (1 << (ry & 7)));
   
}

void Unplot(unsigned char rx, unsigned char ry)
{
   unsigned char data;
   //lcd_waitbusy();

   // select the correct side
   //StartLine(0);

   if (rx & 64)
      ChipSelect(1);
   else
      ChipSelect(2);

   PageSelect( ry >> 3);
   RowSelect( rx );

   data = ReadByte(); // dummy read needed here
   data = ReadByte();

   RowSelect( rx );

   DI = 1;
   RW = 0;
   // WriteByte (data & (~(1 << (ry & 7))));
   WriteByte (0);   
}

#define fabs(n)  ( ((n) >= 0.0) ? (n) : -(n) )
#define abs(n)  ( ((n) >= 0) ? (n) : -(n) )

void Line(int x0, int y0, int x1, int y1) {
   int dx = abs(x1-x0);
   int dy = abs(y1-y0);
   int sx, sy, err, e2;

   if (x0 < x1) sx = 1; else sx = -1;
   if (y0 < y1) sy = 1; else sy = -1;
   err = dx-dy;
 
   while(1) {
     Plot(x0,y0);
     if (x0 == x1 && y0 == y1) return;
     e2 = 2*err;
     if (e2 > -dy) { 
       err = err - dy;
       x0 = x0 + sx;
     }
     if (e2 <  dx) { 
       err = err + dx;
       y0 = y0 + sy;
     }

   }
}

