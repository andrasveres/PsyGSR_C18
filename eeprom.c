#include "eeprom.h"
//#include <i2c.h>
//#include "i2c_extras.h"
//#include <p18f4550.h>
#include <sw_i2c.h>

void WriteLongEEPROM(unsigned long addr, unsigned long i) {
   WriteEEPROM(addr,   (i & 0x000000ff));
   WriteEEPROM(addr+1, (i & 0x0000ff00)>>8);
   WriteEEPROM(addr+2, (i & 0x00ff0000)>>16);
   WriteEEPROM(addr+3, (i & 0xff000000)>>24);
}

int ReadLongEEPROM(unsigned long addr, unsigned long *i) {
   unsigned char c[4];
   unsigned long b0, b1, b2, b3;
   if(ReadEEPROM(addr, c, 4)!=0) return -1;

   b0 = c[0]; b1 = c[1], b2=c[2], b3=c[3];
   *i = b0 + (b1<<8) + (b2<<16) + (b3<<24);

   return 0;
}

int ReadCharEEPROM(unsigned long addr, unsigned char *c) {
   // unsigned char c[1];
   int ret = ReadEEPROM(addr, c, 1);

   return ret;
}

/*
void WriteEEPROM(unsigned long addr, unsigned char data) {
    unsigned char ControlByte = 0xA0;       
    unsigned char HighAdd, LowAdd;

    if(addr>65535) ControlByte = ControlByte | 0x08; // block select for addresses over 512kbit

    HighAdd = (addr & 0x0000ff00) >> 8;
    LowAdd = addr & 0x000000ff;   

    //sprintf(lcd_msg, "E %lu %u %u", addr, HighAdd, LowAdd);    

    HDByteWriteI2C(ControlByte, HighAdd, LowAdd, data );
}

void ReadEEPROM(unsigned long addr, unsigned char *data, unsigned char length) {
    unsigned char ControlByte = 0xA0;       
    unsigned char HighAdd, LowAdd;

    if(addr>65535) ControlByte = ControlByte | 0x08; // block select for addresses over 512kbit

    HighAdd = (addr & 0x0000ff00) >> 8;
    LowAdd = addr & 0x000000ff;       

    HDByteReadI2C(ControlByte, HighAdd, LowAdd, data, length);
}
*/

void ack_poll(unsigned char ControlByte) {
    // ack poll
    SWStartI2C();
    SWPutcI2C( ControlByte); // control byte
    while( SWAckI2C() )
    {
       SWRestartI2C();
       //SWStartI2C(); 

       SWPutcI2C(ControlByte); // data
    }
    SWStopI2C();
}


int WriteEEPROM(unsigned long addr, unsigned char data) {
    unsigned char ControlByte = 0xA0;       
    unsigned char HighAdd, LowAdd;
    int ret=0;

    unsigned char *c = &addr;
    unsigned char hh = c[2];
    if(hh & 1) ControlByte = ControlByte | 0x08; // A16 address bit
    if(hh & 2) ControlByte = ControlByte | 0x02; // Chip select set for second chip

    HighAdd = (addr & 0x0000ff00) >> 8;
    LowAdd = addr & 0x000000ff;   

    SWStartI2C();
    if(SWPutcI2C(ControlByte)) {
       ret=-1; // control byte
       goto stop;
    }
    if(SWAckI2C()) {
       ret=-2;
       goto stop;
    }
    if(SWPutcI2C(HighAdd)) {
       ret=-3; // word address
       goto stop;
    }
    if(SWAckI2C()) {
       ret=-4;
       goto stop;
    }
    if(SWPutcI2C(LowAdd)) {
       ret=-5;// word address
       goto stop;
    }
    if(SWAckI2C()) {
       ret=-6;
       goto stop;
    }
    if(SWPutcI2C(data)) {
       ret=-7; // data
       goto stop;
    }
    if(SWAckI2C()) ret=-8;

    SWStopI2C();
    ack_poll(ControlByte);
    return ret;

 stop:
    SWStopI2C();
    return ret;

}

int ReadEEPROM(unsigned long addr, unsigned char *data, unsigned char length) {
    unsigned char ControlByte = 0xA0;       
    unsigned char HighAdd, LowAdd;
    int ret=0;

    unsigned char *c = &addr;
    unsigned char hh = c[2];

    //if(addr>65535) ControlByte = ControlByte | 0x08;

    if(hh & 1) ControlByte = ControlByte | 0x08; // A16 address bit
    if(hh & 2) ControlByte = ControlByte | 0x02; // Chip select set for second chip

    HighAdd = (addr & 0x0000ff00) >> 8;
    LowAdd = addr & 0x000000ff;       

    SWStartI2C();
    if(SWPutcI2C(ControlByte)) {
       ret=-1; // control byte
       goto stop; 
    }
    if(SWAckI2C()) {
       ret=-2;
       goto stop;
    }
    if(SWPutcI2C(HighAdd)) {
       ret=-3; // word address
       goto stop;
    }
    if(SWAckI2C()) {
       ret=-4;
       goto stop;
    }
    if(SWPutcI2C(LowAdd)) {
       ret=-5; // word address
       goto stop;
    }
    if(SWAckI2C()) {
       ret=-6;
       goto stop;
    }

    SWRestartI2C();
    if(SWPutcI2C(ControlByte | 1)) {
       ret=-7; 
       goto stop;
    }
    if(SWAckI2C()) {
       ret=-8;
       goto stop;
    }

    if(SWGetsI2C(data, length)) {
       ret=-9;
       goto stop;
    }

    SWNotAckI2C();

   stop:
    SWStopI2C();

    return ret;

    //HDByteReadI2C(ControlByte, HighAdd, LowAdd, data, length);
}

void PageWriteEEPROM(unsigned long addr, unsigned char *data, unsigned char length) {
    unsigned char ControlByte = 0xA0;       
    unsigned char HighAdd, LowAdd;
    int i;

    unsigned char *c = &addr;
    unsigned char hh = c[2];

    if(hh & 1) ControlByte = ControlByte | 0x08; // A16 address bit
    if(hh & 2) ControlByte = ControlByte | 0x02; // Chip select set for second chip


    HighAdd = (addr & 0x0000ff00) >> 8;
    LowAdd = addr & 0x000000ff;   

    SWStartI2C();
    SWPutcI2C(ControlByte); // control byte
    SWAckI2C();
    SWPutcI2C(HighAdd); // word address
    SWAckI2C();
    SWPutcI2C(LowAdd); // word address
    SWAckI2C();

    for(i=0; i<length; i++) {
       SWPutcI2C(data[i]); // data
       SWAckI2C();
    }
    
    SWStopI2C();

    ack_poll(ControlByte);
}

void PageClearEEPROM(unsigned long addr) {
    unsigned char ControlByte = 0xA0;       
    unsigned char HighAdd, LowAdd;
    int i;

    unsigned char *c = &addr;
    unsigned char hh = c[2];

    if(hh & 1) ControlByte = ControlByte | 0x08; // A16 address bit
    if(hh & 2) ControlByte = ControlByte | 0x02; // Chip select set for second chip


    HighAdd = (addr & 0x0000ff00) >> 8;
    LowAdd = addr & 0x000000ff;   

    SWStartI2C();
    SWPutcI2C(ControlByte); // control byte
    SWAckI2C();
    SWPutcI2C(HighAdd); // word address
    SWAckI2C();
    SWPutcI2C(LowAdd); // word address
    SWAckI2C();

    for(i=0; i<128; i++) {
       SWPutcI2C(0); // data
       SWAckI2C();
    }
    
    SWStopI2C();

    ack_poll(ControlByte);
}
