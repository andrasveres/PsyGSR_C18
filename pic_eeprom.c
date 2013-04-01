#include "p18cxxx.h"

unsigned char ReadPicEEPROM(unsigned char address){
    EEADR=address;
    EECON1bits.EEPGD= 0; // 0 = Access data EEPROM memory
    EECON1bits.CFGS = 0; // 0 = Access Flash program or DATA EEPROM memory
    EECON1bits.RD   = 1; // EEPROM Read
    return EEDATA;       // return data
}
 
void WritePicEEPROM(unsigned char address,unsigned char data){
   unsigned char INTCON_SAVE;

    EEADR  = address;
    EEDATA = data;

    EECON1bits.EEPGD= 0; // 0 = Access data EEPROM memory
    EECON1bits.CFGS = 0; // 0 = Access Flash program or DATA EEPROM memory
    EECON1bits.WREN = 1; // enable writes to internal EEPROM

    INTCON_SAVE=INTCON; // Save INTCON register contants
    INTCON=0;             // Disable interrupts, Next two lines SHOULD run without interrupts
    
    EECON2=0x55;        // Required sequence for write to internal EEPROM
    EECON2=0xaa;        // Required sequence for write to internal EEPROM

    EECON1bits.WR=1;    // begin write to internal EEPROM
    INTCON=INTCON_SAVE; //Now we can safely enable interrupts if previously used
    
    Nop();

    while (PIR2bits.EEIF==0)//Wait till write operation complete
    {
        Nop();
    }

    EECON1bits.WREN=0; // Disable writes to EEPROM on write complete (EEIF flag on set PIR2 )
    PIR2bits.EEIF=0; //Clear EEPROM write complete flag. (must be cleared in software. So we do it here)
}