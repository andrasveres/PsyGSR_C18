
// Ez a kod a PIC18F4550-re mukodik GSR katyaval grafikus LCD
// RTC-t kivettem belole az egyszeruseg kedveert ezert kell neki egy 32.768 kristaly
// az aramot is 7805-tel kapja 9v-rol
// 

// Szukseges: MPLAB IDE v8.83
//            Nem biztos: Microchip Solutions v2011-12-05 (innen jon az USB)
//            MPLAB C18
//          

#ifndef MAIN_C
#define MAIN_C



/** INCLUDES *******************************************************/
#include "./USB/usb.h"
#include "HardwareProfile.h"
#include "./USB/usb_function_hid.h"
#include "font.h"
#include "eeprom.h"
#include <delays.h>
#include <string.h>

//Andris includes
#include <i2c.h>
#include <sw_i2c.h>
#include "xlcd.h"



/** CONFIGURATION **************************************************/
// Configuration bits for PICDEM FS USB Demo Board (based on PIC18F4550)
        #pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
        #pragma config CPUDIV   = OSC1_PLL2   // Andras 24Mhz system clock
        //#pragma config CPUDIV   = OSC4_PLL6     // Andras 12
        #pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
        #pragma config FOSC     = HSPLL_HS // Andras eredeti
        //#pragma config FOSC     = HS
        #pragma config FCMEN    = OFF
        #pragma config IESO     = OFF
        #pragma config PWRT     = OFF
        #pragma config BOR      = ON
        #pragma config BORV     = 3
        #pragma config VREGEN   = ON      //USB Voltage Regulator
        #pragma config WDT      = OFF
        #pragma config WDTPS    = 32768
        #pragma config MCLRE    = ON
        #pragma config LPT1OSC  = ON   // KULONBEN NAGYON GYORSAN FUTOTT, EZ EGY HACK
        #pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
        #pragma config STVREN   = ON
        #pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
        #pragma config XINST    = OFF       // Extended Instruction Set
        #pragma config CP0      = OFF
        #pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
        #pragma config CPB      = OFF
//      #pragma config CPD      = OFF
        #pragma config WRT0     = OFF
        #pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
        #pragma config WRTB     = OFF       // Boot Block Write Protection
        #pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
        #pragma config EBTR0    = OFF
        #pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
        #pragma config EBTRB    = OFF


#pragma romdata

const far rom char *VERSION = "PsyGSR 0.2.1d";


/** VARIABLES ******************************************************/
#pragma udata

#if defined(__18F4550)
    #pragma udata USB_VARIABLES=0x500
#else
    #pragma udata
#endif

#define USBSIZE 64

int connected=0;
unsigned far char ReceivedDataBuffer[USBSIZE];
unsigned far char ToSendDataBuffer[USBSIZE];

#define IDLE 0
#define REC 1
#define FULL 2

// RECORD TYPES in EEPROM
#define T_MEAS  0b00000000
#define T_GSR   0b00100000
#define T_MARK  0b01000000 
#define T_BAT   0b01100000
#define T_BPM   0b10000000
#define T_PP    0b10100000
//#define T_DT    0b11000000

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma udata



unsigned long msec;
float fmsec=0;

unsigned long last_msec=0;
unsigned long last_lcd=0;
unsigned long sw_on=0;
unsigned long eeprom_addr=0; // current address we are writing to

float hgsr=0; // last GSR state
float fgsr=0;
int gsr_n=0;

char state=IDLE; // we start with idle state
unsigned char meas=0; // current measurement
unsigned long meas_start=0;
int nmark=0;
unsigned long last_tl = 0;
unsigned char c=0;

int n_inth=0; // number of high prio interrupts
int n_intl=0; // number of low prio interrupts

USB_HANDLE USBOutHandle = 0;
USB_HANDLE USBInHandle = 0;
BOOL blinkStatusValid = TRUE;

unsigned char rtc_s, rtc_m, rtc_h;


unsigned long last_pulse=0;
//int pulse=0; //last pulse reading
double avg_pulse=0;
double avg_pulse_fast=0;
double pulse_level=0;
int max_pulse = 0;
int pp=0;

int pulse_vec[10]={0,0,0,0,0,0,0,0,0,0};
int pulse_vec_ptr = 0;

int beat_detected = 0;

int n_beat=0;
unsigned long last_beat_calc=0;
unsigned long last_pulserec=0;
unsigned int bpm=0;

unsigned long last_bat = 0;
unsigned int bat=0;

#pragma udata otherdata
//char bigbuf[128];
int errcnt=0;


typedef struct Config {
   unsigned int empty;
   unsigned int GSR_DT;     // GSR write period
   char WRITE_PP;    // if 1 write inter-pulse time
   unsigned int BPM_DT;  // BPM write period
   unsigned long timestamp[2]; // java timestamp
} Config ;

Config conf = {0, 250, 1, 2, 0, 0};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////



/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
static void InitializeSystem(void);
void ProcessIO(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
WORD_VAL ReadPOT(void);

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
    float dt;
    int intf=0;
	void YourHighPriorityISRCode()
	{

        //n_inth++;
        // INTCONbits.TMR0IF = 0;
        if(PIR1bits.TMR1IF) {
           //unsigned char dtimer = 0b11111000; // 62.547727ms
           //const unsigned char dtimer = 0b11111110; // ~15ms 
           const unsigned char dtimer = 0b11111111; // ~7.5ms 
           const unsigned char dtimerl = 128; // substr. approx 4ms

           n_inth++;
           intf=1;
           
           while((TMR1L&0x01));
           while(!(TMR1L&0x01));

           TMR1L+=dtimerl;
           TMR1H = dtimer; 

           dt = 1000.0 / 32768.0 * (65536-((float)dtimer*256));
           dt-=1000.0*(128)/32768.0;
           
           fmsec += dt;
           msec = (unsigned long)fmsec;

           PIR1bits.TMR1IF = 0;

           // USUAL SEQUENCE

   		   //Check which interrupt flag caused the interrupt.
		   //Service the interrupt
		   //Clear the interrupt flag
       }
       
       //if(!USB_BUS_SENSE) USBDeviceDetach();

       //#if defined(USB_INTERRUPT)
       USBDeviceTasks();
       //#endif
      
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
        //n_intl++;
        //INTCONbits.TMR0IF = 0;
        //PIR1bits.TMR1IF = 0;

		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif

/** DECLARATIONS ***************************************************/
#pragma code


/// ANDRAS
void InitI2C(void)
{

return;

//OpenI2C(MASTER, SLEW_ON);
//SSPADD=0x09;
//return;

  //here is the I2C setup from the Seeval32 code.
  TRISBbits.RB1 = 1; //Configure SCL as Input
  TRISBbits.RB0 = 1; //Configure SDA as Input
  SSPSTAT = 0x80;   //Disable SMBus & Slew Rate Control
  SSPCON1 = 0x28;   //Enable MSSP Master
  //SSPADD = 0x18;    //Should be 0x18 for 100kHz
  SSPADD = 0x7E;    //Should be 0x77 for 100kHz 
  SSPCON2 = 0x00;   //Clear MSSP Conrol Bits
}


///////////////////////// ANDRIS ---- Delays for LCD module created by AppMaestro code generator
void Delay_ms(unsigned int i) {
   unsigned int j, k;
   for(j=0; j<i; j++) {
      for(k=0; k<10; k++) Delay100TCYx(12);
   }
}

void Delay_us(unsigned int i) {
   unsigned int j, k;
   for(j=0; j<i; j++) {
      Delay10TCY();
      Delay1TCY();Delay1TCY();
   }
}

void XLCDDelay15ms (void)
{
    Delay_ms(15);
    return;
}

void XLCDDelay4ms (void)
{
    Delay_ms(4);
    return;
}
void XLCD_Delay500ns(void)
{
    Nop();
    Nop();
    Nop();
}
void XLCDDelay(void)
{
    int i;
    for(i=0;i<1000;i++)
        {
        Nop();
        }
    return;

}


void ProcessLcd();


unsigned int ReadBattery(void) {
    unsigned long i;

    TRISAbits.TRISA0=1;
    ADCON0=0b00001101;  // AN3
    ADCON2=0x3C;
    ADCON2bits.ADFM = 1;

    Delay_us(10);

    ADCON0bits.GO = 1;              // Start AD conversion
    while(ADCON0bits.NOT_DONE);     // Wait for conversion

    i = ADRESH;
    i = i*256 + ADRESL;

    return i;
}

unsigned int ReadGsr(void) {
    unsigned int i;

    TRISAbits.TRISA0=1;
    ADCON0=0x01;   // AN0

    ADCON2 = 0b10111110;
    ADCON2bits.ADFM = 1;


    Delay_us(10);

//    ADCON0=0x00;                    // select AN0
    ADCON0bits.GO = 1;              // Start AD conversion
    while(ADCON0bits.NOT_DONE);     // Wait for conversion

    i = ADRESL + (unsigned int)ADRESH*256;

    return i;
}

unsigned int ReadPulse(void) {
    unsigned int i;

    TRISAbits.TRISA1=1;
    ADCON0=0b00000101; // AN1

    ADCON2 = 0b10111110;
    ADCON2bits.ADFM = 1;

    Delay_us(10);

//    ADCON0=0x00;                    // select AN0
    ADCON0bits.GO = 1;              // Start AD conversion
    while(ADCON0bits.NOT_DONE);     // Wait for conversion

    i = ADRESL + (unsigned int)ADRESH*256;

    return i;
}

unsigned long msg_set = 0; // time when msg was printed
void ShowMsg() {
   //XLCDClear();
   msg_set = msec;
}



void PrintRec(void) {
   char buf[20];
   int min;
   int sec;
   unsigned long s = (msec - meas_start)/1000;
   
   int meas_i = (int)meas;
   int nmark_i = (int)nmark;
   int gsr_i = (int)hgsr;
   int bpm_i = bpm;
   

   min = s / 60;
   sec = s - min * 60;
   sprintf(buf, (const far rom char*)"T=%2d:%02d M%d H%d ", (int)min, (int)sec, (int)meas_i, (int)nmark_i);
   //sprintf(buf, "%lu  ", eeprom_addr);

   XLCDL1home();
   XLCDPutRamString(buf);

   sprintf(buf, (const far rom char*)"P=%3d G=%3d  ", bpm_i, gsr_i);
   // %d%% %d", bpm, (int) blev, b);
   XLCDL2home();
   XLCDPutRamString(buf);
}

void PrintIdle(void) {
   unsigned char c;
   unsigned int b;
   char buf[20];
   float blev;
   int bb;
   double f;
   int fi;


   if(state==FULL) {
      XLCDClear();
      XLCDL1home();
      XLCDPutRomString((const far rom char*)"MEMORY FULL");
      return;
   }

   // if(USB_BUS_SENSE==1) sprintf( buf, "USB CONNECTED");
   // else sprintf( buf, "NOTC %d",USB_BUS_SENSE);

   blev = bat;
   blev = 10.0*blev/1024.0;

   blev = (blev-7) / (9-7) * 100.0;

   bb = (int)blev;
   if(bb<0) bb = 0;
   if(bb>=100) bb=99;

   f = 100.0 * (eeprom_addr) / 131072.0 ;
   fi = (int)f;

   XLCDL1home();
   sprintf( buf, (const far rom char*)"BAT:%2d%% MEM:%2d%% ", (int)bb, (int)fi);
   XLCDPutRamString(buf);
  
   //sprintf( buf, "%2d %2d %2d", pbuf[0], pbuf[1], pbuf[2]);
   
   {
      int gsr_i = (int) hgsr;
      int meas_i = (int)meas;
      int bpm_i = (int) bpm;
      sprintf(buf, (const far rom char*)"M%d P%3d G%3d ", (int)meas, (int)bpm, (int)gsr_i);
      //sprintf(buf, "M%d P%3d G%3d", (int)meas, (int)bpm, (int)errcnt);
   }

   // %d%% %d", bpm, (int) blev, b);
   XLCDL2home();
   XLCDPutRamString(buf);
}

void PrintInfo(char *buf) {
   XLCDClear();
   XLCDL1home();
   XLCDPutRamString(buf);
   Delay_ms(1000);
   XLCDClear();
}

void PrintInfoROM(const far rom char *buf) {
   XLCDClear();
   XLCDL1home();
   XLCDPutRomString(buf);
   Delay_ms(1000);
   XLCDClear();
}

void ProcessLcd() {

   if(msec - last_lcd < 200) return;
   last_lcd = msec;

   if(state==REC) {
      PrintRec();
      return;
   }

   PrintIdle();      
}

void StoreDataType(unsigned char type, unsigned int value) {
   unsigned char *c = (unsigned char*) &value;

   if(eeprom_addr>=2*65536) {
      state=FULL;
      return;
   }

   while(WriteEEPROM(eeprom_addr, c[0])!=0) {errcnt++;};
   while(WriteEEPROM(eeprom_addr+1, c[1] | type)!=0) {errcnt++;};
   eeprom_addr += 2;
}

void StoreData(unsigned char type, unsigned int value) {
   unsigned char *c = (unsigned char *)&eeprom_addr;

   if( (eeprom_addr & 0b00111111) == 0) {
      StoreDataType(T_MEAS, (unsigned int)meas);
      //StoreDataType(T_DT, (unsigned int)conf.GSR_DT);
   }
   StoreDataType(type, value);
}

void ProcessBat(void) {
   int dt = 20000;
   if(msec >= last_bat + dt) {
      bat = ReadBattery();
      last_bat = msec;
      if(state==REC) StoreData(T_BAT, bat);
   }
}

void ProcessGsr(void) {
   unsigned int i;
   i = ReadGsr(); 
  
   fgsr += i;
   gsr_n++;

   // dt msec has elapsed
   if(msec >= last_msec + conf.GSR_DT) {
      
      hgsr = fgsr / gsr_n;
      fgsr=0;
      gsr_n=0;

      if(state==REC) {
         i = (unsigned int)(4.0*hgsr);
         StoreData(T_GSR, i);
      }

      last_msec += conf.GSR_DT; 
   }


}

// at startup, read current state
void InitMeas(void) {
   unsigned long i;
   unsigned char c;
   // find last block
   
   state = IDLE;

   PrintInfoROM((const far rom char*)VERSION);

   for(i=0; i<2*65536; i+=64) {
      unsigned char c;
      int ret;

      while(ReadCharEEPROM(i, &c)!=0) {errcnt++;};

      if(c==0) break;
   }

   if(i>=2*65536) {
      // full
      state = FULL;
      return;
   }

   if(i==0) {
      // empty
      meas = 1;
      eeprom_addr = 0;
      return;
   }

   // read last
   eeprom_addr = i;

   while(ReadCharEEPROM(i-64, &c)!=0) {errcnt++;};

   if(c==0) {
      while(1) PrintInfoROM((const far rom char*)"Call Andras");
   }

   meas = c+1;

}

void StartMeas(void) {
   unsigned long m;
   int i;

   if(eeprom_addr % 64 != 0) {
      PrintInfoROM((const far rom char*)"StartMeas Error");
      return;
   }

   state = REC;

   PrintInfoROM("Starting");

   nmark=0;
   meas_start=msec;

   last_msec = msec;
   last_pulserec = msec;

}


void HardReset(void) {
   unsigned long i;
   char buf[20];

   XLCDClear();

   XLCDL1home();
   XLCDPutRomString((const far rom char*)"HARD RESET");

   for(i=0; i<65536*2; i+=128) {
      PageClearEEPROM(i);   

      if(i%100 == 0) {
         XLCDL2home();
         sprintf( buf, (const far rom char*)"%lu", i);
         XLCDPutRamString(buf);
      }
   }

   XLCDL1home();
   sprintf( buf, (const far rom char*)"OK");
   XLCDPutRamString(buf);
   
   InitMeas();

}



// process switch button
char longpress_processed = 0;
char marker_processed = 0;
void ProcessSw(void) {
   unsigned long dt=0;

   if(sw==1) {
      sw_on = 0;
      marker_processed = 0;
      longpress_processed = 0;
      return;
   }

   if(sw_on==0) sw_on = msec;

   dt = msec - sw_on;

   if(dt>10 && state == IDLE && longpress_processed == 0) {

      marker_processed = 1; // suppress marker processing

      StartMeas();
      return;
   }

   if(dt>10 && marker_processed == 0 && state == REC) {

      // USBDeviceDetach(); TEST: POWER

      marker_processed = 1;
      
      nmark++;
      StoreData(T_MARK, (unsigned int)nmark);
   }

   if(dt>3000 && longpress_processed == 0  && state==REC) {
      longpress_processed = 1;

      InitMeas();
      
   }
}

void ProcessPulse(void) {

   if(msec >= last_pulse + 10) {               
        int p;

        // if beat is lost, delete history
        if(msec > last_beat_calc + 2000) {
           int i;
           last_beat_calc = msec;
           for(i=0; i<10; i++) pulse_vec[i]=0;
           pulse_vec_ptr = 0;
           bpm = 0;
        }

        p = ReadPulse();
        avg_pulse = 0.99 * avg_pulse + 0.01 * p;
        avg_pulse_fast = 0.2 * avg_pulse + 0.8 * p;

        if(p>max_pulse) max_pulse = p;

        if(beat_detected==1 && p < avg_pulse) {
           beat_detected=0;
           pulse_level = avg_pulse;
           max_pulse = 0;
           pulse_led=0;

        } else if(beat_detected==0 && p > avg_pulse && max_pulse > 250) {
           int dt = msec - last_beat_calc;
           int sum = 0;
           int n=0;
           int i;

           last_beat_calc = msec;

           beat_detected=1;
           pulse_led=1;
           n_beat++;

           if(state==REC && conf.WRITE_PP==1) StoreData(T_PP, dt);
           pp=dt;

           pulse_vec[pulse_vec_ptr] = dt;

           for(i=0; i<10; i++) {
              int d = pulse_vec[i];
              if(d<2000 && d>0) {
                 sum += d;
                 n++;
              }
           }

           // if at least 5 beats available, calculate bpm
           if(n>5) bpm = (int) (1.0 * n * 60000.0 / sum);

           pulse_vec_ptr++;
           if(pulse_vec_ptr>=10) pulse_vec_ptr = 0;

        } 
        last_pulse = msec;           

   } 

   if(msec >= last_pulserec + conf.BPM_DT) {

      last_pulserec += conf.BPM_DT;
      
      if(state==REC) StoreData(T_BPM, (unsigned int)bpm);
   }
}

/********************************************************************
 *******************************************************************
*******************************************************************
*******************************************************************
*******************************************************************
*******************************************************************
*******************************************************************
*******************************************************************
*******************************************************************
 *******************************************************************/

void main(void)
{   

    XLCDInit();  
    
    ADCON1 |= 0x0F;                 // Default all pins to digital

    // PULSE LED
    tris_pulse_led=0; 
    pulse_led = 0;

    // SHUTDOWN USB
    // UCONbits.USBEN=0;

    INTCON2bits.RBPU = 0; // 0=enable pull ups on portB

    // voltage reference config
    CVRCON = 0b11100011; //(lowest 4 bits: from 5V * D * 2/3)
    //CVRCON = 0b11100111; //(lowest 4 bits: from ~ 0V to 3V)

    // usb
    InitializeSystem(); 

    InitI2C();
    mInitSwitch();

    //eeprom_addr = ReadLongEEPROM(0);

    //HardReset();


    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif

    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = 1; // See HardwareProfile.h
    #endif


/*
    // enable timer0 and set scale down factor to 256
    // TIMER0 test
    INTCONbits.TMR0IF = 0; // clear timer flag
    T0CON = 0b00000001; // 256 prescaler
    TMR0H = 0; 
    TMR0L = 0; 
    T0CONbits.TMR0ON = 0; // enable interrupt
    INTCONbits.TMR0IE = 0; //enable 
*/

    // TIMER1 test
    T1CONbits.T1OSCEN=1; // enable/disable on-chip 32khz oscillator, 0=external
    T1CONbits.TMR1CS=1; // 1=32khz 0=Fosc/4
    T1CONbits.T1CKPS1 =0; // scaler bit1
    T1CONbits.T1CKPS0 =0; // scaler bit0 
    T1CONbits.RD16 =0; // 1=16 bit counter enabled
    T1CONbits.T1SYNC =1; // 1=switch off sync mode
    PIR1bits.TMR1IF = 0; // clear timer flag
    TMR1H = 0b11110000; // preset to 125msec
    TMR1L = 0;
    T1CONbits.TMR1ON = 1; // enable interrupt
    PIE1bits.TMR1IE = 1; // enable interrupt

    // enable all interrupts
    RCONbits.IPEN=1;
    INTCONbits.GIEH=1;

    TRISAbits.TRISA0=1; // GSR input pin
    TRISAbits.TRISA2=1; // Battery input pin

    ADCON1=0b00101011; // select ports 0,1,2,3 as Analog
    ADCON2=0x3C;         // set A/D clock and A/D duration
    ADCON2bits.ADFM = 1; // A/D result right justified

    msec=0;
    last_msec=0;
    eeprom_addr = 0;
    //gsr = 0;

    //UpdateTime();

    InitMeas(); 

    USBDeviceAttach(); //////////////////// TEST

    bat = ReadBattery();
  
    while(1)
    {

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        //UpdateTime();

        ProcessIO();

        if(intf) {
           intf=0;

           ProcessSw();
           ProcessGsr();
           ProcessLcd();
           ProcessPulse();
           ProcessBat();        
        }

        if(!USB_BUS_SENSE) {   
           //USBDeviceDetach(); // may not need this?
           OSCCONbits.IDLEN=0;
           //TEST_LED = 0;
           Sleep();        
           //TEST_LED = 1;
        } //else USBDeviceAttach();

        //while(!OSCCONbits.OSTS);

    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *******************************************************************/
static void InitializeSystem(void)
{

    USBOutHandle = 0;
    USBInHandle = 0;

    blinkStatusValid = TRUE;
    
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
}//end InitializeSystem



/********************************************************************
 * Function:        void ProcessIO(void)
 * *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *******************************************************************/
void ProcessIO(void)
{   
    //Blink the LEDs according to the USB device status
    if(blinkStatusValid)
    {
        BlinkUSBStatus();
    }

    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    
    if(!HIDRxHandleBusy(USBOutHandle))				//Check if data was received from the host.
    {   
        switch(ReceivedDataBuffer[0])				//Look at the data the host sent, to see what kind of application specific command it sent.
        {
            case 0x34:	//Set connected flag
                {
	                while(HIDTxHandleBusy(USBInHandle));
	                 
					ToSendDataBuffer[0] = 0x34;  	//Echo back to the host the command we are fulfilling in the first byte.  In this case, the Read POT (analog voltage) command.
					connected = 1;

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;


            case 0x35:	//Reset connected flag
                {
	                while(HIDTxHandleBusy(USBInHandle));
	                 
					ToSendDataBuffer[0] = 0x35;  	//Echo back to the host the command we are fulfilling in the first byte.  In this case, the Read POT (analog voltage) command.
					connected = 0;

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x36:	//Check if already connected
                {
	                while(HIDTxHandleBusy(USBInHandle));
	                 
					ToSendDataBuffer[0] = 0x36;  	//Echo back to the host the command we are fulfilling in the first byte.  In this case, the Read POT (analog voltage) command.
					ToSendDataBuffer[1] = connected;

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x37:	//Read GSR current value
                {
                    unsigned int i;
                    unsigned char *c;

	                while(HIDTxHandleBusy(USBInHandle));
	                
                    i= ((int) (hgsr*16.0))/4;
                    c = (unsigned char*) &i;
 
					ToSendDataBuffer[0] = 0x37;  	//Echo back to the host the command we are fulfilling in the first byte.  In this case, the Read POT (analog voltage) command.
					ToSendDataBuffer[1] = c[0];
					ToSendDataBuffer[2] = c[1]; 

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x38:	// Get version
                {
                    WORD_VAL w;

	                while(HIDTxHandleBusy(USBInHandle)) ;
	                 
					// ToSendDataBuffer[0] = 0x38;
                    sprintf((const far char *)ToSendDataBuffer, (const far rom char*)VERSION);

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);
                }
                break;

            case 0x39:	// Get byte from EEPROM
                {
                    long addr = 0;
                    long b0 = ReceivedDataBuffer[1];
                    long b1 = ReceivedDataBuffer[2];
                    long b2 = ReceivedDataBuffer[3];
                    long b3 = ReceivedDataBuffer[4];

                    addr = b0 + (b1<<8) + (b2<<16) + (b3<<24);

	                while(HIDTxHandleBusy(USBInHandle)) ;
	                 
					ToSendDataBuffer[0] = 0x39;
                    ReadEEPROM(addr, ToSendDataBuffer+1, 1);

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x40:	// Get EEPROM in 64 bytes block
                {
                    long addr = 0;
                    long b0 = ReceivedDataBuffer[1];
                    long b1 = ReceivedDataBuffer[2];
                    long b2 = ReceivedDataBuffer[3];
                    long b3 = ReceivedDataBuffer[4];

                    addr = b0 + (b1<<8) + (b2<<16) + (b3<<24);

	                while(HIDTxHandleBusy(USBInHandle)) ;
	                
                    // this time we do not reflect the command back
					//ToSendDataBuffer[0] = 0x40;
                    ReadEEPROM(addr, ToSendDataBuffer+0, 64);

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x41:	// HARD RESET
                {
	                while(HIDTxHandleBusy(USBInHandle)) ;

                    // THIS SHOULD NOT BE HERE, LONG OPERATION SHOULD NOT HAPPEN HERE

                    USBDeviceDetach();
                    HardReset();
                    USBDeviceAttach();
                    return;
	                
					ToSendDataBuffer[0] = 0x41;

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x42:	//Read PULSE current value
                {
                    unsigned int i;
                    unsigned char *c;

	                while(HIDTxHandleBusy(USBInHandle));
	                
	                
	                // i = ReadPulse();
                    i = bpm;

                    c = (unsigned char*) &i;
 
					ToSendDataBuffer[0] = 0x42;  	//Echo back to the host the command we are fulfilling in the first byte.  In this case, the Read POT (analog voltage) command.
					ToSendDataBuffer[1] = c[0];
					ToSendDataBuffer[2] = c[1]; 

                    i = (int) avg_pulse;
                    ToSendDataBuffer[3] = c[0];
					ToSendDataBuffer[4] = c[1]; 

                    i = (int) avg_pulse_fast;
                    ToSendDataBuffer[5] = c[0];
					ToSendDataBuffer[6] = c[1]; 

                    i = (int) pp;
                    ToSendDataBuffer[7] = c[0];
					ToSendDataBuffer[8] = c[1]; 

                    pp=0;

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x50:	//Read MSEC
                {
                    unsigned char *c;

	                while(HIDTxHandleBusy(USBInHandle));

                    c = (unsigned char*) &msec;
 
					ToSendDataBuffer[0] = 0x50;  	//Echo back to the host the command we are fulfilling in the first byte.
					ToSendDataBuffer[1] = c[0];
					ToSendDataBuffer[2] = c[1]; 
                    ToSendDataBuffer[3] = c[2];
					ToSendDataBuffer[4] = c[3]; 

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x51:	//Read config
                {
                    unsigned char *c;

	                while(HIDTxHandleBusy(USBInHandle));

                    c = (unsigned char*) &msec;
 
					ToSendDataBuffer[0] = 0x51;  	//Echo back
                    
					memcpy((void far *)(ToSendDataBuffer+1), (const void far *)&conf, sizeof(conf));
					

	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

            case 0x52:	//Save config
                {
                    unsigned char *c;

	                while(HIDTxHandleBusy(USBInHandle));

                    c = (unsigned char*) &msec;
 
					ToSendDataBuffer[0] = 0x51;  	//Echo back
                    
					memcpy((void far *)&conf, (const void far *)(ReceivedDataBuffer+1), sizeof(conf));
					
	                USBInHandle = HIDTxPacket(HID_EP,(BYTE*)&ToSendDataBuffer[0],USBSIZE);	                					
                }
                break;

        }
        //Re-arm the OUT endpoint for the next packet
        USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,USBSIZE);
    }

    
}//end ProcessIO




/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
    if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
       // kivettem a villogast
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            //mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            //mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            //mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            //mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                //mLED_1_Toggle();
               // mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
               // mLED_1_Toggle();
               // if(mGetLED_1())
                //{
                 //   mLED_2_Off();
               // }
               // else
               // {
               //     mLED_2_On();
               // }
            }//end if
        }
    }
}//end BlinkUSBStatus




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
        //This function requires that the _IPL level be something other than 0.
        //  We can set it here to something other than 
        #ifndef DSPIC33E_USB_STARTER_KIT
        _IPL = 1;
        USBSleepOnSuspend();
        #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 ******************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,USBSIZE);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }      
    return TRUE; 
}

/** EOF main.c *************************************************/
#endif
