//
//  File........: hst_LS_KBD_NOHUB.c
//  Authors.....: 
//  Description.: 
//  
//
//  Conditions:
//    IFACE:  SPI
//
//  Date........: 
//
// The proper interface can be found in interface.c

#ifndef PER_LS_KBD_NOHUB_C
#define PER_LS_KBD_NOHUB_C

// Interrupt or not? - define only one.
//#define INT_DRIVEN
//#define POLLED

// Host or peripheral? - define only one.
#define HST
//#define PER

// Which interface? - define only one.
#define IFACE_SPI
//#define IFACE_I2C


//#include <stdio.h>
//#include <stdlib.h>
#include <intrinsics.h>      // MAXQ2000 specific stuff
#include <iomaxq200x.h>      // MAXQ2000 specific stuff
#include "RT53reg.h"         // RT99 Registers and bits
#include "..\interface.c"    // SPI/I2C Implementation
#include "..\usb.c"          // USB Implementation

// prototypes in this module
void Read_Keypad(void);
void prep_keypad(void);
//********************

///// Declare GLOBALS Here ///////////////////////

BYTE SUD[8];			// my copy of setup data

///////////////////////////////////////////////////

main()
{
///// Declare Variables Here ///////////////////////
    BYTE Set_Addr4[8]   = {0x00,0x05,0x04,0x00,0x00,0x00,0x00,0x00};
    BYTE Set_Config1[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
    BYTE Set_Idle[8]    = {0x21,0x0A,0x00,0x00,0x00,0x00,0x00,0x00};
    BYTE HR1,HR2,HR3;
    BYTE dum;
///////////////////////////////////////////////////

    SPI_Init();                 // set up MAXQ2000 to use its SPI port as a master


    wreg(rPINCTL,(bmFDUPSPI|bmPOSINT));//|bmINTLEVEL)); // RT53: INTLEVEL=0, POSINT=1 for pos edge interrupt pin


    // chip reset
    //  Reset_MAX(1000);        // RT53: Reset USB bus [SPI port must be set up before this will work]
    wreg(rUSBCTL,0x20);			
    timeDELAY(500);		// a delay
    wreg(rUSBCTL,0x00);		// remove the reset
    timeDELAY(2000);		// a delay

    // Set up the host
    wreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmSOFKAENAB|bmLOWSPEED|bmHOST));  // low speed
    //wreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmSOFKAENAB|bmHOST));  // full speed

    // LED RELATED MAXQ2000    
    PD0=0xFF;                           // MAXQ: Set port 0 to outputs
    PO0 = rreg(rRevision);              // MAXQ: 
    PD7 = 0x01;                         // MAXQ: P70 is scope TRIG signal

    // INTERRUPT RELATED MAXQ2000
//    // MAX3410E INT pin is tied to MAXQ2000 P60: make it an input
    PD6 &= ~0x01;                       // MAXQ: PD6.0=0 (turn off output)
    EIES1_bit.IT12 = 0;                 // MAXQ: 0=pos edge triggered IRQ
    EIE1_bit.EX12 = 1;                  // MAXQ: Enable Int12
    IMR  |= 0x02;                       // MAXQ: enable interrupts for module 1

//    wreg(rUSBIEN,0x00);
//    wreg(rGPINIEN,0x00);

    wreg( rHIEN, 0x00 );      // disable all interrupt
    wreg(rCPUCTL, bmIE);			        // Enable interrupt pin (for scope observation)
    __enable_interrupt();


///// Your Code Here //////////////////////////////

    // Reset the USB device
    wreg(rHIEN, bmBUSEVENTIE);           // enable the reset done irq
    wreg(rHCTL,bmBUSRST);               // initiate the 50 msec bus reset
    Wait_for_HIRQ(bmBUSEVENTIRQ);       // wait for and clear this interrupt
    //
    waitframes(4);

    // Set_Address to 4
    wreg(rPERADDR,0x00);                // send to address 0
    writebytes(rSUDFIFO,8,Set_Addr4);   // Load the SUDFIFO with Set_Address=4 request
    HR1 = CTL_WRITE();                  // this function returns only after completion 

    // Set_Config to 1
    wreg(rPERADDR,0x04);                // address is now 4
    writebytes(rSUDFIFO,8,Set_Config1); // Load the SUDFIFO with Set_Config=1 request
    HR2 = CTL_WRITE();

    // Send the "set_idle" request
    writebytes(rSUDFIFO,8,Set_Idle);    // Load the SUDFIFO with Set_Idle request
    HR3 = CTL_WRITE();

    //
    Read_Keypad();
    dum = rreg(rHRSL);
    dum >>= 6;
    dum &=0x03;
    wreg(rIOPINS2,dum);                 // show bus state
    //
    while(1);

}

//
// functions local to this module
//
// FUNCTION: Read_Keypad
// Send IN tokens to EP1, once every 10 frames.
// If NAK handshake, do nothing. If ACK, read the 8 keyboard data bytes in RCVFIFO, and
// write the keycode to the MAXQ2000 LED bar.
// 
void Read_Keypad(void)              // constantly read the keypad, return if the connect 
{                                   // state changes
    BYTE HR,rxnum,KB_data[8],newkey;
    wreg(rHIRQ,bmCONNIRQ);              // clear any remnants
    wreg(rHCTL,bmRCVTOG0);              // very first data toggle should be DATA0
    while(1)
    {
        waitframes(10);
        if(rreg(rHIRQ) & bmCONNIRQ)     // check for connect-status change
        {
            wreg(rHIRQ,bmCONNIRQ);      // clear the IRQ
            return;
        }                 
        wreg(rHXFR,0x01);               // send an IN token to EP1 (OUTNIN=0)
        HR = Wait_for_HIRQ(bmHXFRDNIRQ);   // this also clears the IRQ bit
        switch(HR)
        {
            case    hrNAK:  L8_ON  
                break;       // do nothing
            case    hrSUCCESS:                  // get the KB packet
                rxnum = rreg(rRCVBC);
                TRIG_HI                         // scope pulse brackets the 8-byte read
                readbytes(rRCVFIFO,rxnum,KB_data);  // read "rxnum" bytes from the RCVFIFO
          //      wreg(rHIRQ,bmRCVDAVIRQ);        // re-arm the endpont and SWITCH THE BUFFERS
                TRIG_LO
                newkey = KB_data[2];            // keycode is in the third byte 
                if(newkey == 0)             // turn off the LEDS when keys are up
                    PO0=0;                  // (comment out this test to latch keys in LEDS)  
                else if (newkey >= 0x59 && newkey <= 0x62)  // 89 is 1-key, 98 is 0-key
                    PO0 = newkey - 0x58;    // show it in the LEDS (0 shows as 0x0A)
                break;
            default:        // any other completion code is an error   
                PO0=0xFF;   // all bar LEDS on
                while(1);   // hang here and examine error code
        } // switch(HR)
    } // while(1)
////////////////////////////////////////////////////
} // End of main(void)

#endif
