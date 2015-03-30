//
//  File........: simple_test.c
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

#ifndef simple_test_C
#define simple_test_C

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
//#include "..\usb.c"          // USB Implementation


///// Declare GLOBALS Here ///////////////////////
int FLAG;
BYTE INTERRUPT;
int FRAMECOUNT;
BYTE errorcode;

BYTE SUD[8];			// my copy of setup data
// prototypes in this module
void Read_Keypad(void);
void prep_keypad(void);
//********************
///////////////////////////////////////////////////

main()
{
///// Declare Variables Here ///////////////////////
    static BYTE Set_Addr4[8]   = {0x00,0x05,0x04,0x00,0x00,0x00,0x00,0x00};
    static BYTE Set_Config1[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
    static BYTE Set_Idle[8]    = {0x21,0x0A,0x00,0x00,0x00,0x00,0x00,0x00};
    static BYTE HR1,HR2,HR3;
    static BYTE dum, result;
///////////////////////////////////////////////////

    // MAXQ2000 SPI port as a master
    SPI_Init();                 

    
    // disable interrupt pin
    wreg(rCPUCTL, 0x00);			        

   
   // disable all interrupt
    wreg(rUSBIEN,0x00);
    wreg(rGPINIEN,0x00);
    wreg( rHIEN, 0x00 );      

   
    // chip reset
    //  Reset_MAX(1000);        // RT53: Reset USB bus [SPI port must be set up before this will work]
    wreg(rUSBCTL,0x20);			
    timeDELAY(500);		// a delay
    wreg(rUSBCTL,0x00);		// remove the reset
    timeDELAY(2000);		// a delay

    
    // RT53: INTLEVEL=0, POSINT=1 for pos edge interrupt pin
//    wreg(rPINCTL,(bmFDUPSPI|bmPOSINT)); 

    // Set up the host
    wreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmSOFKAENAB|bmLOWSPEED|bmHOST));  // low speed
    //wreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmSOFKAENAB|bmHOST));  // full speed


    // LED RELATED MAXQ2000    
    PD0=0xFF;                    // MAXQ: Set port 0 to outputs
    PO0 = rreg(rRevision);       // MAXQ: 
    PD7 = 0x01;                  // MAXQ: P70 is scope TRIG signal


   // INTERRUPT RELATED MAXQ2000
    // MAX3410E INT pin is tied to MAXQ2000 P60: make it an input
    PD6 &= ~0x01;                // MAXQ: PD6.0=0 (turn off output)
    EIES1_bit.IT12 = 1;          // MAXQ: 0=pos edge triggered IRQ
    EIE1_bit.EX12 = 1;           // MAXQ: Enable Int12
    IMR  |= 0x02;                // MAXQ: enable interrupts for module 1
    
    // MAXQ: enable interrupts
    __enable_interrupt();
    
    // Enable interrupt pin
    wreg(rCPUCTL, bmIE);			        


///// Your Code Here //////////////////////////////

    timeDELAY(500);		// a delay

    // Reset the device on the USB bus
    wreg(rHIEN, bmBUSEVENTIE);           // enable the reset done irq
    wreg(rHCTL,bmBUSRST);               // initiate the 50 msec bus reset



 //   Wait_for_HIRQ(bmBUSEVENTIRQ);       // wait for and clear this interrupt
    wreg(rHIEN,bmBUSEVENTIRQ);         // enable only one IRQ in the HIEN register
    while( FLAG != bmBUSEVENTIRQ );     // hang until an irq 
//    wreg(rHIRQ,regbit);         // clear the irq
    wreg(rHIEN,0x00);           // all ints off
    result = (rreg(rHRSL) & 0x0F);       // get the completion code
    FLAG = 0;



//    waitframes(4);
    FRAMECOUNT = 0;
    while(FRAMECOUNT <= 4)
    {
        wreg(rHIEN,bmFRAMEIE);
        while( FLAG != bmFRAMEIRQ );
        FRAMECOUNT++;
        FLAG = 0;
    }
    wreg(rHIEN,0x00);


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

    Read_Keypad();

    dum = rreg(rHRSL);
    dum >>= 6;
    dum &=0x03;
    wreg(rIOPINS2,dum);                 // show bus state
    
    while(1);



/////////////////////////////////////////////////////
} // End of main(void)


void Read_Keypad(void)              // constantly read the keypad, return if the connect 
{                                   // state changes
BYTE HR, rxnum, KB_data[8], newkey;

//wreg(rHIRQ,bmCONNIRQ);              // clear any remnants

wreg(rHCTL,bmRCVTOG0);              // very first data toggle should be DATA0

    while(1)
    {
        waitframes(10);
/*        __disable_interrupt();
        if(rreg(rHIRQ) & bmCONNIRQ)     // check for connect-status change
        {
            wreg(rHIRQ,bmCONNIRQ);      // clear the IRQ
            return;
        }                 
        __enable_interrupt();
*/
//        FLAG = 0;
        wreg(rHXFR,0x01);               // send an IN token to EP1 (OUTNIN=0)
//        HR = Wait_for_HIRQ(bmHXFRDNIRQ);   // this also clears the IRQ bit
    wreg(rHIEN, bmHXFRDNIRQ);         // enable only one IRQ in the HIEN register
    while( FLAG != bmHXFRDNIRQ );     // hang until an irq 
    wreg(rHIEN,0x00);           // all ints off
    HR = (rreg(rHRSL) & 0x0F);       // get the completion code
    FLAG = 0;
        switch(HR)
        {
            case    hrNAK :                 // do nothing
                L8_ON  
                break;
            case    hrSUCCESS:              // get the KB packet
                rxnum = rreg(rRCVBC);
                TRIG_HI                             // scope pulse brackets the 8-byte read
                readbytes(rRCVFIFO,rxnum,KB_data);  // read "rxnum" bytes from the RCVFIFO
                wreg(rHIRQ,bmRCVDAVIRQ);            // re-arm the endpont and SWITCH THE BUFFERS
                TRIG_LO
                newkey = KB_data[2];                // keycode is in the third byte 
                if(newkey == 0)                     // turn off the LEDS when keys are up
                    PO0=0;                          // (comment out this test to latch keys in LEDS)  
                else if (newkey >= 0x59 && newkey <= 0x62)  // 89 is 1-key, 98 is 0-key
                    PO0 = newkey - 0x58;    // show it in the LEDS (0 shows as 0x0A)
                break;
             default:        // any other completion code is an error   
                PO0=0xFF;   // all bar LEDS on
                while(1);   // hang here and examine error code
        } // switch(HR)
    } // while(1)
}


// Purpose: To remain in a loop until the FRAMEIRQ decrements a global variable to zero.
void waitframes(int num)   
{
    FRAMECOUNT = 0;
    while(FRAMECOUNT <= num)
    {
        wreg(rHIEN,bmFRAMEIE);
        while( FLAG != bmFRAMEIRQ );
        FRAMECOUNT++;
        FLAG = 0;
    }
}



BYTE Wait_for_HIRQ(BYTE regbit)
{
    static BYTE result;
    wreg(rHIEN,regbit);         // enable only one IRQ in the HIEN register
    while( FLAG != regbit );     // hang until an irq 
    wreg(rHIEN,0x00);           // all ints off
    result = (rreg(rHRSL) & 0x0F);       // get the completion code
    FLAG = 0;
    return result;
/*
    static BYTE result;
    wreg(rHIEN,regbit);         // enable only one IRQ in the HIEN register
    result = regbit;
    while( FLAG <= 0 );     // hang until an irq 
    FLAG = 0;
//    wreg(rHIRQ,regbit);         // clear the irq
    wreg(rHIEN,0x00);           // all ints off
    result = rreg(rHRSL);       // get the completion code
    return (result & 0x0F);     // 4 LSB only
*/
}



//periph 15.1.3
// **** CONTROL-Write transfer with no OUT data stage. ****
//  0. Before calling this function, load the SUDFIFO with the 8 setup bytes, and PERADDR with the address.
//  1. This function sends a SETUP token to fnaddr, EP=0, then a DATA0 PID with the 8 setup bytes in SUDFIFO.
//      If the transfer error code is nonzero, the function exits with the return value of 1x, where x
//      is the RT53 error code. 
//  2. If no error, sends an IN handshake to fnaddr, EP=0. By setting the IN and HS bits, the SIE 
//      automatically checks for the correct peripheral response--an empty DATA1 packet--and ACKS it.
//      If the transfer error code is nonzero, the function exits with the return value of 2x, where x
//      is the RT53 error code.  
//
BYTE CTL_WRITE(void)    // call with SUDFIDO loaded with 8 bytes and PERADDR=device address
{
  //  static BYTE errorcode;
    // Phase 1. Send the SETUP token and 8 setup bytes. Device should immediately ACK.
    wreg(rHIEN,bmHXFRDNIRQ);         // enable only one IRQ in the HIEN register
    wreg(rHXFR,tokSETUP);          
    //errorcode = check_HRSL();                   // check_HRSL() waits for any interrupt and returns host result code
//tt    errorcode = Wait_for_HIRQ(bmHXFRDNIRQ);
    while( FLAG != bmHXFRDNIRQ );     // hang until an irq 
    wreg(rHIEN,0x00);           // all ints off
    errorcode = (rreg(rHRSL) & 0x0F);       // get the completion code
    FLAG = 0;



    if (errorcode)
        return 0x10 + errorcode;    // it's nonzero. The 0x10 indicates first xfr phase


    // Phase 2. Send IN handshakes until we get the DATA1 PID (expect NAKS)
    do  
    {
        wreg(rHXFR,bmHS);                  // [*** was A0, now 80h]
        //        errorcode = check_HRSL();
        errorcode = Wait_for_HIRQ(bmHXFRDNIRQ);
    }
    while (errorcode == hrNAK);             // Try again if a NAK
    
    if (errorcode)
        return 0x20 + errorcode;     // 0x20 indicates second xfr phase
    
    return(0);                                  // success
}



//periph 15.1.2
// **** CONTROL-Read transfer
// Before calling, load SUDFIFO with the command request.
//  The function sends the SETUP packet, then an IN packet, then reads the IN data
//  into the RCVFIFO. 
//  Returns 0x00 on success, or 0xXN where X is 0x80, 0x90, etc. & n is the 4-bit HRSL error code. 
// On exit, the RCVBC reg indicates the byte count, and the IN data is in the RCVFIFO.
//
//
BYTE CTL_READ(void)
{
    static BYTE dum;//, errorcode;
    //BYTE dum,errorcode,xlen;
    // 0. Load the SUDFIFO
    //writebytes(rSUDFIFO,8,pSUD);

    // 1. Send a SETUP token to addr 0, ep0, followed by a DATA0 PID and the 8 setup bytes in SUDFIFO
    // Note that we don't need to initialize the DATA toggle value, the SIE always sends DATA0 for SETUPS.
    //
    wreg(rHIEN,bmHXFRDNIE);         // enable the host transfer done interrupt
    wreg(rHXFR,bmSETUP);            // trigger the transfer--SIE takes care of both SETUP and DATA stages. EP=0.
    errorcode = check_HRSL();       // check_HRSL() waits for HXFRDN IRQ and returns host result code
    if (errorcode)
        return (0x80+errorcode);   

    // 2. Send an IN token to addr0, ep0.
    dum = rreg(rHIRQ);              // 0x68 FRAME, CONN, SNDBAV
    SETBIT(rHCTL,bmRCVTOG1);        // expect a DATA1 packet
    wreg(rHXFR,0x00);               // IN token to fnaddr, EP0
    errorcode = check_HRSL();       // check_HRSL() waits for any interrupt and returns host result code
    if (errorcode)
        return (0x90+errorcode);

    // 3. Get the byte count, read EP0FIFO into PeriphBytes array
    //dum = rreg(rHIRQ);              // 0x6C +++ FRAME CONN SNDBAV RCVDAV
    //xlen = rreg(rRCVBC);            // get the byte count ??? ERROR xlen=0, not 8 
    //readbytes(rRCVFIFO,xlen ,PeriphBytes);
    //SETBIT(rHIRQ,bmRCVDAVIRQ);      // clear the IRQ (and rRCVBC register)

    // 4. 
    wreg(rHXFR,(bmOUTNIN | bmHS));   // OUT packet as a handshake
    errorcode = check_HRSL();       // check_HRSL() waits for any interrupt and returns host result code
    if (errorcode)
        return (0xA0+errorcode);    
    //
    return (0);                     // no errors
}



#pragma vector = 1 // Module 1 Vector
__interrupt void interrupt_service_routine()
{
        // Has INT12 been triggered?
        if ( EIF1_bit.IE12 ) // IE12 EIF1.4
        {  
            // Loop while a valid interrupts exists
            if( (INTERRUPT = (rreg( rHIRQ ) & 0xF7)) != 0x00 ) 
//            if( (INTERRUPT = (rreg( rHIRQ ) )) != 0x00 ) 
            {
                // Check which host interrupt
                if ( INTERRUPT & bmHXFRDNIRQ )    // Host Transfer Done
                {
                    FLAG = bmHXFRDNIRQ;
                    wreg( rHIRQ, bmHXFRDNIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmBUSEVENTIRQ )  // [bus reset + 50mSec] or [bus resume + 20mSec]
                {
                    FLAG = bmBUSEVENTIRQ;
                    wreg( rHIRQ,  bmBUSEVENTIRQ);    // clear this irq
                }
                else if ( INTERRUPT & bmFRAMEIRQ )  // begin SOF Packet
                {
                 //   FRAMECOUNT--;
            //        *p_int--;
                    FLAG = bmFRAMEIRQ;
                    wreg( rHIRQ, bmFRAMEIRQ ); // clear the IRQ
                }
                else if ( INTERRUPT & bmRSMREQDETIRQ )   // Bus Resume Request Detected
                {
                    FLAG = bmRSMREQDETIRQ;
                    wreg( rHIRQ, bmRSMREQDETIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmRCVDAVIRQ )   // Receiver FIFO contains Data
                {
                    FLAG = bmRCVDAVIRQ;
                    wreg( rHIRQ, bmRCVDAVIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmSUSPENDDNIRQ )   // Suspend generation Done
                {
                    FLAG = bmSUSPENDDNIRQ;
                    wreg( rHIRQ, bmSUSPENDDNIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmCONNIRQ )  // State of Connection Changed
                {
                    FLAG = bmCONNIRQ;
                    switch ( rreg(rHRSL) & bmKSTATUS & bmJSTATUS )
                    {
                        case 0x00 : // SE0 - EOP or disconnect
                        break;
                        case bmKSTATUS :
                        break;
                        case bmJSTATUS :
                        break;
                        default :
                        // error
                        break;
                    }
                    wreg( rHIRQ, bmCONNIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmSNDBAVIRQ )   // Send Buffer Available
                {
                  //  static BYTE dummy;
                    FLAG = -8;
                    wreg( rHIRQ, bmSNDBAVIRQ );    // clear this irq
                  // dummy = rreg(rHIRQ);
                  //  EIF1_bit.IE12 = 0;
                }
                else if ( INTERRUPT == 0x00 )  // No valid interrupt
                {
                    FLAG = -1;
                    // Reset the uProc interrupt
                    EIF1_bit.IE12 = 0;
                    // Exit the ISR
                     return;
                }
          } // if(INTERRUPT)
          EIF1_bit.IE12 = 0;
    } // end if( EIF1_bit.IE12 )
} // end ISR

#endif
