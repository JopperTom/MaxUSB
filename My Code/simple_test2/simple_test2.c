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
int NEXT;
BYTE SUD[8];			// my copy of setup data
// prototypes in this module
void waitframes(int num);   
BYTE Wait_for_HIRQ(BYTE regbit);
void Read_Keypad(void);
void prep_keypad(void);
//********************
///////////////////////////////////////////////////

main()
{
///// Declare Variables Here ///////////////////////
 //   static BYTE Set_Addr4[8]   = {0x00,0x05,0x04,0x00,0x00,0x00,0x00,0x00};
//    static BYTE Set_Config1[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
//    static BYTE Set_Idle[8]    = {0x21,0x0A,0x00,0x00,0x00,0x00,0x00,0x00};
//    static BYTE HR1,HR2,HR3;
    static BYTE dum, result;
///////////////////////////////////////////////////

    // MAXQ2000 SPI port as a master
    SPI_Init();                 

    // MUST GO BEFORE AMY SPI TRAFFIC
    // RT53: INTLEVEL=0, POSINT=1 for pos edge interrupt pin
    wreg(rPINCTL,(bmFDUPSPI|bmPOSINT)); 

    
    // disable interrupt pin
//    wreg(rCPUCTL, 0x00);			        

   
   // disable all interrupt
    wreg(rUSBIEN,0x00);
    wreg(rGPINIEN,0x00);
    wreg( rHIEN, 0x00 );      
    wreg(rCPUCTL, 0x00);			        

   
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
    PD0=0xFF;                    // MAXQ: Set port 0 to outputs
    PO0 = rreg(rRevision);       // MAXQ: 
    PD7 = 0x01;                  // MAXQ: P70 is scope TRIG signal


   // INTERRUPT RELATED MAXQ2000
    // MAX3410E INT pin is tied to MAXQ2000 P60: make it an input
    PD6 &= ~0x01;                // MAXQ: PD6.0=0 (turn off output)
    EIES1_bit.IT12 = 0;          // MAXQ: 0=pos edge triggered IRQ
    EIE1_bit.EX12 = 1;           // MAXQ: Enable Int12
    IMR  |= 0x02;                // MAXQ: enable interrupts for module 1
    
    
    // Enable interrupt pin
    wreg(rCPUCTL, 0x0F);			        

    wreg(rHIRQ, 0xFF);	// Clear Pending INTs		        

    // MAXQ: enable interrupts
    __enable_interrupt();

///// Your Code Here //////////////////////////////



    dum = Wait_for_HIRQ(bmFRAMEIRQ);


    while(1);



/////////////////////////////////////////////////////
} // End of main(void)






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
    FRAMECOUNT = 0;
}



BYTE Wait_for_HIRQ(BYTE regbit)
{
    static BYTE result;
    wreg(rHIEN,regbit);              // enable only one IRQ in the HIEN register
    while( FLAG != regbit );         // hang until an irq 
    result = (rreg(rHRSL) & 0x0F);   // get the completion code
    FLAG = 0;
    wreg(rHIEN,0x00);                // all ints off
    return result;
}




#pragma vector = 1 // Module 1 Vector
__interrupt void interrupt_service_routine()
{
//    static BYTE fred;
//    while(1) // Stay in loop while a valid interrupt exists
//    {
        // Has INT12 been triggered?
        if ( EIF1_bit.IE12 ) // IE12 EIF1.4
        {  
            // Loop while a valid interrupts exists
            if( INTERRUPT = (rreg( rHIRQ ) & rreg( rHIEN )) ) 
//            if( (INTERRUPT = (rreg( rHIRQ ) )) != 0x00 ) 
            {
                // Check which host interrupt
                if ( INTERRUPT & bmCONNIRQ )  // State of Connection Changed
                {
                    FLAG = bmCONNIRQ;
                    switch ( rreg(rHRSL) & 0xC0 )
                    {
                        case 0x00 : // SE0 - EOP or disconnect
                            NEXT = 0;
                            break;
                        case bmKSTATUS :    // K STATE
                            NEXT = 1;
                            break;
                        case bmJSTATUS :    // J STATE
                            NEXT = 2;
                            break;
                        default :           // error
                            NEXT = 3;
                            break;
                    }
                    wreg( rHIRQ, bmCONNIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmHXFRDNIRQ )    // Host Transfer Done
                {
                    FLAG = bmHXFRDNIRQ;
                    wreg( rHIRQ, bmHXFRDNIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmBUSEVENTIRQ )  // [bus reset + 50mSec] or [bus resume + 20mSec]
                {
                    FLAG = bmBUSEVENTIRQ;
                    wreg( rHIRQ,  bmBUSEVENTIRQ);    // clear this irq
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
                else if ( INTERRUPT & bmSNDBAVIRQ )   // Send Buffer Available
                {
                    FLAG = bmSNDBAVIRQ;
                    wreg( rHIRQ, bmSNDBAVIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmFRAMEIRQ )  // begin SOF Packet
                {
                    FLAG = bmFRAMEIRQ;
                    wreg( rHIRQ, bmFRAMEIRQ ); // clear the IRQ
                }
            } // end if(INTERRUPT)
            else 
            {
                FLAG = -1;
                EIF1_bit.IE12 = 0;
                return;
            }
        } // end if( EIF1_bit.IE12 )
//    } //end while(1)
} // end ISR


















#endif
