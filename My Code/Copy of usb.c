#ifndef USB_C
#define USB_C

#include <intrinsics.h> // MAXQ2000 specific stuff
#include <iomaxq200x.h> // ditto
#include "..\usb.h"
#include "..\interface.c"
#include "RT53reg.h"




void Bus_Reset(void)
{
    wreg(rHIEN,bmBUSEVENTIE);           // enable the reset done irq
    wreg(rHCTL,bmBUSRST);               // initiate the 50 msec bus reset
    Wait_for_HIRQ(bmBUSEVENTIRQ);       // wait for, and then clear this interrupt
    wreg(rHIEN,0x00);                   // all ints disabled
}

//
void Bus_Resume(void)
{
    wreg(rHIEN,bmBUSEVENTIE);           // enable the reset done irq
    wreg(rHCTL,bmBUSRSM);               // initiate the 20 msec bus resume
    Wait_for_HIRQ(bmBUSEVENTIRQ);       // wait for, and then clear this interrupt
    wreg(rHIEN,0x00);                   // all ints disabled
}

void Reset_MAX(int time)   // applies to MAX3420E or MAX3421E
{
//    int k;
    wreg(rUSBCTL,0x20);			// chip reset
    timeDELAY(500);		// a delay
    wreg(rUSBCTL,0x00);			// remove the reset
    timeDELAY(2000);		// a delay
}


//periph
BYTE Set_Address(BYTE addr)     
{
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x05);
    wreg(rSUDFIFO,addr);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    //
    wreg(rPERADDR,0x00);    // send to address 0
    return CTL_WRITE();      // return its error code
}


//
// DEVICE: Get_Status
//
BYTE Get_Device_Status(void)
{
    wreg(rSUDFIFO,0x80);    // 10000000 OUT, STD REQ, to Device
    wreg(rSUDFIFO,0x00);    // 0 is Get_Status
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x02);    // ask for two bytes
    wreg(rSUDFIFO,0x00);
    return CTL_READ();
}


//
// HUB: get hub/port status.  If port=0, returns hub status.
// PeriphBytes[0]   = StatusL
// PeriphBytes      = StatusH
// PeriphBytes      = PortChangeL. NOTE: PortChangeH is all zeros so we skip it.
//
// Returns the result code for the CTL-RD transfer (0=no errors)    
BYTE H_GetStat(BYTE port)                        
{
    wreg(rSUDFIFO,0xA3);    // 10100011 IN, class, "other"
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,port);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x04);
    wreg(rSUDFIFO,0x00);
    return CTL_READ();
}



// HUB: Set/Clear port Feature
BYTE HubPort_Feature(BYTE setnotclr,BYTE feat,BYTE port)
{
    wreg(rSUDFIFO,0x23);
    wreg(rSUDFIFO, setnotclr ? 0x03:0x01);  // Set_Feature or Clear_Feature
    wreg(rSUDFIFO,feat);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,port);   // wIndexL
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    wreg(rSUDFIFO,0x00);
    return CTL_WRITE();
}



// Set_Idle[8]    = {0x21,0x0A,0x00,0x00,0x00,0x00,0x00,0x00};
BYTE Set_Idle(BYTE iface, BYTE duration, BYTE reportID)
{
    wreg(rSUDFIFO,0x21);    // bmRequestType=Output, class request, directed to interface
    wreg(rSUDFIFO,0x0A);    // SET_IDLE
    wreg(rSUDFIFO,reportID);// wValueL
    wreg(rSUDFIFO,duration);// wValueH
    wreg(rSUDFIFO,0x00);    // wIndexL
    wreg(rSUDFIFO,iface);   // wIndexH
    wreg(rSUDFIFO,0x0);     // wLengthL
    wreg(rSUDFIFO,0x0);     // wLengthH
    return CTL_WRITE();
}


//
BYTE Set_Config(BYTE cfgval)
{
    wreg(rSUDFIFO,0x00);    // bmRequestType=Output, std request, directed to device
    wreg(rSUDFIFO,0x09);    // SET_CONFIG
    wreg(rSUDFIFO,cfgval);  // wValueL
    wreg(rSUDFIFO,0x00);    // wValueH
    wreg(rSUDFIFO,0x00);    // wIndexL
    wreg(rSUDFIFO,0x00);    // wIndexH
    wreg(rSUDFIFO,0x0);     // wLengthL
    wreg(rSUDFIFO,0x0);     // wLengthH
    return CTL_WRITE();
}




//
BYTE Set_Interface(BYTE interface, BYTE altsetting)
{
    wreg(rSUDFIFO,0x00);        // bmRequestType=Output, std request, directed to device
    wreg(rSUDFIFO,0x0B);        // SET_CONFIG
    wreg(rSUDFIFO,altsetting);  // wValueL=alternate setting
    wreg(rSUDFIFO,0x00);        // wValueH=0
    wreg(rSUDFIFO,interface);   // wIndexL=Interface number
    wreg(rSUDFIFO,0x00);        // wIndexH
    wreg(rSUDFIFO,0x0);         // wLengthL
    wreg(rSUDFIFO,0x0);         // wLengthH
    return CTL_WRITE();
}



// Get_Descriptor_Device[8] = {0x80,0x06,0x00,0x01,0x00,0x00,0x40,0x00};
BYTE Get_Descriptor(BYTE type,WORD length)
{
    wreg(rSUDFIFO,0x80);    // bmRequestType=Input, Std request, directed to device
    wreg(rSUDFIFO,0x06);    // GET_DESCRIPTOR
    wreg(rSUDFIFO,0x00);    // wValueL = descriptor index
    wreg(rSUDFIFO,type);    // wValueH = descriptor type
    wreg(rSUDFIFO,0x00);    // wIndexL
    wreg(rSUDFIFO,0x00);    // wIndexH
    wreg(rSUDFIFO,MSB(length)); // wLengthL
    wreg(rSUDFIFO,LSB(length));  // wLengthH
    return CTL_READ();
}


#ifdef HST

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
    static BYTE errorcode;
    // Phase 1. Send the SETUP token and 8 setup bytes. Device should immediately ACK.
    wreg(rHXFR,tokSETUP);          
    //errorcode = check_HRSL();                   // check_HRSL() waits for any interrupt and returns host result code
//tt    errorcode = Wait_for_HIRQ(bmHXFRDNIRQ);
//tt    if (errorcode)
//tt        return 0x10 + errorcode;    // it's nonzero. The 0x10 indicates first xfr phase


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
    static BYTE dum, errorcode;
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


/*
//
BYTE Wait_for_HIRQ(BYTE regbit)
{
    static BYTE result;
    wreg(rHIEN,regbit);         // enable only one IRQ in the HIEN register
    result = regbit;
    while( FLAG <= 0 );     // hang until an irq 
    FLAG = 0;
//    wreg(rHIRQ,regbit);         // clear the irq
    wreg(rHIEN,0x00);           // all ints off
    result = rreg(rHRSL);       // get the completion code
    return (result & 0x0F);     // 4 LSB only
}
*/

/*
BYTE check_HRSL(void)   // wait for the HXFRDN IRQ, then read and return the result   
{
    BYTE done;
    done = 0;
    wreg(rHIEN,bmHXFRDNIE);              // enable only this interrupt
    while(!done)
    {
        while( FLAG == 0 ) ;         // hang until an irq 
        if(rreg(rHIRQ) & bmHXFRDNIRQ)   // is it the HXFRDN IRQ?
        {
            wreg(rHIRQ,bmHXFRDNIRQ);    // clear the irq
            done = 1;
        }
    }
    wreg(rHIEN,0x00);                   // disable all interrupts
    return (rreg(rHRSL) & 0x0F);        // 4 LSB of completion code
}
*/


// Purpose: To remain in a loop until the FRAMEIRQ decrements a global variable to zero.
void waitframes(int num)   
{
//    BYTE temp;
////    FLAG = 0;
    //FRAMECOUNT = num;
//    temp = rreg( rHIEN ); // retrieve currently active interrupts
    wreg( rHIEN, bmFRAMEIE );      // enable all interrupt
 //   timeDELAY(500);		// a delay
//    while( FRAMECOUNT >= 0 );               // loop until zero
    wreg( rHIEN, 0x00 );           // disable all interrupts
}

/*

#pragma vector = 1 // Module 1 Vector
__interrupt void interrupt_service_routine()
{
        // Has INT12 been triggered?
        if ( EIF1_bit.IE12 ) // IE12 EIF1.4
        {  
            // Loop while a valid interrupts exists
            if( INTERRUPT = (rreg( rHIRQ ) & 0xF7) ) 
            {
                // Check which host interrupt
                if ( INTERRUPT & bmBUSEVENTIRQ )  // [bus reset + 50mSec] or [bus resume + 20mSec]
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
                else if ( INTERRUPT & bmFRAMEIRQ )  // begin SOF Packet
                {
                    FRAMECOUNT--;
                    FLAG = 64;
                    wreg( rHIRQ, bmFRAMEIRQ ); // clear the IRQ
                }
                else if ( INTERRUPT & bmHXFRDNIRQ )    // Host Transfer Done
                {
                    FLAG = bmHXFRDNIRQ;
                    wreg( rHIRQ, bmHXFRDNIRQ );    // clear this irq
                }
                else if ( INTERRUPT & bmSNDBAVIRQ )   // Send Buffer Available
                {
                  //  static BYTE dummy;
                    //FLAG = 8;
                    wreg( rHIRQ, bmSNDBAVIRQ );    // clear this irq
                  // dummy = rreg(rHIRQ);
                  //  EIF1_bit.IE12 = 0;
                }
                else if ( 0x00 )  // No valid interrupt
                {
                    FLAG = -1;
                    // Reset the uProc interrupt
                    //EIF1_bit.IE12 = 0;
                    // Exit the ISR
                    // return;
                }
          } // if(INTERRUPT)
          EIF1_bit.IE12 = 0;
    } // end if( EIF1_bit.IE12 )
} // end ISR
*/
#endif // HOST






#endif
