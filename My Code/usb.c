#ifndef USB_C
#define USB_C

#include <intrinsics.h> // MAXQ2000 specific stuff
#include <iomaxq200x.h> // ditto
#include "..\usb.h"
#include "..\interface.c"
#include "RT53reg.h"

static DEVICE *device = NULL;


// Mode register macros
#define HOST_LS_THRUHUB wreg(rMODE,0xCF);   // DPPD,DMPD,KAENAB,HUBRPE,LOWSPEED,HOST
#define HOST_LS         wreg(rMODE,0xCB);   // as above but HUBPRE=0
#define HOST_FS         wreg(rMODE,0xC9);   // as HOST_LS but LOWSPEED=0
//
#define SUSPEND_INT_ON wreg(rUSBIEN,(rreg(rUSBIEN) | bmSUSPEND));
#define SUSPEND_INT_OFF wreg(rUSBIEN,(rreg(rUSBIEN) & !bmSUSPEND));
#define BUSACT_INT_ON wreg(rUSBIEN,(rreg(rUSBIEN) | bmBUSACT));
#define BUSACT_INT_OFF wreg(rUSBIEN,(rreg(rUSBIEN) & !bmBUSACT));
#define SETBIT(reg,val) wreg(reg,(rreg(reg)|val));
#define CLRBIT(reg,val) wreg(reg,(rreg(reg)&~val));
//
#define STALL_EP0 wreg(9,0x23);	// Set all three EP0 stall bits--data stage IN/OUT and status stage



void Host_Init( void )
{
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
    wreg(rPINCTL,(bmFDUPSPI)); 

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

}



/*
int Connect_Device( DEVICE * my_device )
{
    static int status;
   
    if ( CURRENT != NEXT )
    {
        CURRENT = NEXT;
        switch ( CURRENT )
        {
            case 0x00 : // SE0 - EOP or disconnect
                if ( my_device != NULL ) 
                Close_Device( my_device );
//                free(my_device);
 //               my_device = NULL;
//                status = 0;
                break;
            case bmKSTATUS :    // K STATE
                ///////////////////////////////////////////////////
                my_device = Open_Device();
                if ( my_device == NULL ) 
                {
                    printf( "Failed to create the device!\n" );
                    exit( 1 );
                }
    //            this_device = my_device;
//                Configure_Device( my_device, addr4, conf1, idle );
                ///////////////////////////////////////////////////
//                status = 1;
                break;
            case bmJSTATUS :    // J STATE
   // PO0 = 0x0C;       // MAXQ: 
                ///////////////////////////////////////////////////

                my_device = Open_Device();
                if ( my_device == NULL ) 
                {
                    printf( "Failed to create the device!\n" );
                    exit( 1 );
                }
//                Configure_Device( my_device, addr4, conf1, idle );
          //      this_device = my_device;
                ///////////////////////////////////////////////////
                status = 2;
                break;
            default :           // error
                break;
        } // end switch ( CURRENT )
        wreg(rHIEN,bmCONNIRQ);
    } // end if( CURRENT != NEXT )
        
     // add a test to determine...   
    // if(my_device == hub) status = 3;

    return status;
        

}
*/

// PURPOSE: This function creates a DEVICE object to store information related to
// the device currently at address 0 on the USB bus.
// PRECONDITIONS: This device was just plugged into the USB bus.
// POSTCONDITIONS: This device will be in DEFAULT mode, the Device and Interface
// descriptors will be store in the fields of the DEVICE object "my_device."
// This function will initialize the global "device" pointer to be used by the ISR
// while this device is at address zero.
// ARGUMENTS: None.
// RETURNED: Pointer to the device object.
DEVICE *Open_Device( void )
{
    DEVICE *my_device;
    my_device = malloc( sizeof( DEVICE ));
    if ( my_device == NULL) 
        return( NULL );
    device = my_device;


timeDELAY(5000);


    // Reset the USB device to default state
    Bus_Reset();

     
//    waitframes(8);

//    Bus_Reset();

     //
    waitframes(10);


//Standard_Request(SR_GET_DESCRIPTOR, GD_DEVICE, device);
//Standard_Request(SR_GET_DESCRIPTOR, GD_CONFIGURATION, device);
//Standard_Request(SR_GET_DESCRIPTOR, GD_INTERFACE, device);


    return( my_device );
}


// PURPOSE: 
// PRECONDITIONS: The USB device is in the default state and the host is fully
// configured. The descriptors have been read from the USB device and stored in the 
// appropriate fields in the my_device object.
// POSTCONDITIONS: The USB device has been assigned a unique address. Based upon the
// configuration data read in the Open_Device() function and stored in my_device, the
// device has been configured. 
// ARGUMENTS:
// RETURNS:
void Configure_Device( DEVICE *my_device,
               const SUD address,
               const SUD config,
               const SUD idle)
{
    static BYTE Set_Addr4[8]   = {0x00,0x05,0x04,0x00,0x00,0x00,0x00,0x00};
    static BYTE Set_Config1[8] = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
    static BYTE Set_Idle[8]    = {0x21,0x0A,0x00,0x00,0x00,0x00,0x00,0x00};
    //my_device->address = address;
    //my_device->config = config;
    //my_device->idle = idle;

timeDELAY(5000);

    // Set_Address to 4
    wreg(rPERADDR,0x00);                // send to address 0
    waitframes(10);
    WriteBytes(rSUDFIFO,8,Set_Addr4);   // Load the SUDFIFO with Set_Address=4 request
    HR1 = CTL_WRITE();                  // this function returns only after completion 

    // Set_Config to 1
    wreg(rPERADDR,0x04);                // address is now 4
    WriteBytes(rSUDFIFO,8,Set_Config1); // Load the SUDFIFO with Set_Config=1 request
    HR2 = CTL_WRITE();

    // Send the "set_idle" request
    WriteBytes(rSUDFIFO,8,Set_Idle);    // Load the SUDFIFO with Set_Idle request
    HR3 = CTL_WRITE();

// Check here to see if the device is a hub; if so, configure the hub and return 1
timeDELAY(100);
}


// Host
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
void Configure_Hub( DEVICE *my_device,
               const SUD address,
               const SUD config,
               const SUD idle)
{

    //
    waitframes(4);

    // Set_Address to 4
    wreg(rPERADDR,0x00);                // send to address 0
    waitframes(10);
    WriteBytes(rSUDFIFO,8,address);   // Load the SUDFIFO with Set_Address=4 request
    HR1 = CTL_WRITE();                  // this function returns only after completion 

    // Set_Config to 1
    wreg(rPERADDR,0x04);                // address is now 4
    WriteBytes(rSUDFIFO,8,config); // Load the SUDFIFO with Set_Config=1 request
    HR2 = CTL_WRITE();

    // Send the "set_idle" request
    WriteBytes(rSUDFIFO,8,idle);    // Load the SUDFIFO with Set_Idle request
    HR3 = CTL_WRITE();

// Check here to see if the device is a hub; if so, configure the hub and return 1

}

// Host
// PURPOSE: This function destroys the DEVICE object and frees the memory used by 
// a device that was unplugged from the USB bus.
// PRECONDITIONS: A DEVICE object existed containing the data for a device.
// POSTCONDITION: The memory allocated for the device is freed and the host is 
// reconfigured to pre-connect state and prepared for another device to be connected.
// ARGUMENTS: Apointer to the the DEVICE object of the disconnected device.
// RETURNS: None.
void Close_Device( DEVICE *my_device )
{
    DEVICE *Tmp;
    Tmp = my_device;
    if ( my_device != NULL ) 
        my_device = NULL;
    free(Tmp);
}

// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
void Bus_Reset(void)
{
    static BYTE temp;
    temp = rreg( rHIEN );
    wreg( rHIEN, bmBUSEVENTIE | temp );           // enable the reset done irq
    wreg( rHCTL, bmBUSRST );               // initiate the 50 msec bus reset
    Wait_for_HIRQ( bmBUSEVENTIRQ );       // wait for, and then clear this interrupt
    wreg( rHIEN, temp );                   // restore ints
}

// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
void Bus_Resume(void)
{
    static BYTE temp;
    temp = rreg( rHIEN );
    wreg( rHIEN, bmBUSEVENTIE | temp );           // enable the reset done irq
    wreg(rHCTL,bmBUSRSM);               // initiate the 20 msec bus resume
    Wait_for_HIRQ(bmBUSEVENTIRQ);       // wait for, and then clear this interrupt
    wreg( rHIEN, temp );                   // restore ints
}


// PURPOSE: This resets the MAX3420E/MAX3421E
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
void Reset_MAX(int time)   // applies to MAX3420E or MAX3421E
{
    wreg( rUSBCTL, 0x20 );	// chip reset
    timeDELAY( 500 );		// a delay
    wreg( rUSBCTL, 0x00 );	// remove the reset
    timeDELAY( 2000 );		// a delay
}

// Host
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
DEVICE *Standard_Request(BYTE request_code, BYTE descriptor_type, DEVICE * my_device)
{

    switch ( request_code )
    {
        case SR_GET_STATUS:
           // Get_Status(my_device);
            break;
        case SR_CLEAR_FEATURE:
           // Clear_Feature(my_device);
            break;
        case SR_RESERVED:
            break;
        case SR_SET_FEATURE:
           // Set_Feature(my_device);
            break;
        case SR_SET_ADDRESS:
           // Set_Address(my_device);
            break;
        case SR_GET_DESCRIPTOR:
           // Get_Descriptor(descriptor_type, my_device);
            break;
        case SR_SET_DESCRIPTOR:
           // Set_Descriptor(descriptor_type, my_device);
            break;
        case SR_GET_CONFIGURATION:
           // Get_Configuration(my_device);
            break;
        case SR_SET_CONFIGURATION:
           // Set_Configuration(my_device);
            break;
        case SR_GET_INTERFACE:
           // Get_Interface(my_device);
            break;
        case SR_SET_INTERFACE:
           // Set_Interface(my_device);
            break;
        default:
            return NULL;
    }

    return my_device;
}


// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
void Get_Configuration(DEVICE *my_device)
{

}

// Host, Address/Configured
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
void Set_Configuration(DEVICE *my_device)
{

}


// Host, Default/Address
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
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
    wreg(rPERADDR,0x00);    // send to address 0
    return CTL_WRITE();      // return its error code
}


// Host, Address/Configured
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
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


// Host
// PURPOSE: 
//      HUB: get hub/port status.  If port=0, returns hub status.
//      PeriphBytes[0]   = StatusL
//      PeriphBytes      = StatusH
//      PeriphBytes      = PortChangeL. NOTE: PortChangeH is all zeros so we skip it.
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS: Result code for the CTL-RD transfer (0=no errors)
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


// Host
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
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


// Host
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
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


// Host, Address/Configured
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
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


// Host, Configured
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
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


// Host, Default/Address/Configured
// Get_Descriptor_Device[8] = {0x80,0x06,0x00,0x01,0x00,0x00,0x40,0x00};
// PURPOSE: 
// PRECONDITIONS:
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
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

//Host
// PURPOSE: // **** CONTROL-Write transfer with no OUT data stage. **** //
//  1. This function sends a SETUP token to fnaddr, EP=0, then a DATA0 PID with the 8 setup bytes in SUDFIFO.
//      If the transfer error code is nonzero, the function exits with the return value of 1x, where x
//      is the RT53 error code. 
//  2. If no error, sends an IN handshake to fnaddr, EP=0. By setting the IN and HS bits, the SIE 
//      automatically checks for the correct peripheral response--an empty DATA1 packet--and ACKS it.
//      If the transfer error code is nonzero, the function exits with the return value of 2x, where x
//      is the RT53 error code.  
 
// PRECONDITIONS: SUDFIFO pre-loaded with the 8 setup bytes, and PERADDR with the address.
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS: HSRLT, 0 = Success
BYTE CTL_WRITE(void)    // call with SUDFIDO loaded with 8 bytes and PERADDR=device address
{
    static BYTE errorcode;

    // Phase 1. Send the SETUP token and 8 setup bytes. Device should immediately ACK.
    do
    {
        wreg(rHXFR,tokSETUP);          
        errorcode = Wait_for_HIRQ(bmHXFRDNIRQ);
    } 
    while(errorcode);


    //    if (errorcode)  
    //        return 0x10 + errorcode;    // it's nonzero. The 0x10 indicates first xfr phase

    // Phase 2. Send IN handshakes until we get the DATA1 PID (expect NAKS)
    do  
    {
        wreg(rHXFR,bmHS);                  // [*** was A0, now 80h]
        errorcode = Wait_for_HIRQ(bmHXFRDNIRQ);
    }
    while (errorcode/* == hrNAK*/);             // Try again if a NAK

    //    if (errorcode) 
    //        return 0x20 + errorcode;     // 0x20 indicates second xfr phase

    return(0);                                  // success
}

//Host
// PURPOSE: // **** CONTROL-Read transfer **** //
//  The function sends the SETUP packet, then an IN packet, then reads the IN data
//  into the RCVFIFO. 
// PRECONDITIONS: Before calling, load SUDFIFO with the command request.
// POSTCONDITIONS: On exit, the RCVBC reg indicates the byte count, and the IN data is in the RCVFIFO.
// ARGUMENTS: None.
// RETURNS: Returns 0x00 on success, or 0xXN where X is 0x80, 0x90, etc. & n is the 4-bit HRSL error code. 
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

// Host
// PURPOSE: To remain in a loop until the FRAMEIRQ decrements a global variable to zero.
// PRECONDITIONS: Host mode, SPI configured.
// POSTCONDITIONS: Interrupts are returned to original state.
// ARGUMENTS: int number of USB frames delay desired
// RETURNS: None.
void waitframes(int num)   
{
static BYTE temp;
    // Store currently enabled IRQs
    temp = rreg(rHIEN);
    FRAMECOUNT = 0;
    while(FRAMECOUNT < num)
    {
        wreg(rHIEN, (bmFRAMEIE | bmCONNIRQ));
        while( !(FLAG & bmFRAMEIRQ) && !(FLAG & bmCONNIRQ) );
        FRAMECOUNT++;
        // FLAG = 0;
    }
    FRAMECOUNT = 0;
    wreg(rHIEN,temp);
}


// Host
// PURPOSE: PAuse for an expected Host IRQ to occur. This pause can be interrupted by a change in
// device connect status.
// PRECONDITIONS: Host mode, SPI configured.
// POSTCONDITIONS: Interrupts are returned to original state.
// ARGUMENTS: BYTE representing the HIRQ bit mask for the expected Host interrupt
// RETURNS: BYTE HSRLT
BYTE Wait_for_HIRQ(BYTE regbit)
{
    static BYTE result, temp1, temp2;
    temp1 = regbit;
    temp2 = rreg(rHIEN);
    wreg(rHIEN,temp1 | (BYTE)bmCONNIRQ);              // enable only one IRQ in the HIEN register

    while( !(FLAG & temp1) && !(FLAG & bmCONNIRQ) );         // hang until an irq 
    result = (rreg(rHRSL) & 0x0F);   // get the completion code
  //  FLAG = 0;
    wreg(rHIEN,temp2);                // restore ints
    return result;
}


// Host
// PURPOSE: This ISR services the Host interrupts triggered in the HIRQ register.
// PRECONDITIONS: Host mode, SPI configured, interrupts enabled.
// POSTCONDITIONS:
// ARGUMENTS:
// RETURNS:
#pragma vector = 1 // MAXQ2000 Module 1 Vector
__interrupt void interrupt_service_routine()
{
    // Has MAXQ2000 INT12 line been triggered?
    if ( EIF1_bit.IE12 ) // IE12 EIF1.4
    {  
        // Does a valid enabled host interrupt exist?
        if( INTERRUPT = (rreg( rHIRQ ) & rreg( rHIEN )) ) 
        {
            // Which host interrupt?
 
            // CONNECT IRQ: USB State of Connection Changed
            if ( INTERRUPT & bmCONNIRQ )  
            {
                FLAG = bmCONNIRQ;
                wreg(rHCTL,0x04); // Sample the USB bus
                // Set the NEXT connection state.
                switch ( rreg(rHRSL) & 0xC0 )
                {
                    case 0x00 :         // SE0 - EOP or disconnect
                        NEXT = 0x00;
                        break;
                    case bmKSTATUS :    // K STATE
                        NEXT = bmKSTATUS;
                        break;
                    case bmJSTATUS :    // J STATE
                        NEXT = bmJSTATUS;
                        break;
                    default :           // error
                        NEXT = 3;
                        break;
                }
                wreg( rHIRQ, bmCONNIRQ );    // clear this irq
            }
            // HXFRDN IRQ: Host Transfer Done
            else if ( INTERRUPT & bmHXFRDNIRQ )
            {
                FLAG = bmHXFRDNIRQ;
                wreg( rHIRQ, bmHXFRDNIRQ );    // clear this irq
            }
            // BUSEVENT IRQ: [Bus Reset + 50mSec] <or> [Bus Resume + 20mSec]
            else if ( INTERRUPT & bmBUSEVENTIRQ )  
            {
                FLAG = bmBUSEVENTIRQ;
                wreg( rHIRQ,  bmBUSEVENTIRQ);    // clear this irq
            }
            // RSMREQDET IRQ: Bus Resume Request Detected
            else if ( INTERRUPT & bmRSMREQDETIRQ )   
            {
                FLAG = bmRSMREQDETIRQ;
                wreg( rHIRQ, bmRSMREQDETIRQ );    // clear this irq
            }
            // RCVDAV IRQ: Receiver FIFO Contains Data
            else if ( INTERRUPT & bmRCVDAVIRQ )   
            {
                FLAG = bmRCVDAVIRQ;
                wreg( rHIRQ, bmRCVDAVIRQ );    // clear this irq
            }
            // SNDBAV IRQ: Send Buffer Available
            else if ( INTERRUPT & bmSNDBAVIRQ )   
            {
                FLAG = bmSNDBAVIRQ;
                wreg( rHIRQ, bmSNDBAVIRQ );    // clear this irq
            }
            // SUSPENDDN IRQ: Suspend Generation Done
            else if ( INTERRUPT & bmSUSPENDDNIRQ )   
            {
                FLAG = bmSUSPENDDNIRQ;
                wreg( rHIRQ, bmSUSPENDDNIRQ );    // clear this irq
            }
            // FRAME IRQ: SOF Packet Begin
            else if ( INTERRUPT & bmFRAMEIRQ )  
            {
                FLAG = bmFRAMEIRQ;
                wreg( rHIRQ, bmFRAMEIRQ ); // clear the IRQ
            }
        } // end if(INTERRUPT)
        else 
        {
            // No Valid Enabled Interrupt
            FLAG = -1;
            // Clear MAXQ2000 INT12 Interrupt
            EIF1_bit.IE12 = 0;
            return;
        }
        // Clear MAXQ2000 INT12 Interrupt
        EIF1_bit.IE12 = 0;
    }    // end if( EIF1_bit.IE12 )
} // end ISR

//#endif // HOST


#endif
