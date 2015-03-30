//
//  File........: connect_test.c
//  Authors.....: Tom Tripp
//  Description.: Auto detect LS Keypad connect/disconnect
//  
//
//  Conditions:
//    IFACE:  SPI
//
//  Date........: 10/05/2005
//
// The proper interface can be found in interface.c
// USB functions are defined in usb.c
// 

#ifndef simple_test_C
#define simple_test_C

// Host or peripheral? - define only one.
#define HST
//#define PER

// Low or Full Speed? - define only one.
//#define FS
#define LS

#include <stdlib.h>
#include <stdio.h>

#include <intrinsics.h>      // MAXQ2000 specific stuff
#include <iomaxq200x.h>      // MAXQ2000 specific stuff
#include "RT53reg.h"         // RT99 Registers and bits
#include "..\interface.c"    // SPI/I2C Implementation
#include "..\usb.c"          // USB Implementation


///// Declare GLOBALS Here ///////////////////////

//int CONNECT = -1;

// This will be an array of configured devices
// to accomodate one hub and one connected device
//  DEVICE *this_device = NULL;
//  this_device = malloc( sizeof( DEVICE ));
//      if ( this_device == NULL) exit( 1 );




// prototypes for functions in this module //

void Read_Keypad(void); 

//****************************************//


///////////////////////////////////////////////////

main()
{
///// Declare Variables Here ///////////////////////

static DEVICE *my_device = NULL;

///////////////////////////////////////////////////

    Host_Init();

///// Your Code Here //////////////////////////////

    wreg(rHCTL,0x04); // Sample the USB bus
    CURRENT = -1; // Must be unique from NEXT to enter state machine.
    NEXT = (rreg(rHRSL) & 0xC0); // USB device connect status

    //    timeDELAY(500);		// a delay
    
    while(1)
    {
        // Wait for a device to connect
        
    // PURPOSE: This function will poll the Connect_Device() function. When a new device
    // is connected, it will determine if it is a Hub, device or device behind a hub and 
    // configure it as needed.
    // PRECONDITIONS:
    // POSTCONDITIONS:
    // ARGUMENTS:
    // RETURNS:
    /*  switch( Connect_Device( this_device ) )
        {
            case 1: // Configure LS Peripheral and drop out to Read_Keypad()
                // HOST_LS
    //           Configure_Device( this_device[x], addr[2] );
                //my_device++;
                break;
            case 2: // Configure FS Peripheral and drop out to Read_Keypad()
                // HOST_FS
    //             Configure_Device( this_device[x], addr[4] );
                //my_device++;
                break;
            case 3: // Configure Hub and go back to wait for device to connect
                // HOST_FS
               //  my_hub = this_device[x];
              //   Configure_Hub( my_hub, addr[6], ... );
              // x++;
                break;
            default: // No Device
                //x = 0;
                break;
        } // end switch()
    */               


    // PURPOSE: This function detects a newly connected or disconnected device
    // creates or destroys an object to store the device information and returns
    // whether it is a LS, FS or Hub type device.
    // PRECONDITIONS: Host and SPI Configured, CURRENT = -1, NEXT = (rreg(rHRSL) & 0xC0)
    // interrupts enabled
    // POSTCONDITIONS: Connected Device will be in default mode
    // ARGUMENTS:
    // RETURNS: 0 = disconnect, 1 = FS, 2 = LS, 3 = HUB
    //int Connect_Device( DEVICE * my_device )
    //{
    if ( CURRENT != NEXT )
    {
        CURRENT = NEXT;
        switch ( CURRENT )
        {
            DEVICE *Tmp;
            ///////////////////////////////////////////////////
            case 0x00 : // SE0 - EOP or disconnect
                // Close_Device( my_device );
                Tmp = my_device;
                if ( my_device != NULL ) 
                    my_device = NULL;
                free(Tmp);
                // dtatus = 0;
PO0=0x55;
                break;
             ///////////////////////////////////////////////////
           case bmKSTATUS :    // K STATE
                my_device = Open_Device( );
                if ( my_device == NULL ) 
                {
                    printf( "Failed to create the device!\n" );
                    exit( 1 );
                }
                Configure_Device( my_device, addr4, conf1, idle );
                // status = 1;
PO0=0x0F;
                break;
            ///////////////////////////////////////////////////
            case bmJSTATUS :    // J STATE
                 my_device = Open_Device( );
                if ( my_device == NULL ) 
                {
                    printf( "Failed to create the device!\n" );
                    exit( 1 );
                }
                // wreg(rMODE,(bmDPPULLDN|bmDMPULLDN|bmSOFKAENAB|bmLOWSPEED|bmHOST));  // low speed
                Configure_Device( my_device, addr4, conf1, idle );
                // status = 2;
PO0=0xF0;
                break;
            ///////////////////////////////////////////////////
            default :           // error
                break;
            ///////////////////////////////////////////////////
        } // end switch ( CURRENT )
        FLAG = 0;                         
        // Make sure CONNECT IRQ is enabled
        wreg(rHIEN,bmCONNIRQ);
    } // end if( CURRENT != NEXT )
        
     // add a test to determine...   
    // if(my_device == hub) status = 3;


    // } //end int Connect_Device( DEVICE * my_device )


        // No device, restart loop
        if( my_device == NULL ) continue;

    ////////// SERVICE THE DEVICE HERE ////////////////////////////////

    Read_Keypad();
        
    ///////////////////////////////////////////////////////////////////
    } // end while(1)


///////////////////////////////////////////////////////////////////////
} // End of main(void)
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////



void Read_Keypad(void)              // constantly read the keypad, return if the connect 
{                                   // state changes
        static BYTE HR, rxnum, newkey;
        static BYTE KB_data[8];
        wreg(rHCTL,bmRCVTOG0);              // very first data toggle should be DATA0
        while( !(FLAG & bmCONNIRQ) )
        {
            waitframes(10);
            wreg(rHXFR,0x01);               // send an IN token to EP1 (OUTNIN=0)
            HR = Wait_for_HIRQ(bmHXFRDNIRQ);   // this also clears the IRQ bit
            switch(HR)
            {
                case    hrNAK:  
                    L8_ON  
                    break;       // do nothing
                case    hrSUCCESS:                  // get the KB packet
                    rxnum = rreg(rRCVBC);
                    TRIG_HI                         // scope pulse brackets the 8-byte read
                    readbytes(rRCVFIFO,rxnum,KB_data);  // read "rxnum" bytes from the RCVFIFO
                    wreg(rHIRQ,bmRCVDAVIRQ);        // re-arm the endpont and SWITCH THE BUFFERS

                    TRIG_LO
                    newkey = KB_data[2];            // keycode is in the third byte 
                    if(newkey == 0)             // turn off the LEDS when keys are up
                        PO0=0;                  // (comment out this test to latch keys in LEDS)  
                    else if (newkey >= 0x59 && newkey <= 0x62)  // 89 is 1-key, 98 is 0-key
                        PO0 = newkey - 0x58;    // show it in the LEDS (0 shows as 0x0A)
                        break;
                default:        // any other completion code is an error   
             //       PO0=0xE7;   // all bar LEDS on
                    break;
  //                  while(1);   // hang here and examine error code
            } // end switch(HR)
    } // end while()
}


#endif


