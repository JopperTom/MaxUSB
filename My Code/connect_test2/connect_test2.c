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

// Host or peripheral? - define only one.
#define HST
//#define PER

// Low or Full Speed? - define only one.
//#define HS
#define LS

// Which interface? - define only one.
#define IFACE_SPI
//#define IFACE_I2C

#define TEMP

#include <stdlib.h>
#include <stdio.h>

#include <intrinsics.h>      // MAXQ2000 specific stuff
#include <iomaxq200x.h>      // MAXQ2000 specific stuff
#include "RT53reg.h"         // RT99 Registers and bits
#include "..\interface.c"    // SPI/I2C Implementation
#include "..\usb.c"          // USB Implementation


///// Declare GLOBALS Here ///////////////////////

int CONNECT = -1;


//int host_send_data( unsigned char c, DEVICE *device );

//int host_recv_data( DEVICE *device );

// This will be an array of configured devices
// to accomodate one hub and one connected device
//  DEVICE *this_device = NULL;
//  this_device = malloc( sizeof( DEVICE ));
//      if ( this_device == NULL) exit( 1 );




// prototypes in this module

int ConnectStatus( void );
void Read_Keypad(void); 


//********************
///////////////////////////////////////////////////

main()
{
///// Declare Variables Here ///////////////////////

  // 

////    static BYTE dum, result, dum1;
static DEVICE *my_device = NULL;
///////////////////////////////////////////////////

    Host_Init();

///// Your Code Here //////////////////////////////

    wreg(rHCTL,0x04); // Sample the USB bus
    CURRENT = -1;
    NEXT = (rreg(rHRSL) & 0xC0);

    timeDELAY(500);		// a delay

    
    while(1)
    {
        // Wait for a device to connect
        
/*        switch( Connect_Device( this_device ) )
        {
            case 1: // Configure LS Peripheral and drop out to Read_Keypad()
                // HOST_LS
   //           Configure_Device( this_device[x], addr[2], conf1, idle );
                //my_device++;
                break;
            case 2: // Configure FS Peripheral and drop out to Read_Keypad()
                // HOST_FS
   //             Configure_Device( this_device[x], addr[4], conf1, idle );
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
    if ( CURRENT != NEXT )
    {
        CURRENT = NEXT;
        // wreg(rHIEN,0x00);
        switch ( CURRENT )
        {
            case 0x00 : // SE0 - EOP or disconnect
   // PO0 = 0xCC;       // MAXQ: 
               // if ( my_device != NULL ) 
                Close_Device( my_device );
//                free(my_device);
 //               my_device = NULL;
              //  status = 0;
                break;
            case bmKSTATUS :    // K STATE
   // PO0 = 0xC0;       // MAXQ: 
                ///////////////////////////////////////////////////
                my_device = Open_Device( addr4 );
                if ( my_device == NULL ) 
                {
                    printf( "Failed to create the device!\n" );
                    exit( 1 );
                }
    //            this_device = my_device;
//                Configure_Device( my_device, addr4, conf1, idle );
                ///////////////////////////////////////////////////
              //  status = 1;
                break;
            case bmJSTATUS :    // J STATE
   // PO0 = 0x0C;       // MAXQ: 
                ///////////////////////////////////////////////////

                my_device = Open_Device( addr4 );
                if ( my_device == NULL ) 
                {
                    printf( "Failed to create the device!\n" );
                    exit( 1 );
                }
//                Configure_Device( my_device, addr4, conf1, idle );
      //          this_device = my_device;
                ///////////////////////////////////////////////////
              //  status = 2;
                break;
            default :           // error
                break;
        } // end switch ( CURRENT )
        wreg(rHIEN,bmCONNIRQ);
    } // end if( CURRENT != NEXT )
        
     // add a test to determine...   
    // if(my_device == hub) status = 3;
                            
        // No device, restart loop
        if( my_device == NULL ) continue;

////////// SERVICE THE DEVICE HERE /////////////////////////////////////

    Read_Keypad();


 //       dum=rreg(rHRSL);
 //       result = CONNECT;
        

    
        
////////////////////////////////////////////////////////////////////////
    } // end while(1)


/////////////////////////////////////////////////////
} // End of main(void)



void Read_Keypad(void)              // constantly read the keypad, return if the connect 
{                                   // state changes
        static BYTE HR, rxnum, newkey, temp, i;
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
                   // wreg(rHIRQ,bmRCVDAVIRQ);        // re-arm the endpont and SWITCH THE BUFFERS

/*                    for(i = 0; i<rxnum;i++)
                    {
                        temp = rreg(rRCVFIFO);
                        if(i==2) KB_data = temp;
                    }
*/
                    TRIG_LO
                    newkey = *KB_data;            // keycode is in the third byte 
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


