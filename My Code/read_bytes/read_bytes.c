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
//#include "..\interface.c"    // SPI/I2C Implementation
//#include "..\usb.c"          // USB Implementation


///// Declare GLOBALS Here ///////////////////////

__monitor void readbytes(BYTE reg, BYTE N, BYTE *p)
{
    static BYTE j,k;
    static BYTE p2[8];

    SPICF_bit.CHR = 0;                  // Set SPI to 8-bit mode.

    CS_LO
    SPIB = reg<<3;            // write bit b1=0 to command a read operation
    while(SPICN_bit.STBY);    // loop if data still being sent
    k = SPIB;                 // NECESSARY TO RE-ENABLE THE INPUT BUFFER in BYTE MODE

    for(j=0; j<N; j++)
    {
        SPIB = 0x00;            // dummy value to get the next read byte
        while(SPICN_bit.STBY);  // loop if data still being received
        p2[j] = *p = SPIB;              // store it in the data array
        p++;                    // bump the pointer
    }
    CS_HI
   // SPICN_bit.SPIC = 0;           // Clear the SPI tansfer complete flag.
}

///////////////////////////////////////////////////

main()
{
///// Declare Variables Here ///////////////////////

  // 
int rxnum = 10;
int KB_data[8];




///////////////////////////////////////////////////
    SPI_Init();
    Host_Init();
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

///// Your Code Here //////////////////////////////





while(1)
{

  readbytes(rRCVFIFO,rxnum,KB_data);  // read "rxnum" bytes from the RCVFIFO
        
}
    
        
////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////
} // End of main(void)





#endif


