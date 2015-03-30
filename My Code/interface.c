// interface.c
// SPI specific to the MAXQ board implementation
// 
//

#ifndef INTERFACE_C
#define INTERFACE_C

#include <intrinsics.h>             // MAXQ2000 specific stuff
#include <iomaxq200x.h>             // ditto
#include "..\interface.h"
#include "RT53reg.h"



#define MSB(word) (BYTE)(((WORD)(word) >> 8) & 0xff)
#define LSB(word) (BYTE)((WORD)(word) & 0xff)



#define CS_HI PO5 |= 0x10;
#define CS_LO PO5 &= ~0x10;
#define SCK_HI PO5 |= 0x40;         // PO6
#define SCK_LO PO5 &=~0x40;
#define DRIVE PD5 |= 0x20;          // set direction to 1 (out)
#define FLOAT PD5 &=~0x20;

#define TRIG_HI PO7 = 0x01; // MAXQ P70
#define TRIG_LO PO7 = 0x00;


// Functions



void update_lites(void)     // Read button states from both IOPINS regs, copy to lights in same reg
{
    BYTE bs;
    bs = ~rreg(rGPIO);		// bs=button states. complement to make the buttons active high
    bs >>= 4;
    wreg(rGPIO,bs);
    bs = ~rreg(21);
    bs >>= 4;
    wreg(21,bs);
}


/*
BYTE Check_INT(void)		// returns 0 if nothing pending, nonzero if something pending
{
    // neg-level
    // return ~PI6 & 0x01;         // Return complement of level on P60 pin

    // pos or neg edge (initialization sets edge polarity)
    if(EIF1_bit.IE12)         // Test the IRQ Flag
    {
    EIF1_bit.IE12 = 0;      // It's set--clear it
    return(1);              // show an IRQ is active
    }
  else return(0);           // flag=0: no IRQ active
}
*/



/////////////////////////////////////////////////////////////////
/////////////////////// SPI Functions ///////////////////////////
/////////////////////////////////////////////////////////////////

void SPI_Init(void)
{
    // MAXQ2000 SPI port
//    CKCN = 0x00;              // system clock divisor is 1
    PD5 |= 0x070;             // Set SPI output pins (CS, SCLK, DOUT) as output.
    PD5 &= ~0x080;            // Set SPI input pin (DIN) as input.
    //
//    SPICK = 0x00;             // fastest SPI clock--div by 2 
    SPICK = 0x02;             // fastest SPI clock--div by 2 
    SPICF = 0x00;             // mode(0,0), 8 bit data
    SPICN_bit.MSTM = 1;       // Set Q2000 as the master.
    SPICN_bit.SPIEN = 1;      // Enable SPI
    SPICF_bit.CHR = 1;        // Set SPI to 16-bit mode.
    CS_HI                     // CS# high  


}



// SPI Send Byte Function
// Purpose: Send one BYTE of data to the RT99 via the SPI interface and wait for xfer to complete.
// Precondition: The Q2000 must be interface with the RT99 via the SPI pins and the 
// reg variable contains any BYTE data to be written to the RT99.
// Postcondition: The argument value is written to the SPI port and the proper amount of delay is
// allowed for the transfer. Invalid returned data is discarded.
//void SENDBYTE(int x){ SPIB = x; while(SPICN_bit.STBY); SPIB; }
__monitor WORD sendBYTE(WORD x )
{
//  SPICF_bit.CHR = 1;                  // Set SPI to 16-bit mode.
    
  SPIB = x;                  // Load the data to send
  while(SPICN_bit.STBY);        // Loop until the data has been sent.
  //SPIB;
  SPICN_bit.SPIC = 0;           // Clear the SPI tansfer complete flag.
  return SPIB;
}

// SPI Read Register Function
// Purpose: Takes a BYTE value as an argument that represents a RT99 register address
// and allows proper delay for the data to be returned from the RT99 in that register address. 
// Precondition: The Q2000 must be interface with the RT99 via the SPI pins and the 
// reg variable must contain a hex value of a valid RT99 register address.
// Postcondition: SPIB contains the data returned from the RT99.
__monitor WORD rreg(WORD reg)
{
  SPICF_bit.CHR = 1;                  // Set SPI to 16-bit mode.
  CS_LO
  sendBYTE( 0x0000 | (reg<<11) ); // bit1 0=read, bit7-bit3 register address value. [12 is filler]
  CS_HI
  return SPIB;
}

// SPI Write Register Function
// Purpose: Takes a WORD value as an argument that represents a RT99 register address
// and one BYTE of data and writes a BYTE value to the RT99 register address. 
// Precondition: The Q2000 must be interface with the RT99 via the SPI pins and the 
// reg variable must contain a hex value of a valid RT99 register address.
// Postcondition: The RT99 register has been written with the data BYTE.
__monitor void wreg(WORD reg, BYTE dat)
{
  SPICF_bit.CHR = 1;                  // Set SPI to 16-bit mode.
  CS_LO
  // Combine write command, register and data and send.
  sendBYTE( 0x0200 | (reg<<11) | (WORD)dat );
  CS_HI
}

__monitor void readbytes(BYTE reg, BYTE N, BYTE *p)
{
    static BYTE j;

    SPICF_bit.CHR = 0;                  // Set SPI to 8-bit mode.

    CS_LO
    SPIB = reg<<3;            // write bit b1=0 to command a read operation
    while(SPICN_bit.STBY);    // loop if data still being sent
    j = SPIB;                 // NECESSARY TO RE-ENABLE THE INPUT BUFFER in BYTE MODE

    for(j=0; j<N; j++)
    {
        SPIB = 0x00;            // dummy value to get the next read byte
        while(SPICN_bit.STBY);  // loop if data still being received
        *p = SPIB;              // store it in the data array
        p++;                    // bump the pointer
    }
    CS_HI
   // SPICN_bit.SPIC = 0;           // Clear the SPI tansfer complete flag.
}

__monitor void ReadBytes(BYTE reg, BYTE N, BYTE *p)
{
    static BYTE j;

    SPICF_bit.CHR = 0;                  // Set SPI to 8-bit mode.

    CS_LO
    SPIB = reg<<3;            // write bit b1=0 to command a read operation
    while(SPICN_bit.STBY);    // loop if data still being sent
    SPIB;                 // NECESSARY TO RE-ENABLE THE INPUT BUFFER in BYTE MODE
    for(j=0; j<N; j++)
    {
        SPIB = 0x00;            // dummy value to get the next read byte
        while(SPICN_bit.STBY);  // loop if data still being received
        *p = SPIB;              // store it in the data array
        p++;                    // bump the pointer
    }
    SPICN_bit.SPIC = 0;           // Clear the SPI tansfer complete flag.
    CS_HI
}

__monitor void WriteBytes(BYTE reg, BYTE N, const BYTE *p)
{
    BYTE j, wd;

    SPICF_bit.CHR = 0;                  // Set SPI to 8-bit mode.

    CS_LO
    SPIB = (reg<<3)+2;            // write bit b1=1 to command a write operation
    while(SPICN_bit.STBY);        // loop if data still being sent
    for(j=0; j<N; j++)
    {
        wd = *p;                    // write the array value
        SPIB = wd;
        while(SPICN_bit.STBY);      // loop if data still being received
        p++;                        // bump the pointer
    }
    CS_HI
}



__monitor void writebytes(BYTE reg, BYTE N, BYTE *p)
{
    static BYTE j,wd;

    SPICF_bit.CHR = 0;                  // Set SPI to 8-bit mode.

    CS_LO
    SPIB = (reg<<3)+2;            // write bit b1=1 to command a write operation
    while(SPICN_bit.STBY);        // loop if data still being sent
    for(j=0; j<N; j++)
    {
        wd = *p;                    // write the array value
        SPIB = wd;
        while(SPICN_bit.STBY);      // loop if data still being received
        p++;                        // bump the pointer
    }
    CS_HI
}


/*
// Read a register, return its value.
BYTE rreg(BYTE reg)
{
    BYTE dum;
    CS_LO
    SPIB = reg<<3;                // reg number w. dir=0 (IN)
    while(SPICN_bit.STBY);        // loop if data still being sent
    dum = SPIB;                   // NECESSARY TO RE-ENABLE THE INPUT BUFFER in BYTE MODE
    SPIB=0x12;                    // data is don't care, we're clocking in MISO bits
    while(SPICN_bit.STBY);        // loop if data still being sent
    CS_HI
    return(SPIB);
}
*/



/*
void wreg(BYTE reg, BYTE dat)
{
    CS_LO                         // Set CS# low
    SPIB = (reg<<3)+2;            // send the register number with the DIR bit (b1) set to WRITE
    while(SPICN_bit.STBY);        // loop if data still being sent
    SPIB = dat;                   // send the data
    while(SPICN_bit.STBY);        // loop if data still being sent
    CS_HI                         // set CS# high
    //  SPICN_bit.SPIC = 0;         // Clear the SPI tansfer complete flag.
}
*/


#endif // INTERFACE_C

