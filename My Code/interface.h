// interface.h
// SPI specific to the MAXQ board prototypes
// 
//

#ifndef INTERFACE_H
#define INTERFACE_H


void update_lites(void);

// SPI Specific Prototypes

void SPI_Init(void);


__monitor WORD sendBYTE( WORD );
__monitor WORD rreg( WORD );
__monitor void wreg( WORD, BYTE );


//void wreg(BYTE r,BYTE v);
//BYTE rreg(BYTE r);

BYTE rreg_bb(BYTE reg);
void wreg_bb(BYTE r,BYTE v);

__monitor void readbytes(BYTE reg, BYTE N, BYTE *p);
__monitor void ReadBytes(BYTE reg, BYTE N, BYTE *p);
__monitor void writebytes(BYTE reg, BYTE N, BYTE *p);
__monitor void WriteBytes(BYTE reg, BYTE N, const BYTE *p);

void wregAS(BYTE r,BYTE v);
BYTE rregAS(BYTE r);

BYTE rreg16(BYTE reg);
void wreg16(BYTE reg,BYTE dat); 


#endif
