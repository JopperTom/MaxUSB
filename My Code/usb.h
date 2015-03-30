#ifndef USB_H
#define USB_H

#define BUFSIZ 8

BYTE INTERRUPT;
int FLAG, FRAMECOUNT, CURRENT, NEXT;
BYTE errorcode;
BYTE HR1,HR2,HR3;

const SUD addr4 = {0x00,0x05,0x04,0x00,0x00,0x00,0x00,0x00};
const SUD conf1 = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00};
const SUD idle  = {0x21,0x0A,0x00,0x00,0x00,0x00,0x00,0x00};




BYTE CTL_WRITE(void);
void Host_Init( void );

typedef struct {
                 BYTE bLength;
                 BYTE bDescriptorType;
                 BYTE bcdUSB[2];
                 BYTE bDeviceClass;
                 BYTE bDeviceSubClass;
                 BYTE bDeviceProtocol;
                 BYTE bMaxPacketSize;
                 BYTE idVendor[2];
                 BYTE idProduct[2];
                 BYTE bcdDevice[2];
                 BYTE iManufacture;
                 BYTE iProduct;
                 BYTE iSerialNumber;
                 BYTE bNumConfigurations;
} DEVICE_DESCR;

typedef struct {
                 BYTE bLength;
                 BYTE bDescriptorType;
                 BYTE wTotalLength[2];
                 BYTE bNumInterfaces;
                 BYTE bConfigurationValue;
                 BYTE iConfiguration;
                 BYTE bmAttributes;
                 BYTE bMaxPower;
} CONFIG_DESCR;

typedef struct {
                 BYTE bLength;
                 BYTE bDescriptorType;
                 BYTE bInterfaceNumber;
                 BYTE bAlternateSetting;
                 BYTE bNumEndpoints;
                 BYTE bInterfaceClass;
                 BYTE bInterfaceSubClass;
                 BYTE bInterfaceProtocol;
                 BYTE iInterface;
} INTERFACE_DESCR;

typedef struct {
                 BYTE bLength;
                 BYTE bDescriptorType;
                 BYTE bEndpoinAddress;
                 BYTE bmAttributes;
                 BYTE wMaxPacketSize;
                 BYTE bInterval;
} ENDPOINT_DESCR;


typedef struct {
                 SUD address;
                 SUD config;
                 SUD idle;
            //     DEVICE_DESCR device_descr;
            //     CONFIG_DESCR config_descr;
            //     INTERFACE_DESCR interface_descr;
            //     ENDPOINT_DESCR endpoint_1;
} DEVICE;

int Connect_Device( DEVICE * );

DEVICE *Open_Device( void );

void Close_Device( DEVICE *device );

void Configure_Device( DEVICE *device,
               const SUD address,
               const SUD config,
               const SUD idle);

void Configure_Hub( DEVICE *device,
               const SUD address,
               const SUD config,
               const SUD idle);


int host_send_data( unsigned char c, DEVICE *device );

int host_recv_data( DEVICE *device );



int FRAME = 0;

BYTE Check_INT(void);

void StallEP0(void);


void Bus_Reset(void);
void Bus_Resume(void);
void Reset_MAX(int);


BYTE Wait_for_HIRQ(BYTE regbit);


void Ack_Status(void);
BYTE readstat(void);

void wregAS16(BYTE reg,BYTE dat);
void std_request(void);
void class_request(void);
void vendor_request(void);
void send_descriptor(void);


// Host-specific
BYTE check_HRSL(void);
BYTE CTL_WRITE(void);
BYTE CTL_READ(void);
void waitframes(int num);
BYTE Set_Address(BYTE addr);
BYTE Get_Device_Status(void);
BYTE H_GetStat(BYTE port);
BYTE HubPort_Feature(BYTE setnotclr,BYTE feat,BYTE port);
BYTE Set_Idle(BYTE iface, BYTE duration, BYTE reportID);
BYTE Set_Config(BYTE cfgval);
BYTE Get_Descriptor(BYTE type,WORD length);

//void Get_Descriptor(BYTE, DEVICE*);
void Set_Descriptor(BYTE, DEVICE*);
//void Get_Descriptor(BYTE, DEVICE*);
void Get_Configuration( DEVICE* );
void Set_Configuration( DEVICE* );
//void Get_Interface( DEVICE* );
//void Set_Interface( DEVICE* );



// SPI_x.C prototypes
void set_addr(void);
void send_keystroke(BYTE);
void feature(BYTE);
void get_status(void);
void set_interface(void);
void get_interface(void);
void set_configuration(void);
void get_configuration(void);

BYTE H_D_GetStat(void);


#endif
