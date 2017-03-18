#include "NRF24L01.h"
#include <stdio.h>
#include <Servo.h>
//***************************************************
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  23  //   unsigned chars TX payload
 #define Relay 7
  #define Motor 6
//***************************************************

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0xaa,0xbb,0xcc,0xdd,0xee
}; // Define a static TX address
unsigned char rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};
//***************************************************
typedef struct{
  int X1;
  int Y1;
  int X2;
  int Y2;
  int X;
  int Y;
  int A;
   int B;
}RX;

typedef struct{
  int Voltage;
}TX;
  
RX *rx;

TX tx;
Servo servo;



void setup() 
{
    Serial.begin(57600);
     pinMode(Relay, OUTPUT);
     servo.attach(2);
  SPI_DIR = ( CE + SCK_pin + CSN + MOSI_pin);
  SPI_DIR &=~ ( IRQ + MISO_pin);
  init_io();                        // Initialize IO port
  unsigned char status=SPI_Read(STATUS);
  initialize_R_T();
  
}

void loop() 
{
  /*
   unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! RF_ReceiveData() && ! timeout )
    if (millis() - started_waiting_at > 250 )
      timeout = true;

     if ( timeout )
  {
   Serial.println("Failed, response timed out.");
   
  }
  else
  {
TX.X1 =22;
TX.Y1 =44;
TX.X2 =55;
TX.Y2 =66;
TX.X =77;
TX.Y =88;
       RF_SendData((char*)&TX);
  }

*/
if(RF_ReceiveData()){

tx.Voltage =analogRead(A0);
//Serial.println(tx.Voltage);
       RF_SendData((char*)&tx);
  
}
  

    //sprintf(kml,"A:%d B:%d C:%d D:%d",Rx_szTemp[0],Rx_szTemp[1],Rx_szTemp[2],Rx_szTemp[3],Rx_szTemp[4]);
   
    // Serial.println(kml);
}


//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{
  SPI_PORT&=~CE;      // chip enable
  SPI_PORT|=CSN;      // Spi disable  
  SPI_PORT&=~SCK_pin;     // Spi clock line init high
}

/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char SPI_RW(unsigned char Byte)
{
  unsigned char i;
  for(i=0;i<8;i++)                      // output 8-bit
  {
    if(Byte&0x80)
    {
      SPI_PORT |=MOSI_pin;    // output 'unsigned char', MSB to MOSI_pin
    }
    else
    {
      SPI_PORT &=~MOSI_pin;
    }
    SPI_PORT|=SCK_pin;                      // Set SCK_pin high..
    Byte <<= 1;                         // shift next bit into MSB..
    if(SPI_IN & MISO_pin)
    {
      Byte |= 1;                // capture current MISO_pin bit
    }
    SPI_PORT&=~SCK_pin;                     // ..then set SCK_pin low again
  }
  return(Byte);                     // return read unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  SPI_PORT&=~CSN;                   // CSN low, init SPI transaction
  status = SPI_RW(reg);             // select register
  SPI_RW(value);                    // ..and write value to it..
  SPI_PORT|=CSN;                    // CSN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  SPI_PORT&=~CSN;                // CSN low, initialize SPI communication...
  SPI_RW(reg);                   // Select register to read from..
  reg_val = SPI_RW(0);           // ..then read register value
  SPI_PORT|=CSN;                 // CSN high, terminate SPI communication

  return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  SPI_PORT&=~CSN;                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  SPI_PORT|=CSN;                   // Set CSN high again

  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  SPI_PORT&=~CSN;                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  SPI_PORT|=CSN;                   // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: Initialize TX & RX mode;
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 * 
 * ToDo: One high pulse(>10us) on CE will now send this
 **************************************************/
void initialize_R_T(void)
{
  SPI_PORT&=~CE;

  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack

  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0

  SPI_RW_Reg(WRITE_REG + RF_CH, 50);        // Select RF channel 50
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x06);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR

  SPI_PORT|=CE;
}
/**************************************************
 * Function: TX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 * 
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
void TX_Mode(void)
{
  SPI_PORT&=~CE;

  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1f); // 500us + 86us, 10 retrans...
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
  SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);

  SPI_PORT|=CE;
}

/**************************************************
 * Function: RX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * RX Mode, set RX address, writes RX payload width,
 * select RF channel, datarate & LNA HCURR.
 * After init, CE is toggled high, which means that
 * this device is now ready to receive a datapacket.
/**************************************************/
void RX_Mode(void)
{
  SPI_PORT&=~CE;
  
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. RX_DR enabled..
  
  SPI_PORT|=CE;                             // Set CE pin high to enable RX device
}

//RF transmit data function
void RF_SendData(char *TXdata)
{
    TX_Mode();                                                  // activate Tx mode
    
    for(short i=0; i<23;i++)
    {
      tx_buf[i] = (unsigned char)*(TXdata+i);                                     // store the data to an array 
    }      
      
    unsigned char status = SPI_Read(STATUS);                   // read register STATUS's value
    
    if(status&TX_DS)                                           // if receive data ready (TX_DS) interrupt
    {
      SPI_RW_Reg(FLUSH_TX,0);                                  
      SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);       // write playload to TX_FIFO
    }
    if(status&MAX_RT)                                         // if receive data ready (MAX_RT) interrupt, this is retransmit than  SETUP_RETR                          
    {
      SPI_RW_Reg(FLUSH_TX,0);
      SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);      // disable standy-mode
    }
    SPI_RW_Reg(WRITE_REG+STATUS,status);                     // clear RX_DR or TX_DS or MAX_RT interrupt flag
    delay(20);
    RX_Mode();    
}


char kml[50];
//RF receive data function
boolean RF_ReceiveData()
{
    RX_Mode();                                                       // activate RX mode
    unsigned char status = SPI_Read(STATUS);                         // read register STATUS's value
    if(status&RX_DR)                                                 // if receive data ready (TX_DS) interrupt
    {
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);             // read playload to rx_buf
      SPI_RW_Reg(FLUSH_RX,0);                                        // clear RX_FIFO                               

rx =(RX*)rx_buf;

      sprintf(kml,"A:%d B:%d C:%d D:%d E:%d F:%d G:%d",rx->X1,rx->Y1,rx->X2,rx->Y2,rx->X,rx->Y,rx->A,rx->B);
      Serial.println(kml);

if(rx->Y){

  digitalWrite(Relay, HIGH);
  Serial.println("OK");
}else{
 digitalWrite(Relay,LOW); 
}


if(rx->Y1>512){
 analogWrite(Motor,map(rx->Y1,512,1023,0,50));
}else{
analogWrite(Motor,0);
  
}

  servo.write(map(rx->X2, 1023, 0, 0, 160));
           SPI_RW_Reg(WRITE_REG+STATUS,status);                             // clear RX_DR or TX_DS or MAX_RT interrupt flag
    delay(20);
    TX_Mode();
return true;
   
    }
     SPI_RW_Reg(WRITE_REG+STATUS,status);                             // clear RX_DR or TX_DS or MAX_RT interrupt flag
    delay(20);
    TX_Mode();
return false;
}


