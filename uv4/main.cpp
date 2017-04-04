#include <lpc17xx.h>

#define BUFSIZE			0x20 
#define MAX_TIMEOUT		0x00FFFFFF 
 
#define I2CMASTER		0x01 
#define I2CSLAVE		0x02 
 
/* For more info, read Philips's LM95 datasheet */ 
#define LM75_ADDR		0x90 
#define LM75_TEMP		0x00 
#define LM75_CONFIG		0x01 
#define LM75_THYST		0x02 
#define LM75_TOS		0x03 
 
#define RD_BIT			0x01 
 
#define I2C_IDLE			0 
#define I2C_STARTED			1 
#define I2C_RESTARTED		2 
#define I2C_REPEATED_START	3 
#define DATA_ACK			4 
#define DATA_NACK			5 
 
#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */ 
#define I2CONSET_AA			0x00000004 
#define I2CONSET_SI			0x00000008 
#define I2CONSET_STO		0x00000010 
#define I2CONSET_STA		0x00000020 
 
#define I2CONCLR_AAC		0x00000004  /* I2C Control clear Register */ 
#define I2CONCLR_SIC		0x00000008 
#define I2CONCLR_STAC		0x00000020 
#define I2CONCLR_I2ENC		0x00000040 
 
#define I2DAT_I2C			0x00000000  /* I2C Data Reg */ 
#define I2ADR_I2C			0x00000000  /* I2C Slave Address Reg */ 
#define I2SCLH_SCLH			0x00000080  /* I2C SCL Duty Cycle High Reg */ 
#define I2SCLL_SCLL			0x00000080  /* I2C SCL Duty Cycle Low Reg */ 

volatile uint32_t I2CMasterState = I2C_IDLE;   
volatile uint32_t I2CSlaveState = I2C_IDLE;   
   
volatile uint32_t I2CCmd;   
volatile uint32_t I2CMode;   
   
volatile uint8_t I2CMasterBuffer[BUFSIZE];   
volatile uint8_t I2CSlaveBuffer[BUFSIZE];   
volatile uint32_t I2CCount = 0;   
volatile uint32_t I2CReadLength;   
volatile uint32_t I2CWriteLength;   
   
volatile uint32_t RdIndex = 0;   
volatile uint32_t WrIndex = 0;

extern volatile uint32_t I2CCount;   
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];   
extern volatile uint32_t I2CCmd, I2CMasterState;   
extern volatile uint32_t I2CReadLength, I2CWriteLength;

int main (void)   
{   
  uint32_t i;   
   
  SystemInit();   
   
  if ( I2CInit( (uint32_t)I2CMASTER ) == flase )    /* initialize I2c */   
  {   
    while ( 1 );                /* Fatal error */   
  }   
   
  /* the example used to test the I2C interface is   
  a Philips's LM75 temp sensor, as an I2C slave.  
      
  the sequence to get the temp reading is:  
  get device ID register,  
  set configuration register,  
  get temp reading  
  */   
   
  /* In order to start the I2CEngine, the all the parameters   
  must be set in advance, including I2CWriteLength, I2CReadLength,  
  I2CCmd, and the I2cMasterBuffer which contains the stream  
  command/data to the I2c slave device.   
  (1) If it's a I2C write only, the number of bytes to be written is   
  I2CWriteLength, I2CReadLength is zero, the content will be filled   
  in the I2CMasterBuffer.   
  (2) If it's a I2C read only, the number of bytes to be read is   
  I2CReadLength, I2CWriteLength is 0, the read value will be filled   
  in the I2CMasterBuffer.   
  (3) If it's a I2C Write/Read with repeated start, specify the   
  I2CWriteLength, fill the content of bytes to be written in   
  I2CMasterBuffer, specify the I2CReadLength, after the repeated   
  start and the device address with RD bit set, the content of the   
  reading will be filled in I2CMasterBuffer index at   
  I2CMasterBuffer[I2CWriteLength+2].   
    
  e.g. Start, DevAddr(W), WRByte1...WRByteN, Repeated-Start, DevAddr(R),   
  RDByte1...RDByteN Stop. The content of the reading will be filled   
  after (I2CWriteLength + two devaddr) bytes. */   
   
  /* Configure temp register before reading */   
  for ( i = 0; i < BUFSIZE; i++ )    /* clear buffer */   
  {   
    I2CMasterBuffer[i] = 0;   
  }   
  I2CWriteLength = 2;   
  I2CReadLength = 0;   
  I2CMasterBuffer[0] = LM75_ADDR;   
  I2CMasterBuffer[1] = LM75_CONFIG;   
  I2CMasterBuffer[2] = 0x00;    /* configuration value, no change from   
                                default */   
  I2CCmd = LM75_CONFIG;   
  I2CEngine();    
   
  /* Get temp reading */   
  for ( i = 0; i < BUFSIZE; i++ )    /* clear buffer */   
  {   
    I2CMasterBuffer[i] = 0;   
  }   
  I2CWriteLength = 1;   
  I2CReadLength = 2;   
  I2CMasterBuffer[0] = LM75_ADDR;   
  I2CMasterBuffer[1] = LM75_TEMP;   
  I2CMasterBuffer[2] = LM75_ADDR | RD_BIT;   
  I2CCmd = LM75_TEMP;   
  I2CEngine();   
   
  /* The temp reading value should reside in I2CMasterBuffer byte 3, 4 */    
  while(1);   
   
}   


void I2C1_IRQHandler(void) {
	
	uint8_t statusValue;							// This handler deals with master read and master write only
	statusValue = LPC_I2C1->I2STAT;
	
	switch ( statusValue ) {
	
		// Int: start condition has been transmitted
    // Do:  send SLA+R or SLA+W
		case 0x08:			
			LPC_I2C1->I2DAT = I2CMasterBuffer[0];
			LPC_I2C1->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
			I2CMasterState = I2C_STARTED;
			break;
		
		// Int: repeated start condition has been transmitted
    // Do:  send SLA+R or SLA+W
		case 0x10:			
			if( I2CCmd == MLX90614_TEMP ) {
				LPC_I2C1->I2DAT = I2CMasterBuffer[2];
			}
			LPC_I2C1->I2CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
			I2CMasterState = I2C_RESTARTED;
			break;
			
		// Int: SLA+W has been transmitted, ACK received
    // Do:  send first byte of buffer if available
		case 0x18:			
			if ( I2CMasterState == I2C_STARTED ) {
				LPC_I2C1->I2DAT = I2CMasterBuffer[1+WrIndex];
				WrIndex++;
				I2CMasterState = DATA_ACK;
			}
			LPC_I2C1->I2CONCLR = I2CONCLR_SIC;
			break;
			
		// Int: SLA+W has been transmitted, NACK received
    // Do:  stop!
		case 0x20:
			
	  // Int: data byte has been transmitted, ACK received
    // Do:  load next byte if available, else stop
		case 0x28:
			
		// Int: data byte has been transmitted, NACK received
    // Do:  stop!
		case 0x30:		//Data byte in I2DAT has been transmitted; NOT ACK has been received.
			if ( WrIndex != I2CWriteLength )   
    {      
      LPC_I2C1->I2DAT = I2CMasterBuffer[1+WrIndex]; /* this should be the last one */   
      WrIndex++;   
      if ( WrIndex != I2CWriteLength )   
      {      
        I2CMasterState = DATA_ACK;   
      }   
      else   
      {   
        I2CMasterState = DATA_NACK;   
        if ( I2CReadLength != 0 )   
        {   
          LPC_I2C1->I2CONSET = I2CONSET_STA; /* Set Repeated-start flag */   
          I2CMasterState = I2C_REPEATED_START;   
        }   
      }   
    }   
    else   
    {   
      if ( I2CReadLength != 0 )   
      {   
        LPC_I2C1->I2CONSET = I2CONSET_STA;   /* Set Repeated-start flag */   
        I2CMasterState = I2C_REPEATED_START;   
      }   
      else   
      {   
        I2CMasterState = DATA_NACK;   
        LPC_I2C1->I2CONSET = I2CONSET_STO;      /* Set Stop flag */   
      }   
    }   
    LPC_I2C1->I2CONCLR = I2CONCLR_SIC;   
    break;  
		
		// Int: arbitration lost in SLA+R/W or Data bytes
    // Do:  release bus
		case 0x38:
					
    // Int: SLA+R has been transmitted, ACK received
    // Do:  determine if byte is to be received
		 case 0x40:  /* Master Receive, SLA_R has been sent */   
    LPC_I2C1->I2CONSET = I2CONSET_AA;    /* assert ACK after data is received */   
    LPC_I2C1->I2CONCLR = I2CONCLR_SIC;   
    break; 
		 
		// Int: SLA+R has been transmitted, NACK received
    // Do:  stop!
		 case 0x48:   
    LPC_I2C0->I2CONCLR = I2CONCLR_SIC;   
    I2CMasterState = DATA_NACK;   
    break;
		 
		// Int: data byte has been received, ACK has been returned
    // Do:  read byte, determine if another byte is needed
    case 0x50:
			
		// Int: data byte has been received, NACK has been returned
    // Do:  transfer is done, stop.
		case 0x58:   
    I2CMasterBuffer[3+RdIndex] = LPC_I2C0->I2DAT;   
    RdIndex++;   
    if ( RdIndex != I2CReadLength )   
    {      
      I2CMasterState = DATA_ACK;   
    }   
    else   
    {   
      RdIndex = 0;   
      I2CMasterState = DATA_NACK;   
    }   
    LPC_I2C0->I2CONSET = I2CONSET_AA;    /* assert ACK after data is received */   
    LPC_I2C0->I2CONCLR = I2CONCLR_SIC;   
    break;
		
		 default:   
    LPC_I2C0->I2CONCLR = I2CONCLR_SIC;      
    break;
		
	}
	
}

uint32_t I2CStart( void )   
{   
  uint32_t timeout = 0;   
  uint32_t retVal = false;   
    
  /*--- Issue a start condition ---*/   
  LPC_I2C1->I2CONSET = I2CONSET_STA; /* Set Start flag */   
       
  /*--- Wait until START transmitted ---*/   
  while( 1 )   
  {   
    if ( I2CMasterState == I2C_STARTED )   
    {   
      retVal = false;   
      break;       
    }   
    if ( timeout >= MAX_TIMEOUT )   
    {   
      retVal = false;   
      break;   
    }   
    timeout++;   
  }   
  return( retVal );   
}   


uint32_t I2CStop( void )   
{   
  LPC_I2C1->I2CONSET = I2CONSET_STO;  /* Set Stop flag */    
  LPC_I2C1->I2CONCLR = I2CONCLR_SIC;  /* Clear SI flag */    
               
  /*--- Wait for STOP detected ---*/   
  while( LPC_I2C1->I2CONSET & I2CONSET_STO );   
  return true;   
}   

uint32_t I2CEngine( void )    
{   
  I2CMasterState = I2C_IDLE;   
  RdIndex = 0;   
  WrIndex = 0;   
  if ( I2CStart() != true )   
  {   
    I2CStop();   
    return ( false );   
  }   
   
  while ( 1 )   
  {   
    if ( I2CMasterState == DATA_NACK )   
    {   
      I2CStop();   
      break;   
    }   
  }       
  return ( true );         
}   













