#include <LPC17xx.h>


//P0.19	SDA1
//P0.20	SCL1

#define WAIT_SI while(!(LPC_I2C1->I2CONSET & (1<<3)))
#define CLEAR_SI LPC_I2C1->I2CONCLR = 1<<3

#define AA      (1 << 2)
#define SI      (1 << 3)
#define STO     (1 << 4)
#define STA     (1 << 5)
#define I2EN    (1 << 6)


void init_I2C1();
void I2C1_Enable();
void I2C1_Start();
void I2C1_Stop();
unsigned char I2C1_Address(uint32_t ADDR);
void I2C1_Write(char cmd);
unsigned char I2C2_Read(char ack);


int main() {
	unsigned char status;
	init_I2C1();
	I2C1_Enable();
	I2C1_Start();
	status=I2C1_Address(0x29<<1);
	if(status == 0x18) {
			I2C1_Stop();
		
	}
	if(status == 0x20)
		I2C1_Write(0x07);
	
	while(1);
	
	
}

void init_I2C1(){
	uint32_t pclk;
	LPC_SC->PCONP |= 1<<19;		//enable power to I2C_1
	LPC_SC -> PCLKSEL1 &= ~(((1<<6)|(1<<7)));
	pclk = SystemCoreClock;

	LPC_GPIO0->FIODIR |= 1<<19;
	LPC_GPIO0->FIODIR |= 1<<20;
	
	  LPC_PINCON->PINSEL1 &= ~((0x3<<6)|(0x3<<8));
  LPC_PINCON->PINSEL1 |= ((0x3<<6)|(0x3<<8));
  LPC_PINCON->PINMODE1 &= ~((0x3<<6)|(0x3<<8));
  LPC_PINCON->PINMODE1 |= ((0x2<<6)|(0x2<<8));	/* No pull-up no pull-down */
  LPC_PINCON->PINMODE_OD0 |= ((0x1<<19)|(0x1<<20));	
	
	LPC_I2C1->I2SCLH = 0x7D;
	LPC_I2C1->I2SCLL = 0x7D;
}

void I2C1_Enable() {
	LPC_I2C1->I2CONSET |= I2EN;
}

void I2C1_Start() {
	LPC_I2C1->I2CONSET |= STA;
	WAIT_SI;
}

void I2C1_Stop() {
	LPC_I2C1->I2CONSET |= STO;
	CLEAR_SI;
	while(LPC_I2C1->I2CONSET & (STO));
}

unsigned char I2C1_Address(uint32_t ADDR) {
	LPC_I2C1->I2DAT = ADDR;
	LPC_I2C1->I2CONSET = AA;
	LPC_I2C1->I2CONCLR = STA;
	LPC_I2C1->I2CONCLR = SI;
	WAIT_SI;
	return (LPC_I2C1->I2STAT);
}

void I2C1_Write(char cmd) {
	LPC_I2C1->I2DAT = cmd;
	CLEAR_SI;
	WAIT_SI;
}

unsigned char I2C2_Read(char ack) {
	if(ack)
		LPC_I2C1->I2CONSET = AA;
	else
		LPC_I2C1->I2CONCLR = AA;
	CLEAR_SI;
	WAIT_SI;
	return (LPC_I2C1->I2DAT);
}