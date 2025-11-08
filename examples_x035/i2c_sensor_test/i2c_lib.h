#define I2C_TIMEOUT 100000

u8 I2C_AWAIT(u32 reg, u32 mask, u8 expected) {
	u32 timeout = I2C_TIMEOUT;
	while(((*(volatile u32*)reg & mask) ? 1 : 0) != expected) {
		if(timeout-- == 0) return 0;
	}
	return 1;
}

//! ####################################
//! I2C INIT FUNCTION
//! ####################################

void i2c_init(u8 systemClock_MHz, u32 i2cSpeed_KHz) {
	// Enable I2C clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

	 // Disable I2C before configuration
	I2C1->CTLR1 &= ~I2C_CTLR1_PE;
	I2C1->CTLR1 |= CTLR1_ACK_Set;
	
	// Set clock frequency (assuming 8MHz system clock)
	I2C1->CTLR2 = systemClock_MHz;
	I2C1->CKCFGR = systemClock_MHz*1000 / (i2cSpeed_KHz << 1);	// SystemClockHz / (100KHz * 2)
	
	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;
}

void i2c_start(u8 i2cAddress, u8 isRead) {
	// Wait until I2C is not busy
	while(I2C1->STAR2 & I2C_STAR2_BUSY);
	// Generate START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;

	// Wait for SB flag
	while(!(I2C1->STAR1 & I2C_STAR1_SB));
	// Send address + read/write. Write = 0, Read = 1
	I2C1->DATAR = (i2cAddress << 1) | isRead;

	// Wait for ADDR flag
	while(!(I2C1->STAR1 & I2C_STAR1_ADDR));

	//! REQUIRED. Clear ADDR by reading STAR1 then STAR2
	(void)I2C1->STAR1;
	(void)I2C1->STAR2;
}


//! ####################################
//! I2C SEND FUNCTION
//! ####################################

void i2c_sendBytes(u8 i2cAddress, u8* data, u8 len) {
	i2c_start(i2cAddress, 0); // Write mode

	// Send all bytes
	for(u8 i = 0; i < len; i++) {
		// Wait for register empty
		while(!(I2C1->STAR1 & I2C_STAR1_TXE));
		I2C1->DATAR = data[i];		// Send data
	}

	// Wait for transmission complete
	while(!(I2C1->STAR1 & I2C_STAR1_BTF));
	
	// Generate STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
}

void i2c_sendByte(u8 i2cAddress, u8 data) {
	i2c_sendBytes(i2cAddress, &data, 1);
}


//! ####################################
//! I2C RECEIVE FUNCTION
//! ####################################

void i2c_readBytes(u8 i2cAddress, u8* buffer, u8 len) {
	i2c_start(i2cAddress, 1); // Read mode
	
	// Enable ACK at the beginning
	I2C1->CTLR1 |= I2C_CTLR1_ACK;

	// Read all bytes
	for(u8 i = 0; i < len; i++) {
		// Before reading the last bytes, disable ACK to signal the slave to stop sending
		if(i == len-1) I2C1->CTLR1 &= ~I2C_CTLR1_ACK;
		
		// Wait for data and read
		while(!(I2C1->STAR1 & I2C_STAR1_RXNE));
		buffer[i] = I2C1->DATAR;
	}
	
	// Generate STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
}

u8 i2c_readByte(u8 i2cAddress) {
	u8 output;
	i2c_readBytes(i2cAddress, &output, 1);
	return output;
}

// Write to register and then do read data
u8 i2c_readReg(u8 i2cAddress, u8 reg, u8 *buffer, u8 len) {
    i2c_sendByte(i2cAddress, reg);              // Send register address
    i2c_readBytes(i2cAddress, buffer, len);  // Read data
    return 1;
}