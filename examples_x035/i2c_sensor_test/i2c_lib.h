// MIT License
// Copyright (c) 2025 UniTheCat

#define I2C_TIMEOUT 100000

//! ####################################
//! I2C INIT FUNCTIONS
//! ####################################

void i2c_init(u32 systemClock_Hz, u32 i2cSpeed_Hz) {
	// Enable I2C clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

	// Disable I2C before configuration
	I2C1->CTLR1 &= ~I2C_CTLR1_PE;
	I2C1->CTLR2 = systemClock_Hz / 1000000;
	I2C1->CKCFGR = systemClock_Hz / (i2cSpeed_Hz << 1);	// SystemClockHz / (100KHz * 2)
	
	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	// Enable ACK
	I2C1->CTLR1 |= I2C_CTLR1_ACK;
}

u8 i2c_start(u8 i2cAddress, u8 isRead) {
	//# Wait while BUSY, when BUSY is set to 0 then continue
	u32 timeout = I2C_TIMEOUT;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && timeout--);
	if (timeout == 0) { I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0x11; }

	//# Generate START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;

	//# Wait while SB is 0, when SB is set to 1 then continue
	timeout = I2C_TIMEOUT;
	while(!(I2C1->STAR1 & I2C_STAR1_SB) && timeout--);
	if (timeout == 0) { I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0x12; }
	// printf("timeoutB: %d\n", I2C_TIMEOUT - timeout);

	//# Send address + read/write. Write = 0, Read = 1
	I2C1->DATAR = (i2cAddress << 1) | isRead;

	//# Wait while ADDR is 0, if ADDR is set to 1 then continue
	timeout = I2C_TIMEOUT;
	while(!(I2C1->STAR1 & I2C_STAR1_ADDR) && timeout--);
	if (timeout == 0) { I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0x13; }
	// printf("timeoutC: %d\n", I2C_TIMEOUT - timeout);

	//! REQUIRED. Clear ADDR by reading STAR1 then STAR2
	(void)I2C1->STAR1;
	(void)I2C1->STAR2;
	return 0;
}

//! ####################################
//! I2C SEND FUNCTION
//! ####################################

u8 i2c_sendBytes_noStop(u8 i2cAddress, u8* buffer, u8 len) {
	u8 err = i2c_start(i2cAddress, 0); // Write mode
	if (err) return err;
	u32 timeout;

	for(u8 i = 0; i < len; i++) {
		//# Wait for register empty
		timeout = I2C_TIMEOUT;
		while(!(I2C1->STAR1 & I2C_STAR1_TXE) && timeout--);
		if (timeout == 0) { I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0x21; }
		I2C1->DATAR = buffer[i];		// Send data
	}

	//# Wait for transmission complete. Wait while BTF is 0, when set to 1 continue
	timeout = I2C_TIMEOUT;
	while(!(I2C1->STAR1 & I2C_STAR1_BTF) && timeout--);
	if (timeout == 0) { I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0x22; }
	
	return 0;
}

u8 i2c_sendBytes(u8 i2cAddress, u8* buffer, u8 len) {
	u8 err = i2c_sendBytes_noStop(i2cAddress, buffer, len);
	//# Generate STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
	return err;
}

u8 i2c_sendByte(u8 i2cAddress, u8 data) {
	return i2c_sendBytes(i2cAddress, &data, 1);
}


//! ####################################
//! I2C RECEIVE FUNCTIONS
//! ####################################

u8 i2c_readBytes(u8 i2cAddress, u8* buffer, u8 len) {
	u8 err = i2c_start(i2cAddress, 1); // Read mode
	if (err) return err;

	//# Enable ACK at the beginning
	I2C1->CTLR1 |= I2C_CTLR1_ACK;

	for(u8 i = 0; i < len; i++) {
		//# Before reading the last bytes, disable ACK to signal the slave to stop sending
		if(i == len-1) I2C1->CTLR1 &= ~I2C_CTLR1_ACK;
		
		//# Wait for data. Wait while RxNE is 0, when set to 1 continue
		u32 timeout = I2C_TIMEOUT;
		while(!(I2C1->STAR1 & I2C_STAR1_RXNE) && timeout--);
		if (timeout == 0) { I2C1->CTLR1 |= I2C_CTLR1_STOP; return 0x31; }

		//# Read data
		buffer[i] = I2C1->DATAR;
	}
	
	//# Generate STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
	return 0;	
}

// Write to register and then do read data, no stop inbetween
u8 i2c_readRegTx_buffer(u8 i2cAddress, u8 *tx_buf, u8 tx_len, u8 *rx_buf, u8 rx_len) {
	u8 err = i2c_sendBytes_noStop(i2cAddress, tx_buf, tx_len);	// Send register address
	if (err) return err;
	err = i2c_readBytes(i2cAddress, rx_buf, rx_len); 	// Read data
	return err;
}

u8 i2c_readReg_buffer(u8 i2cAddress, u8 reg, u8 *rx_buf, u8 rx_len) {
	return i2c_readRegTx_buffer(i2cAddress, &reg, 1, rx_buf, rx_len);
}

//! ####################################
//! I2C SLAVE FUNCTIONS
//! ####################################

void i2c_slave_init(u16 self_addr, u32 systemClock_Hz, u32 i2cSpeed_Hz) {
	// Enable I2C clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;
	 // Disable I2C before configuration
	I2C1->CTLR1 &= ~I2C_CTLR1_PE;

	// << 1 means x2
	I2C1->CTLR2 |= (systemClock_Hz / 1000000) & I2C_CTLR2_FREQ;
	I2C1->CKCFGR = systemClock_Hz / (i2cSpeed_Hz << 1);
	I2C1->OADDR1  = (self_addr << 1);
	I2C1->OADDR2 = 0;

	// Enable Event and Error Interrupts
	I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN | I2C_CTLR2_ITBUFEN;
	NVIC_EnableIRQ(I2C1_EV_IRQn); // I2C Event interrupt
	NVIC_EnableIRQ(I2C1_ER_IRQn); // I2C Error interrupt

	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	// Enable ACK
	I2C1->CTLR1 |= I2C_CTLR1_ACK;
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) {
	uint16_t STAR1 = I2C1->STAR1;

	// Obtain and clear Bus error
	if (STAR1 & I2C_STAR1_BERR) {
		I2C1->STAR1 &= ~(I2C_STAR1_BERR);
	}

	// Obtain and clear Arbitration lost error
	if (STAR1 & I2C_STAR1_ARLO) {
		I2C1->STAR1 &= ~(I2C_STAR1_ARLO);
	}

	// Obtain and clear Acknowledge failure error
	if (STAR1 & I2C_STAR1_AF) {
		I2C1->STAR1 &= ~(I2C_STAR1_AF);
	}
}