#define I2C_TIMEOUT 100000

u8 I2C_AWAIT(u32 reg, u32 mask, u8 expected) {
	u32 timeout = I2C_TIMEOUT;
	while(((*(volatile u32*)reg & mask) ? 1 : 0) != expected) {
		if(timeout-- == 0) return 0;
	}
	return 1;
}

//! ####################################
//! I2C INIT FUNCTIONS
//! ####################################

void i2c_init(u32 systemClock_Hz, u32 i2cSpeed_Hz) {
	// Enable I2C clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

	// Disable I2C before configuration
	I2C1->CTLR1 &= ~I2C_CTLR1_PE;

	// Set clock frequency (assuming 8MHz system clock)
	I2C1->CTLR2 = systemClock_Hz / 1000000;
	I2C1->CKCFGR = systemClock_Hz / (i2cSpeed_Hz << 1);	// SystemClockHz / (100KHz * 2)
	
	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	// Enable ACK
	I2C1->CTLR1 |= I2C_CTLR1_ACK;
}


u8 i2c_start(u8 i2cAddress, u8 isRead) {
	// Wait until I2C is not busy
	u32 timeout = I2C_TIMEOUT;
	while(I2C1->STAR2 & I2C_STAR2_BUSY && timeout--);
	if (timeout == 0) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 0x11;
	}

	// Generate START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;

	// Wait for SB flag
	timeout = I2C_TIMEOUT;
	while(!(I2C1->STAR1 & I2C_STAR1_SB) && timeout--);
	if (timeout == 0) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 0x12;
	}

	// Send address + read/write. Write = 0, Read = 1
	I2C1->DATAR = (i2cAddress << 1) | isRead;

	// Wait for ADDR flag
	timeout = I2C_TIMEOUT;
	while(!(I2C1->STAR1 & I2C_STAR1_ADDR) && timeout--);
	if (timeout == 0) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 0x13;
	}

	//! REQUIRED. Clear ADDR by reading STAR1 then STAR2
	(void)I2C1->STAR1;
	(void)I2C1->STAR2;

	return 0;
}

//! ####################################
//! I2C SEND FUNCTION
//! ####################################

u8 i2c_sendBytes(u8 i2cAddress, u8* data, u8 len) {
	u8 err = i2c_start(i2cAddress, 0); // Write mode
	if (err) return err;

	u32 timeout;

	// Send all bytes
	for(u8 i = 0; i < len; i++) {
		// Wait for register empty
		timeout = I2C_TIMEOUT;
		while(!(I2C1->STAR1 & I2C_STAR1_TXE) && timeout--);
		if (timeout == 0) {
			I2C1->CTLR1 |= I2C_CTLR1_STOP;
			return 0x21;
		}
		I2C1->DATAR = data[i];		// Send data
	}

	// Wait for transmission complete
	timeout = I2C_TIMEOUT;
	while(!(I2C1->STAR1 & I2C_STAR1_BTF) && timeout--);
	if (timeout == 0) {
		I2C1->CTLR1 |= I2C_CTLR1_STOP;
		return 0x22;
	}

	// Generate STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;

	return 0;
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
	
	// Enable ACK at the beginning
	I2C1->CTLR1 |= I2C_CTLR1_ACK;

	// Read all bytes
	for(u8 i = 0; i < len; i++) {
		// Before reading the last bytes, disable ACK to signal the slave to stop sending
		if(i == len-1) I2C1->CTLR1 &= ~I2C_CTLR1_ACK;
		
		// Wait for data and read
		u32 timeout = I2C_TIMEOUT;
		while(!(I2C1->STAR1 & I2C_STAR1_RXNE) && timeout--);
		if (timeout == 0) {
			I2C1->CTLR1 |= I2C_CTLR1_STOP;
			return 0x31;
		}
		buffer[i] = I2C1->DATAR;
	}
	
	// Generate STOP condition
	I2C1->CTLR1 |= I2C_CTLR1_STOP;

	return 0;
}

u8 i2c_readByte(u8 i2cAddress) {
	u8 output;
	i2c_readBytes(i2cAddress, &output, 1);
	return output;
}

// Write to register and then do read data
u8 i2c_readReg_buffer(u8 i2cAddress, u8 reg, u8 *buffer, u8 len) {
	i2c_sendByte(i2cAddress, reg);			  // Send register address
	i2c_readBytes(i2cAddress, buffer, len);  // Read data
	return 1;
}

u8 i2c_readReg_byte(u8 i2cAddress, u8 reg) {
	u8 data;
	i2c_readReg_buffer(i2cAddress, reg, &data, 1);
	return data;
}

//! ####################################
//! I2C SLAVE FUNCTIONS
//! ####################################
#define CTLR1_CLEAR_Mask         ((uint16_t)0xFBF5)

void i2c_slave_init(u16 self_addr, u32 systemClock_Hz, u32 i2cSpeed_Hz) {
	// Enable I2C clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;
	
	 // Disable I2C before configuration
	I2C1->CTLR1 &= ~I2C_CTLR1_PE;

	// << 1 means x2
	I2C1->CTLR2 |= (systemClock_Hz / 1000000) & I2C_CTLR2_FREQ;

	printf("systemClock_hz: %d\n", systemClock_Hz);
	printf("i2cSpeed_Hz: %d\n", i2cSpeed_Hz);
	I2C1->CKCFGR = systemClock_Hz / (i2cSpeed_Hz << 1);

	// Enable Event and Error Interrupts
	I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN | I2C_CTLR2_ITBUFEN;
	NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
	NVIC_SetPriority(I2C1_EV_IRQn, 2 << 4);
	NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
	NVIC_SetPriority(I2C1_ER_IRQn, 2 << 4);

	// // I2C1->CTLR1 &= ~I2C_CTLR1_NOSTRETCH;  // Ensure stretching is ENABLED

	I2C1->OADDR1  = (self_addr << 1);
	I2C1->OADDR2 = 0;

	// Enable I2C
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	// Enable ACK
	I2C1->CTLR1 |= I2C_CTLR1_ACK;


    // I2C1->CTLR1 &= ~I2C_CTLR1_PE;

	// // Enable Event and Error Interrupts
	// I2C1->CTLR2 |= I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN | I2C_CTLR2_ITBUFEN;
	// NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
	// NVIC_SetPriority(I2C1_EV_IRQn, 2 << 4);
	// NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
	// NVIC_SetPriority(I2C1_ER_IRQn, 2 << 4);

    // I2C1->CTLR2 = systemClock_Hz / 1000000;
    // I2C1->CKCFGR = systemClock_Hz / (i2cSpeed_Hz << 1);	// SystemClockHz / (100KHz * 2)
    // I2C1->OADDR1 = (self_addr << 1);

    // I2C1->CTLR1 |= I2C_CTLR1_PE;
    // I2C1->CTLR1 |= I2C_CTLR1_ACK;
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void) {
	uint16_t STAR1 = I2C1->STAR1;

	// Obtain and clear Bus error
	if (STAR1 & I2C_STAR1_BERR) {
		printf("I2C Bus Error\r\n");
		I2C1->STAR1 &= ~(I2C_STAR1_BERR);
	}

	// Obtain and clear Arbitration lost error
	if (STAR1 & I2C_STAR1_ARLO) {
		printf("I2C Arbitration Lost\r\n");
		I2C1->STAR1 &= ~(I2C_STAR1_ARLO);
	}

	// Obtain and clear Acknowledge failure error
	if (STAR1 & I2C_STAR1_AF) {
		printf("I2C Acknowledge Failure\r\n");
		I2C1->STAR1 &= ~(I2C_STAR1_AF);
	}
}


#define FLAG_Mask          ((uint32_t)0x00FFFFFF)
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                 ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED                 ((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */
#define  I2C_EVENT_MASTER_MODE_SELECT                      ((uint32_t)0x00030001)  /* BUSY, MSL and SB flag */

FlagStatus I2C_GetFlagStatus(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG) {
    FlagStatus    bitstatus = RESET;
    __IO uint32_t i2creg = 0, i2cxbase = 0;

    i2cxbase = (uint32_t)I2Cx;
    i2creg = I2C_FLAG >> 28;
    I2C_FLAG &= FLAG_Mask;

    if(i2creg != 0) {
        i2cxbase += 0x14;
    }
    else {
        I2C_FLAG = (uint32_t)(I2C_FLAG >> 16);
        i2cxbase += 0x18;
    }

    if(((*(__IO uint32_t *)i2cxbase) & I2C_FLAG) != (uint32_t)RESET) {
        bitstatus = SET;
    }
    else {
        bitstatus = RESET;
    }

    return bitstatus;
}

ErrorStatus I2C_CheckEventA(uint32_t I2C_EVENT) {
    uint32_t    lastevent = 0;
    uint32_t    flag1 = 0, flag2 = 0;
    ErrorStatus status = NoREADY;

    flag1 = I2C1->STAR1;
    flag2 = I2C1->STAR2;
    flag2 = flag2 << 16;

    lastevent = (flag1 | flag2) & FLAG_Mask;

    if((lastevent & I2C_EVENT) == I2C_EVENT) {
        status = READY;
    }
    else {
        status = NoREADY;
    }

    return status;
}

u8 I2C_CheckEvent(uint32_t I2C_EVENT) {
    uint32_t flag1 = I2C1->STAR1;
    uint32_t flag2 = I2C1->STAR2;
    flag2 = flag2 << 16;

    uint32_t lastevent = (flag1 | flag2) & FLAG_Mask;
    return (lastevent & I2C_EVENT) == I2C_EVENT;
}
