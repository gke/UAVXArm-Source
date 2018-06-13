// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "UAVX.h"

#define I2C_DEFAULT_TIMEOUT 30000

#if (MAX_I2C_PORTS>0)

volatile I2CStateDef I2CState[MAX_I2C_PORTS] = { { 0 } };


void i2c_er_handler(uint8 CurrI2C) {
	// Original source unknown but modified from those on baseflight by TimeCop
	volatile uint32 SR1Register, SR2Register;
	const I2CPortDef * d;

	d = &I2CPorts[CurrI2C];

	SR1Register = d->I2C->SR1;
	if (SR1Register & 0x0f00) { //an error
		// I2C1error.error = ((SR1Register & 0x0f00) >> 8);        //save error
		// I2C1error.job = job;    //the task
	}
	/* If AF, BERR or ARLO, abandon the current job and commence new if there are jobs */
	if (SR1Register & 0x0700) {
		SR2Register = d->I2C->SR2; //read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
		I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable the RXNE/TXE interrupt - prevent the ISR tail-chaining onto the ER (hopefully)
		if (!(SR1Register & 0x0200) && !(d->I2C->CR1 & 0x0200)) { //if we don't have an ARLO error, ensure sending of a stop
			if (d->I2C->CR1 & 0x0100) { //We are currently trying to send a start, this is very bad as start,stop will hang the peripheral
				while (d->I2C->CR1 & 0x0100) {//wait for any start to finish sending
				}
				I2C_GenerateSTOP(d->I2C, ENABLE); //send stop to finalise bus transaction
				while (d->I2C->CR1 & 0x0200) {//wait for stop to finish sending
				}
				InitI2C(CurrI2C); //reset and configure the hardware
			} else {
				I2C_GenerateSTOP(d->I2C, ENABLE); //stop to free up the bus
				I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE); //Disable EVT and ERR interrupts while bus inactive
			}
		}
	}
	d->I2C->SR1 &= ~0x0f00; //reset all the error bits to clear the interrupt
	I2CState[CurrI2C].busy = false;
} // i2c_er_handler

void i2c_ev_handler(uint8 CurrI2C) {
	// Original source unknown but based on those in baseflight by TimeCop
	static int8 i; //index is signed -1==send the sub-address
	uint8 SReg_1; //read the status register here
	const I2CPortDef * d;

	d = &I2CPorts[CurrI2C];

	SReg_1 = d->I2C->SR1;

	if (SReg_1 & 0x0001) { //we just sent a start - EV5 in reference manual
		d->I2C->CR1 &= ~0x0800; //reset the POS bit so ACK/NACK applied to the current byte
		I2C_AcknowledgeConfig(d->I2C, ENABLE); //make sure ACK is on
		i = 0; //reset the index
		if (I2CState[CurrI2C].reading && (I2CState[CurrI2C].subaddress_sent
				|| (0xff == I2CState[CurrI2C].reg))) { //we have sent the sub-address
			I2CState[CurrI2C].subaddress_sent = true; //make sure this is set in case of no sub-address, so following code runs correctly
			if (I2CState[CurrI2C].bytes == 2)
				d->I2C->CR1 |= 0x0800; //set the POS bit so NACK applied to the final byte in the two byte read
			I2C_Send7bitAddress(d->I2C, I2CState[CurrI2C].addr,
					I2C_Direction_Receiver); //send the address and set hardware mode
		} else { //direction is Tx, or we haven't sent the sub and rep start
			I2C_Send7bitAddress(d->I2C, I2CState[CurrI2C].addr,
					I2C_Direction_Transmitter); //send the address and set hardware mode
			if (I2CState[CurrI2C].reg != 0xff) //0xff as sub-address means it will be ignored, in Tx or Rx mode
				i = -1; //send a sub-address
		}
	} else if (SReg_1 & 0x0002) { //we just sent the address - EV6 in ref manual
		//Read SR1,2 to clear ADDR
		volatile uint8 a;
		__DMB(); // memory fence to control hardware
		if (I2CState[CurrI2C].bytes == 1 && I2CState[CurrI2C].reading
				&& I2CState[CurrI2C].subaddress_sent) { //we are receiving 1 byte - EV6_3
			I2C_AcknowledgeConfig(d->I2C, DISABLE); //turn off ACK
			__DMB();
			a = d->I2C->SR2; //clear ADDR after ACK is turned off
			I2C_GenerateSTOP(d->I2C, ENABLE);
			I2CState[CurrI2C].final_stop = true;
			I2C_ITConfig(d->I2C, I2C_IT_BUF, ENABLE); //allow us to have an EV7
		} else { //EV6 and EV6_1
			a = d->I2C->SR2; //clear the ADDR here
			__DMB();
			if ((I2CState[CurrI2C].bytes == 2) && I2CState[CurrI2C].reading
					&& I2CState[CurrI2C].subaddress_sent) { //rx 2 bytes - EV6_1
				I2C_AcknowledgeConfig(d->I2C, DISABLE); //turn off ACK
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to fill
			} else if ((I2CState[CurrI2C].bytes == 3)
					&& I2CState[CurrI2C].reading
					&& I2CState[CurrI2C].subaddress_sent) //rx 3 bytes
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //make sure RXNE disabled so we get a BTF in two bytes time
			else
				//receiving greater than three bytes, sending sub-address, or transmitting
				I2C_ITConfig(d->I2C, I2C_IT_BUF, ENABLE);
		}
	} else if (SReg_1 & 0x004) { //Byte transfer finished - EV7_2, EV7_3 or EV8_2
		I2CState[CurrI2C].final_stop = true;
		if (I2CState[CurrI2C].reading && I2CState[CurrI2C].subaddress_sent) { //EV7_2, EV7_3
			if (I2CState[CurrI2C].bytes > 2) { //EV7_2
				I2C_AcknowledgeConfig(d->I2C, DISABLE); //turn off ACK
				I2CState[CurrI2C].read_p[i++] = I2C_ReceiveData(d->I2C); //read data N-2
				I2C_GenerateSTOP(d->I2C, ENABLE);
				I2CState[CurrI2C].final_stop = true; //required to fix hardware
				I2CState[CurrI2C].read_p[i++] = I2C_ReceiveData(d->I2C); //read data N-1
				I2C_ITConfig(d->I2C, I2C_IT_BUF, ENABLE); //enable TXE to allow the final EV7
			} else { //EV7_3
				if (I2CState[CurrI2C].final_stop)
					I2C_GenerateSTOP(d->I2C, ENABLE);
				else
					I2C_GenerateSTART(d->I2C, ENABLE); //repeated start
				I2CState[CurrI2C].read_p[i++] = I2C_ReceiveData(d->I2C); //read data N-1
				I2CState[CurrI2C].read_p[i++] = I2C_ReceiveData(d->I2C); //read data N
				i++; //to show job completed
			}
		} else { //EV8_2, which may be due to a sub-address sent or a write completion
			if (I2CState[CurrI2C].subaddress_sent
					|| (I2CState[CurrI2C].writing)) {
				if (I2CState[CurrI2C].final_stop)
					I2C_GenerateSTOP(d->I2C, ENABLE);
				else
					I2C_GenerateSTART(d->I2C, ENABLE); //repeated start
				i++; //to show that the job is complete
			} else { //send a sub-address
				I2C_GenerateSTART(d->I2C, ENABLE); //repeated Start
				I2CState[CurrI2C].subaddress_sent = true; //this is set back to zero upon completion of the current task
			}
		}
		//we must wait for the start to clear, otherwise we get constant BTF
		while (d->I2C->CR1 & 0x0100) {
		}
	} else if (SReg_1 & 0x0040) { //Byte received - EV7
		I2CState[CurrI2C].read_p[i++] = I2C_ReceiveData(d->I2C);
		if (I2CState[CurrI2C].bytes == (i + 3))
			I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush so we can get an EV7_2
		if (I2CState[CurrI2C].bytes == i) //We have completed a final EV7
			i++; //to show job is complete
	} else if (SReg_1 & 0x0080) { //Byte transmitted -EV8/EV8_1
		if (i != -1) { //we don't have a sub-address to send
			I2C_SendData(d->I2C, I2CState[CurrI2C].write_p[i++]);
			if (I2CState[CurrI2C].bytes == i) //we have sent all the data
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush
		} else {
			i++;
			I2C_SendData(d->I2C, I2CState[CurrI2C].reg); //send the sub-address
			if (I2CState[CurrI2C].reading || (I2CState[CurrI2C].bytes == 0)) //if receiving or sending 0 bytes, flush now
				I2C_ITConfig(d->I2C, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush
		}
	}
	if (i == I2CState[CurrI2C].bytes + 1) { //we have completed the current job
		//Completion Tasks go here
		//End of completion tasks
		I2CState[CurrI2C].subaddress_sent = false; //reset this here
		// d->I2C->CR1 &= ~0x0800;   //reset the POS bit so NACK applied to the current byte
		if (I2CState[CurrI2C].final_stop) //if there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
			I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE); //Disable EVT and ERR interrupts while bus inactive
		I2CState[CurrI2C].busy = false;
	}
} // i2c_ev_handler


boolean I2CReadBlock(uint8 i2cDev, uint8 id, uint8 reg, uint8 len, uint8* buf) {
	// Original source unknown but based on those in baseflight by TimeCop
	uint32 timeout = I2C_DEFAULT_TIMEOUT;
	idx CurrI2C;
	const I2CPortDef * d;

	CurrI2C = busDev[i2cDev].busNo;
	d = &I2CPorts[CurrI2C];

	I2CState[CurrI2C].addr = id;
	I2CState[CurrI2C].reg = reg;
	I2CState[CurrI2C].writing = false;
	I2CState[CurrI2C].reading = true;
	I2CState[CurrI2C].subaddress_sent = false;
	I2CState[CurrI2C].final_stop = false;
	I2CState[CurrI2C].read_p = buf;
	I2CState[CurrI2C].write_p = buf;
	I2CState[CurrI2C].bytes = len;
	I2CState[CurrI2C].busy = true;

	if (!(d->I2C->CR2 & I2C_IT_EVT)) { //if we are restarting the driver
		if (!(d->I2C->CR1 & 0x0100)) { // ensure sending a start
			while (d->I2C->CR1 & 0x0200) { //wait for any stop to finish sending
			}
			I2C_GenerateSTART(d->I2C, ENABLE); //send the start for the new job
		}
		I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE); //allow the interrupts to fire off again
	}

	while (I2CState[CurrI2C].busy && (--timeout > 0)) {
	}
	if (timeout == 0) {
		I2CState[CurrI2C].i2cErrors++;
		setStat(I2CFailS, I2CState[CurrI2C].i2cErrors);
		InitI2C(CurrI2C);
		return (false);
	}

	return (true);
} // I2CReadBlock

boolean I2CWriteBlock(uint8 i2cDev, uint8 id, uint8 reg, uint8 len_,
		uint8 *data) {
	// Original source unknown but based on those in baseflight by TimeCop
	idx i, CurrI2C;
	uint8 my_data[128]; // TODO: magic number
	uint32 timeout = I2C_DEFAULT_TIMEOUT;
	const I2CPortDef * d;

	if (len_ > 127)
		return (false);

	CurrI2C = busDev[i2cDev].busNo;
	d = &I2CPorts[CurrI2C];

	I2CState[CurrI2C].addr = id;
	I2CState[CurrI2C].reg = reg;
	I2CState[CurrI2C].subaddress_sent = false;
	I2CState[CurrI2C].final_stop = false;
	I2CState[CurrI2C].writing = true;
	I2CState[CurrI2C].reading = false;
	I2CState[CurrI2C].write_p = my_data;
	I2CState[CurrI2C].read_p = my_data;
	I2CState[CurrI2C].bytes = len_;
	I2CState[CurrI2C].busy = true;

	for (i = 0; i < len_; i++)
		my_data[i] = data[i];

	if (!(d->I2C->CR2 & I2C_IT_EVT)) { //if we are restarting the driver
		if (!(d->I2C->CR1 & 0x0100)) { // ensure sending a start
			while (d->I2C->CR1 & 0x0200) { //wait for any stop to finish sending
			}
			I2C_GenerateSTART(d->I2C, ENABLE); //send the start for the new job
		}
		I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE); //allow the interrupts to fire off again
	}

	while (I2CState[CurrI2C].busy && --timeout > 0) {
	}
	if (timeout == 0) {
		I2CState[CurrI2C].i2cErrors++;
		setStat(I2CFailS, I2CState[CurrI2C].i2cErrors);
		InitI2C(CurrI2C);
		return (false);
	}

	return (true);
} // I2CWriteBlock


boolean I2CResponse(uint8 i2cSel, uint8 d) { // returns true unless there is an I2C timeout????
	uint8 v;

	v = 77;
	return (I2CReadBlock(i2cSel, d, 0, 1, &v) && (v != 77));

} // response

#endif
