#pragma once

#include <Arduino.h>
#include <DigitalIO.h>

#define _MCP23017_INTERRUPT_SUPPORT_ ///< Enables support for MCP23017 interrupts.

enum class MCP23017Port : uint8_t
{
	A = 0,
	B = 1
};

/**
 * Controls if the two interrupt pins mirror each other.
 * See "3.6 Interrupt Logic".
 */
enum class MCP23017InterruptMode : uint8_t
{
	Separated = 0,	///< Interrupt pins are kept independent
	Or = 0b01000000	///< Interrupt pins are mirrored
};

/**
 * Registers addresses.
 * The library use addresses for IOCON.BANK = 0.
 * See "3.2.1 Byte mode and Sequential mode".
 */
enum class MCP23017Register : uint8_t
{
	IODIR_A		= 0x00, 		///< Controls the direction of the data I/O for port A.
	IODIR_B		= 0x01,			///< Controls the direction of the data I/O for port B.
	IPOL_A		= 0x02,			///< Configures the polarity on the corresponding GPIO_ port bits for port A.
	IPOL_B		= 0x03,			///< Configures the polarity on the corresponding GPIO_ port bits for port B.
	GPINTEN_A	= 0x04,			///< Controls the interrupt-on-change for each pin of port A.
	GPINTEN_B	= 0x05,			///< Controls the interrupt-on-change for each pin of port B.
	DEFVAL_A	= 0x06,			///< Controls the default comparaison value for interrupt-on-change for port A.
	DEFVAL_B	= 0x07,			///< Controls the default comparaison value for interrupt-on-change for port B.
	INTCON_A	= 0x08,			///< Controls how the associated pin value is compared for the interrupt-on-change for port A.
	INTCON_B	= 0x09,			///< Controls how the associated pin value is compared for the interrupt-on-change for port B.
	IOCON		= 0x0A,			///< Controls the device.
	GPPU_A		= 0x0C,			///< Controls the pull-up resistors for the port A pins.
	GPPU_B		= 0x0D,			///< Controls the pull-up resistors for the port B pins.
	INTF_A		= 0x0E,			///< Reflects the interrupt condition on the port A pins.
	INTF_B		= 0x0F,			///< Reflects the interrupt condition on the port B pins.
	INTCAP_A	= 0x10,			///< Captures the port A value at the time the interrupt occured.
	INTCAP_B	= 0x11,			///< Captures the port B value at the time the interrupt occured.
	GPIO_A		= 0x12,			///< Reflects the value on the port A.
	GPIO_B		= 0x13,			///< Reflects the value on the port B.
	OLAT_A		= 0x14,			///< Provides access to the port A output latches.
	OLAT_B		= 0x15,			///< Provides access to the port B output latches.
};

inline MCP23017Register operator+(MCP23017Register a, MCP23017Port b) {
	return static_cast<MCP23017Register>(static_cast<uint8_t>(a) + static_cast<uint8_t>(b));
};

template<uint8_t SCL_PIN, uint8_t SDA_PIN>
class MCP23017
{
private:
	FastI2cMaster<SCL_PIN, SDA_PIN> i2c;
	uint8_t _deviceAddr;
public:
	/**
	 * Instantiates a new instance to interact with a MCP23017 at the specified address.
	 */
	MCP23017(uint8_t address) {
		_deviceAddr = address;
	}
	
	~MCP23017() {}
	
#ifdef _DEBUG
	void debug();
#endif
	/**
	 * Initializes the chip with the default configuration.
	 * Enables Byte mode (IOCON.BANK = 0 and IOCON.SEQOP = 1).
	 * Enables pull-up resistors for all pins. This will only be effective for input pins.
	 * 
	 * See "3.2.1 Byte mode and Sequential mode".
	 */
	void init()
	{
		// Init soft I2C
		i2c.begin();
		
		//BANK = 	0 : sequential register addresses
		//MIRROR = 	0 : use configureInterrupt 
		//SEQOP = 	1 : sequential operation disabled, address pointer does not increment
		//DISSLW = 	0 : slew rate enabled
		//HAEN = 	0 : hardware address pin is always enabled on 23017
		//ODR = 	0 : open drain output
		//INTPOL = 	0 : interrupt active low
		writeRegister(MCP23017Register::IOCON, 0b00100000);

		//enable all pull up resistors (will be effective for input pins only)
		writeRegister(MCP23017Register::GPPU_A, 0xFF, 0xFF);
	}
	
	/**
	 * Controls the pins direction on a whole port at once.
	 * 
	 * 1 = Pin is configured as an input.
	 * 0 = Pin is configured as an output.
	 * 
	 * See "3.5.1 I/O Direction register".
	 */
	void portMode(MCP23017Port port, uint8_t directions, uint8_t pullups = 0xFF, uint8_t inverted = 0x00)
	{
		writeRegister(MCP23017Register::IODIR_A + port, directions);
		writeRegister(MCP23017Register::GPPU_A + port, pullups);
		writeRegister(MCP23017Register::IPOL_A + port, inverted);
	}
	
	/**
	 * Controls a single pin direction. 
	 * Pin 0-7 for port A, 8-15 fo port B.
	 * 
	 * 1 = Pin is configured as an input.
	 * 0 = Pin is configured as an output.
	 *
	 * See "3.5.1 I/O Direction register".
	 * 
	 * Beware!  
	 * On Arduino platform, INPUT = 0, OUTPUT = 1, which is the inverse
	 * of the MCP23017 definition where a pin is an input if its IODIR bit is set to 1.
	 * This library pinMode function behaves like Arduino's standard pinMode for consistency.
	 * [ OUTPUT | INPUT | INPUT_PULLUP ]
	 */
	void pinMode(uint8_t pin, uint8_t mode, bool inverted = false)
	{
		MCP23017Register iodirreg = MCP23017Register::IODIR_A;
		MCP23017Register pullupreg = MCP23017Register::GPPU_A;
		MCP23017Register polreg = MCP23017Register::IPOL_A;
		uint8_t iodir, pol, pull;

		if(pin > 7)
		{
			iodirreg = MCP23017Register::IODIR_B;
			pullupreg = MCP23017Register::GPPU_B;
			polreg = MCP23017Register::IPOL_B;
			pin -= 8;
		}

		iodir = readRegister(iodirreg);
		if(mode == INPUT || mode == INPUT_PULLUP) bitSet(iodir, pin);
		else bitClear(iodir, pin);

		pull = readRegister(pullupreg);
		if(mode == INPUT_PULLUP) bitSet(pull, pin);
		else bitClear(pull, pin);

		pol = readRegister(polreg);
		if(inverted) bitSet(pol, pin);
		else bitClear(pol, pin);

		writeRegister(iodirreg, iodir);
		writeRegister(pullupreg, pull);
		writeRegister(polreg, pol);
	}

	/**
	 * Writes a single pin state.
	 * Pin 0-7 for port A, 8-15 for port B.
	 * 
	 * 1 = Logic-high
	 * 0 = Logic-low
	 * 
	 * See "3.5.10 Port register".
	 */
	void digitalWrite(uint8_t pin, uint8_t state)
	{
		MCP23017Register gpioreg = MCP23017Register::GPIO_A;
		uint8_t gpio;
		if(pin > 7)
		{
			gpioreg = MCP23017Register::GPIO_B;
			pin -= 8;
		}

		gpio = readRegister(gpioreg);
		if(state == HIGH) bitSet(gpio, pin);
		else bitClear(gpio, pin);

		writeRegister(gpioreg, gpio);
	}
	
	/**
	 * Reads a single pin state.
	 * Pin 0-7 for port A, 8-15 for port B.
	 * 
	 * 1 = Logic-high
	 * 0 = Logic-low
	 * 
	 * See "3.5.10 Port register".
	 */ 
	uint8_t digitalRead(uint8_t pin)
	{
		MCP23017Register gpioreg = MCP23017Register::GPIO_A;
		uint8_t gpio;
		if(pin > 7)
		{
			gpioreg = MCP23017Register::GPIO_B;
			pin -=8;
		}

		gpio = readRegister(gpioreg);
		if(bitRead(gpio, pin)) return HIGH;
		return LOW;
	}

	/**
	 * Writes pins state to a whole port.
	 * 
	 * 1 = Logic-high
	 * 0 = Logic-low
	 * 
	 * See "3.5.10 Port register".
	 */
	void writePort(MCP23017Port port, uint8_t value)
	{
		writeRegister(MCP23017Register::GPIO_A + port, value);
	}
	
	/**
	 * Writes pins state to both ports.
	 * 
	 * 1 = Logic-high
	 * 0 = Logic-low
	 * 
	 * See "3.5.10 Port register".
	 */
	void write(uint16_t value)
	{
		writeRegister(MCP23017Register::GPIO_A, lowByte(value), highByte(value));
	}

	/**
	 * Reads pins state for a whole port.
	 * 
	 * 1 = Logic-high
	 * 0 = Logic-low
	 * 
	 * See "3.5.10 Port register".
	 */
	uint8_t readPort(MCP23017Port port)
	{
		return readRegister(MCP23017Register::GPIO_A + port);
	}
	
	/**
	 * Reads pins state for both ports. 
	 * 
	 * 1 = Logic-high
	 * 0 = Logic-low
	 * 
	 * See "3.5.10 Port register".
	 */
	uint16_t read()
	{
		uint8_t a = readPort(MCP23017Port::A);
		uint8_t b = readPort(MCP23017Port::B);

		return a | b << 8;
	}

	/**
	 * Writes a single register value.
	 */
	void writeRegister(MCP23017Register reg, uint8_t value)
	{
		// Write address and continue transfer for write of data.
		if (i2c.transfer (_deviceAddr | I2C_WRITE, &reg, 1, I2C_CONTINUE)) {
			// Write data.
			i2c.transferContinue (&value, 1);
		}
	}
	
	/**
	 * Writes values to a register pair.
	 * 
	 * For portA and portB variable to effectively match the desired port,
	 * you have to supply a portA register address to reg. Otherwise, values
	 * will be reversed due to the way the MCP23017 works in Byte mode.
	 */
	void writeRegister(MCP23017Register reg, uint8_t portA, uint8_t portB)
	{
		// Write address and continue transfer for write of data.
		if (i2c.transfer (_deviceAddr | I2C_WRITE, &reg, 1, I2C_CONTINUE)) {
			// Write data.
			i2c.transferContinue (&portA, 1, I2C_CONTINUE);
			i2c.transferContinue (&portB, 1);
		}
	}
	
	/**
	 * Reads a single register value.
	 */
	uint8_t readRegister(MCP23017Register reg)
	{
		uint8_t buf = 0xFF;

		// Send address of data.
		if (i2c.transfer (_deviceAddr | I2C_WRITE, &reg, 1, I2C_CONTINUE)) {			// We must send a Restart condition
			// Read data.
			i2c.transfer (_deviceAddr | I2C_READ, &buf, 1);	
		}
		
		return buf;
	}
	
	/**
	 * Reads the values from a register pair.
	 * 
	 * For portA and portB variable to effectively match the desired port,
	 * you have to supply a portA register address to reg. Otherwise, values
	 * will be reversed due to the way the MCP23017 works in Byte mode.
	 */
	void readRegister(MCP23017Register reg, uint8_t& portA, uint8_t& portB)
	{
		// Send address of data.
		if (i2c.transfer (_deviceAddr | I2C_WRITE, &reg, 1)) {
			// Read data.
			//~ uint8_t buf;
			i2c.transfer (_deviceAddr | I2C_READ, &portA, 1, I2C_CONTINUE);
			i2c.transferContinue (&portB, 1);
		}
	}

#ifdef _MCP23017_INTERRUPT_SUPPORT_

	/**
	 * Controls how the interrupt pins act with each other.
	 * If intMode is SEPARATED, interrupt conditions on a port will cause its respective INT pin to active.
	 * If intMode is OR, interrupt pins are OR'ed so an interrupt on one of the port will cause both pints to active.
	 * 
	 * Controls the IOCON.MIRROR bit. 
	 * See "3.5.6 Configuration register".
	 */
	void interruptMode(MCP23017InterruptMode intMode)
	{
		uint8_t iocon = readRegister(MCP23017Register::IOCON);
		if(intMode == MCP23017InterruptMode::Or) iocon |= static_cast<uint8_t>(MCP23017InterruptMode::Or);
		else iocon &= ~(static_cast<uint8_t>(MCP23017InterruptMode::Or));

		writeRegister(MCP23017Register::IOCON, iocon);
	}
	
	/**
	 * Configures interrupt registers using an Arduino-like API.
	 * mode can be one of CHANGE, FALLING or RISING.
	 */
	void interrupt(MCP23017Port port, uint8_t mode)
	{
		MCP23017Register defvalreg = MCP23017Register::DEFVAL_A + port;
		MCP23017Register intconreg = MCP23017Register::INTCON_A + port;

		//enable interrupt for port
		writeRegister(MCP23017Register::GPINTEN_A + port, 0xFF);
		switch(mode)
		{
		case CHANGE:
			//interrupt on change
			writeRegister(intconreg, 0);
			break;
		case FALLING:
			//interrupt falling : compared against defval, 0xff
			writeRegister(intconreg, 0xFF);
			writeRegister(defvalreg, 0xFF);
			break;
		case RISING:
			//interrupt rising : compared against defval, 0x00
			writeRegister(intconreg, 0xFF);
			writeRegister(defvalreg, 0x00);
			break;
		}
	}
	
	/**
	 * Disable interrupts for the specified port.
	 */
	void disableInterrupt(MCP23017Port port)
	{
		writeRegister(MCP23017Register::GPINTEN_A + port, 0x00);
	}
	
	/**
	 * Reads which pin caused the interrupt.
	 */
	void interruptedBy(uint8_t& portA, uint8_t& portB)
	{
		readRegister(MCP23017Register::INTF_A, portA, portB);
	}
	
	/**
	 * Clears interrupts on both ports.
	 */
	void clearInterrupts()
	{
		uint8_t a, b;
		clearInterrupts(a, b);
	}
	
	/**
	 * Clear interrupts on both ports. Returns port values at the time the interrupt occured.
	 */
	void clearInterrupts(uint8_t& portA, uint8_t& portB)
	{
		readRegister(MCP23017Register::INTCAP_A, portA, portB);
	}

#endif
};
