# Soft MCP23017 for Arduino
[![License](https://img.shields.io/badge/license-MIT%20License-blue.svg)](http://doge.mit-license.org)

This library provides full control over the Microchip's [MCP23017](https://www.microchip.com/wwwproducts/en/MCP23017), including interrupt support, via **software** i2c.

The software i2c implementation used is *FastI2cMaster*, contained in the [DigitalIO library by greiman](https://github.com/greiman/DigitalIO), which achieves 400 kHz i2c communication on any pins of a 16 MHz Arduino (!).

This library is based on v2.0.0 of [blemasle's library](https://github.com/blemasle/arduino-mcp23017).

## Features
 * Individual pins read & write
 * Ports read & write
 * Registers read & write
 * Full interrupt support

## Usage
Unlike most Arduino library, no default instance is created when the library is included. It's up to you to create one with the appropriate chip I2C pins and address.

```cpp
#include <MCP23017Fast.h>

const uint8_t SCL_PIN = A4;
const uint8_t SDA_PIN = A5;

MCP23017<SCL_PIN, SDA_PIN> mcp(0x20);
```

See the included examples for further usage instructions.
