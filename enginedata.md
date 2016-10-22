#Engine Data Protocol

## Overview

To receive data from the gas engine, an Arduino Micro reads data and sends it at a given interval to the Supervisory controller, the Arduino Mega. The data is sent to the Mega over a UART serial line with at a pre-determined baud rate, no parity, two stop bits.

## Protocol

The data-link layer uses the standard HDLC protocol for framing data. The [Arduhdlc HDLC Library](https://github.com/jarkko-hautakorpi/Arduhdlc) is used to provide the functionality

## Data

The data send from the Mirco consists of the fields below. The entire message is 4 bytes long, transmitted in network (big-endian) byte order.

- RPM: uint16, [0-65535]
  - Scale: 1
  - Offset: 0
- Fuel consumption: uint16, [0-65535]
  - Scale: 1
  - Offset: 0