# relay_usbtmc
Two relay channels are controlled with an Adafruit QT PY SAMD21 board running USBTMC

I bought a five-pack of single channel AC/DC relays which can be controlled with a 3.3V level. I then glued 2 of them together and then glued an Adafruit QT PY microcontroller board to the top. The QT PY's STEMMA QT connector is used with a STEMMA QT cable to physically connect power, ground and control to the two relays. I modified the TINYUSB project's USBTMC device example to control the SDA and SCL pins of the QT PY as GPIOs. The relays are powered and controlled with the USB connection to the QT PY board through the STEMMA QT cable. The cool part of this new device is that it is a USBTMC device, which can be controlled and queried with simple SCPI commands. Another benefit of this approach is that the *IDN? command returns the SAMD21's unique identifier as the serial number of the device (which means that you can connect many of these homemade devices on a single host and keep track of all of them).

Here are the SCPI commands which can be used:

GPIO1:RELAY 1 # relay #1 on

GPIO1:RELAY 0 # relay #1 off

GPIO2:RELAY 1 # relay #2 on

GPIO2:RELAY 0 # relay #2 off

*RST # set both relays off

*IDN? # returns "QT PY Relay," with a 32bit unique identifier from the SAMD21

Here's my parts list:

https://www.adafruit.com/product/4600

https://www.adafruit.com/product/4210

https://www.amazon.com/Channel-Optocoupler-Isolated-Control-Arduino/dp/B07XGZSYJV

Feel free to test the UF2 on any QT PY SAMD21 board, you can observe the SDA and SCL pins and see that they are controlled by the SCPI commands as GPIOs.

A simpler approach could be to just purchase a USB powered relay which has serial control of the relays. 1, 2 and 4 channel boards are sold on Amazon. I have used these boards and they work just as well as the USBTMC version. The downside is that the operating system determines the COM or TTY device name, and they can not be queried to check their presence or status.

I have used this board from Amazon:
https://www.amazon.com/NOYITO-2-Channel-Module-Control-Intelligent/dp/B081RM7PMY


