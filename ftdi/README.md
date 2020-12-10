# FTDI EEPROM programming

Disconnect all USB-to-serial devices but the Hayashi remote dongle.

Install the package `ftdi_eeprom` and execute the `flash.sh` script as root to
program the EEPROM memory of the dongle FT232H device. This will configure the
vendor and description strings to respectively "Ledger" and
"hayashi-light-remote".

Configuring the EEPROM is optional, but when programmed, the software is able
to recognize automatically which serial port corresponds to the dongle.
