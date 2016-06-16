# Arduino targets of Aseba
This repository contains the Arduino targets of Aseba.
The "common" directory contains the common parts to all Arduino targets, and the other directories contain the target-specific parts.

## Elisa-3
The Elisa-3 robot is based on the ATMega2560 microcontroller, for more information refer to the wiki page http://www.gctronic.com/doc/index.php/Elisa-3 .
At the moment there is no separation between Elisa-3 specific code and common code related to Arduino targets.

### Building
1. Clone the repository: git clone --recursive https://github.com/gctronic/aseba-targets-arduino.git
2. The elisa3 target is an Atmel Studio 7 project so the easiest way of rebuilding the project is installing this IDE
3. Open the project file "elisa3-aseba.atsln" and build



