[![Build Status](https://travis-ci.org/RFD-FHEM/SIGNALESP.svg?branch=dev-cc1101)](https://travis-ci.org/RFD-FHEM/SIGNALESP)
# SIGNALESP uC v3.3 (development version)

System to receive digital signals and provide them to other systems pro demodulatiob. Currently tested with 433 MHZ, but not limited to that frequency or media.

### Imporant note

SIGNALESP is currently not really working for production
D4 on NodeMCU is GPIO2. 
Serial Port operates with 2500000 Baud
Free Ram can't be received via command
IT Send commands are not available

### Getting started


Just clone the repo and open the project file with Visual Studio / Visual Micro. (currently this works only for windows)
In case you choose download ZIP, the subprojects used by this project are not downloaded in this zip. The preferred way is really to clone via git.
In case, you still want to download via ZIP, you also have to download
https://github.com/tzapu/WiFiManager/tree/master and place the content into 
`SIGNALESP\src\_micro-api\libraries\WIFIManager`

Compile it and have fun.
If you are using the Arduino IDE, you have to copy all the libs into your sketch folder.


### Using SIGNALDuino in FHEM

If you want to use the SIGNALDuino with FHEM, you can use it directly from FHEM. No neet to compile any sourcode.
You find more Information here:
http://www.fhemwiki.de/wiki/SIGNALDuino


### Tested microcontrollers

* ESP8266 (NodeMCU)

### Signal from my device ist not detected

We have a pattern detection engine, that detect serval signal types. May not all, but most of them.

Uncomment #define debugdetect in libs/remotesensor/patterdecoder.h
Search for some output which describes a pattern with serval bits received.
If you find something, open an issue and provide as much as possible informations with it.


### You found a bug

First, sorry. This software is not perfect.
1. Open a issue
-With helpful title - use descriptive keywords in the title and body so others can find your bug (avoiding duplicates).
- Which branch, what microcontroller, what setup
- Steps to reproduce the problem, with actual vs. expected results
- If you find a bug in our code, post the files and the lines. 

### Contributing

1. Open one ore more issue for your development.
2. Ask to be added to our repository or just fork it.
3. Make your modifications and test them.
4. Create a branch (git checkout -b my_branch)
5. Commit your changes (git commit -am "<some description>")
6 .Push to a developer branch (git push dev-<xyz >my_branch)
7. Open a Pull Request, put some useful informations there, what your extension does and why we should add it, reference to the open issues which are fixed whith this pull requet.


