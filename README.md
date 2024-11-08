# PicoA10400
Flashcart for Atari 2600 and Atari 7800 based on Pico "Purple" clone, easy to build and cheap.
This is a "double-face" flashcart, it could be used for both Atari 2600 and Atari 7800, simply rotating it and inserting the cart in different Atari!!
It doesn't support all bank-switching schemas, but enough to enjoy your A2600 / A7800 with a single flash-carts!!

A special thanks to other opensource project for Atari multicarts from which i got a lot of info, ideas and also code:
https://github.com/robinhedwards/UnoCart-2600

https://github.com/karrika/Otaku-flash


![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_06.jpg)

**WARNING!** "purple" Pico has not the same pinout of original Raspberry "green" ones, you MUST use the clone or you may damage your hardware.
Also note that the battery used is a RECHARGEABLE LIR2032, if you want to use a NON reachargeable battery you must add a diode in circuit!!!

**NOTE** Please look at picture for soldering side of the components, or your shell won't close!! they are different from the pcb mask!!!

![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_01.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_02.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_03.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_04.jpg)

![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_08.jpg)


Gerbers file are provided for the PCB, add you pico clone, and flash the firmware ".uf2" in the Pico by connecting it while pressing button on Pico and drop it in the opened windows on PC.
After flashed with firmware, and every time you have to change your ROMS repository, you can simply connect the Pico to PC and drag&drop "BIN" files  into.

**NOTE 2** Due to different timing of PicoA10400 and the Atari consoles, that can't be resetted, the flashcart MUST BE POWERED ON (with POWER SWITCH ON CART) BEFORE POWERING THE CONSOLE!!! Also, some games and ALL A7800 GAMES NEEDS THAT THE CONSOLE IS POWERED OFF THEN POWERED ON TO START!!!!

Even if the diode should protect your console, **DO NOT CONNECT PICO WHILE INSERTED IN A POWERED ON CONSOLE!**

