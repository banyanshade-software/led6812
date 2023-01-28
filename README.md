# led6812
Simple test driving **sk6812** leds from stm32/freeRtos using **SPI** with **DMA**

Code is highly inspired from https://github.com/innomatica/sk6812led (with many bug fixed),
specially the use of SPI/nibble to handle specific sk6812 serial port bit format

The interesting code is in Led/ledtask.c
