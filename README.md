# learn-tiva-c-alarm

Coded using Keil uVision V5.34.0.0 using a LM4F120H5QR from Texas Instruments. This does not have a PWM module, hence I have used the GPTM to generate PWM where required. However, I have used the Tiva C (TM4C123GH6PM ) libraries for coding (GPIO_PORTD_DATA_R for LM4F becomes a more concise GPIOD->DATA for TM4C). I have not used the libraries because I want to learn how to read and follow the LM4F/TM4C datasheets.

I have a timer on my kitchen oven with two buttons; I have tried to emulate that using UART0 for the display and the two buttons on the LM4F. The components are from the Elegoo Super Starter kit. The servo is not required but I added it for further learning. Refer to main.c for details. 

I have referred to:
https://www.youtube.com/watch?v=fUac_C1aZP0&list=RDCMUCMOgTxgkrWUZ4HUtc-4JwkA&start_radio=1&t=1s
https://microcontrollerslab.com/category/tiva-launchpad/
http://www.airsupplylab.com/ti-tiva-c-serial.html
