09.03.2025 test application description:
- SPI Transmits its txbuffer evey second with a timer interrrupt and increases txbuffer value every iteration. LD3 led toggles for every succesful transmission.
  
- TCP and UDP server apllications append a string to the client message and send it back. They also control the LD2 led with commands form client. (LT = LED TOGGLE, LS = LED SET, LR = LED RESET)
  
- UDP Client periodicaly sends the number of server messages recieved during runtime using a timer interrupt.
