Motion Lights project with interconnected nodes of motion sensor. Here, two nodes each with it's own
Atmega16a mcu, NRF24L01 rf module and PIR sensor are used. Data among the nodes are exchanged in Json format. Both nodes
can independently detect motions and turn lights on as well as send message to other node to turn on lights in advance.
There is a timeout in seconds for the lights to turn off in case no motion is detected further.

To learn more about json decoder for AVR, visit https://github.com/MicrochipTech/json_decoder
