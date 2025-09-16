# SPI chained network
Discovered quirks/limitations:
- Though both SPIs can use DMA, that only works in Master mode. In slave mode I found the best way is to read bytes in a loop inside EXTI interrupt on CS pin.
- At least with my setup MISO GPIO of Slave SPI needs to be in 2MHz mode, or it will trash both incoming and outgoing transmissions while sending.
- 2MHz GPIO mode works for all pins on both SPIs even when working at higher baudrate.
- Processor couldn't keep up sending data in time at speeds > 18MHz.
- Slave SPI needs a delay on Master side between pulling CS low and starting a transmission to be able to write data to DATAR register in time.
- I'm using HW CS pink on Slave side and toggling CS in software on Master side. This makes the most sense.
- You can modify the code to use 16 bit transmission mode. This will work at the same speed with double data rate. But then you'd need to pack uint8_t to uint16_t and vice versa.
- In this setup SPI works in Full Duplex mode. This means that it's transmitting and receiving at the same time synchronously. Pro is that it fast and *convenient*. Con is that it's *inconvenient* because if you want to receive something on Master from Slave you need to send something, all zeroes for example. And if you want to send something from Slave to Master, you need for Master to initiate the transmission first. There is a way to make signalling from Slave back to Master to trigger the transmission, but for now I think polling periodically is the best solution.
- Interrupts are strange, I don't think I was able to make nesting priority work, so I decided to use DMA Transmission Complete interrupt only to disable DMA Out channel and set flag. And then do all the work in main loop. Slave SPI has to do everything in interrupt, because it is very time sensitive procedure

## Protocol

This protocol is built using 2 separate SPI peripherals - one in Master mode and one in Slave mode. Both are running in Full-duplex mode. The master represents downstream segment of the chain and slave - upstream. Slave doesn't have control over how many bytes it will send during each transmission, it will send anything it has in the buffer until upstream node's master sends anything to it. Master of the current node, on the other hand, can send any amount of data it wants, and it will receive the same amount of bytes in return. So in both cases both in and out buffers are being fulled/emptied at the same rate. And if all nodes on the chain are working according to the same standard, we also can expect that number of bytes in each transmission will be a multiple of the chosen *packet size*.

### Message structure

Packet = 10 bytes:

| 0             | 1           | 2-3                                         | 4-9               |
|       -       | -           | -                                           | -                 |
| [receiver ID] | [sender ID] | [message number + ACK/NAK bit + reply flag] | [6 bytes of data] |

Each message has unique ID number so nodes can know what command is data replies to. Also you can make messages of any length by sending multiple packets with the same message number.
Each message has to have sender ID or it will be discarded. Receiver ID can be left blank for broadcast purposes, or when you don't know an ID of nearby nodes, for example during enumeration.
If incoming command can't be processed at the moment (node is processing another command for example) NAK reply should be send. Sender will repeat the command in new message later.
Reply flag is used to indicate that the message is a reply to previously sent message. If receiver has a pending command with this message ID, then it processes the data, otherwise message is discarded.
If received packet has receiver ID that is different from the current node ID, then it copied to OUT buffer of another SPI peripheral to be sent during next transmission. This way messages can be sent from each node to any other node in both up and downstream directions.

