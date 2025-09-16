# SPI chained network

## Protocol

This protocol is built using 2 separate SPI peripherals - one in Master mode and one in Slave mode. Both are running in Full-duplex mode. The master represents downstream segment of the chain and slav - upstream. Slave doesn't have control over how many bytes it will send during each transmission, it will send anything it has in the buffer until upstream node's master sends anything to it. Master of the current node, on the other hand, can send any amount of data it wants, and it will receive the same amount of bytes in return. So in both cases both in and out buffers are being fulled/emptied at the same rate. And if all nodes on the chain are working according to the same standard, we also can expect that number of bytes in each transmission will be a multiple of the chosen *packet size*.

