# CSSE4011 Prac 2
## James King 47443732

All design tasks and commands have been implemented as per the spec. In particular, the commands `gcurec, gcugraph, gcunumeric, gcumeter, blecon, blescan` have been implemented.

## Folder structure:
Application located in prac2/mycode/apps/prac2_ahu and Thingy52 application located in prac2/mycode/apps/prac2_thingy52

## References:

## Source Instructions:
Within the prac2 directory, compile with:
`west build -b nrf52840dk_nrf52840 mycode/apps/prac2_ahu --pristine`
Flash with:
`west flash --recover`

Then attach the thingy52 to the nrf52840 debug port. Compile with:
`west build -b thingy52_nrf52832 mycode/apps/prac2_thingy52 --pristine`
Flash with:
`west flash --recover`

## User Instructions:
GCU UART connection uses uart1 on pins P1.1 (RX) and P1.2 (TX).

Be sure to connect GCU and AHU to common ground.

Shell is output through uart0 USB connection.