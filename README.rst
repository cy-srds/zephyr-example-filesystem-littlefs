.. _littlefs-filesystem:

Littlefs Filesystem
######

Overview
********

This code example uses the `Arm® Cortex®-M4 (CM4) CPU of PSoC™ 6 MCU to execute a littlefs filesystem task`.

At device reset, the default Cortex-M0+ (CM0+) application enables the CM4 CPU and configures the CM0+ CPU to go to sleep.
The CM4 CPU mounts the filesystem on the storage device, reads a 32-bit value from a file, increments the value, writes the
value back to the file, and finally prints the value to the UART terminal. You will see the value incrementing after every 
reset. The storage device can be either an SD card or a QSPI NOR flash. This project uses the `mtb-littlefs Middleware library 
<https://github.com/Infineon/mtb-littlefs>` that implements the littlefs-compatible block device drivers for different storage
devices such as SD card and NOR flash.

Requirements
************

The board hardware must have storage device SD card or a QSPI NOR flash.
This sample is intended to be used with `CY8CPROTO-062-4343W PSoC™ 6 Wi-Fi Bluetooth® prototyping kit <https://www.infineon.com/CY8CPROTO-062-4343W>`.

Refer to the Zephyr `Getting Started Guide <https://docs.zephyrproject.org/latest/develop/getting_started/index.html>`_ for more details on how to setup and run applications in Zephyr.

Building and Running
********************

In this example we will build it for the cy8cproto_062_4343w board:

.. zephyr-app-commands::
   :zephyr-app: zephyr_example_filesystem_littlefs
   :board: cy8cproto_062_4343w
   :goals: build
   :compact:

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. After programming, the application starts automatically. Confirm that  "Littlefs File System on SD Card and QSPI NOR Flash" is displayed on the UART terminal. 

When the application is run for the first time, mounting the filesystem might result in error in which case the memory is formatted and then the filesystem is mounted.

4. Now, press the reset button on the kit and observe that the boot_count is incremented.

5. If you press the User button on the kit, the memory will be formatted and the boot_count will be reset. You can press the reset button to increment the boot_count again.

6. By default, the application uses the QSPI NOR flash as the storage device. To change the storage device to the SD card, set the value of the macro STORAGE_DEVICE_SD_CARD in main.c to 1 and then build the application and program the kit.
