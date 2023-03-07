CAPSENSE Buttons and Slider
######

Overview
********

This code example features a 5-segment CAPSENSE™ slider and two CAPSENSE™ buttons.
Button 0 turns the LED ON, button 1 turns the LED OFF, and the slider controls the brightness of the LED.
The code example also demonstrates monitoring CAPSENSE™ data using the CAPSENSE™ tuner GUI tool.
This project uses the `CAPSENSE™ Middleware Library <https://github.com/Infineon/capsense>`_.

Requirements
************

The board hardware must have CAPSENSE buttons and leanier slider connected via a GPIO pin.
This sample is intended to be used with `CY8CPROTO-062-4343W PSoC™ 6 Wi-Fi Bluetooth® prototyping kit <https://www.cypress.com/CY8CPROTO-062-4343W>`_.

Refer to the Zephyr `Getting Started Guide <https://docs.zephyrproject.org/latest/develop/getting_started/index.html>`_ for more details on how to setup and run applications in Zephyr.

Building and Running
********************

In this example we will build it for the cy8cproto_062_4343w board:

.. zephyr-app-commands::
   :zephyr-app: zephyr_example_capsense_buttons_slider
   :board: cy8cproto_062_4343w
   :goals: build
   :compact:

1. After programming, the application starts automatically. Confirm that the user LED is glowing.

2. To test the application, touch CAPSENSE™ button 1 (BTN1) to turn the LED OFF, touch CAPSENSE™ Button 0 (BTN0) to turn the LED ON, and touch the slider in different positions to change the brightness.

3. You can also monitor the CAPSENSE™ data using the CAPSENSE™ Tuner application.
