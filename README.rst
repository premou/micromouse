Welcome to the micro-mouse robots project
=========================================

.. topic:: Abstract

   On this page you will learn how we designed our robots.

   * How does the high level C code communicate with the micro-controller
     and other hardware components ?
   * How do we manage to bring the micro-mouses at the right speed and in the right direction
     at each step all along the way ?
   * How do we manage to detect obstacles ?

   Those are only a few examples of all the questions that will be addressed here :-)

.. contents::
   :local:

Overview
--------

motors
------

Part 1
^^^^^^

Part 2
^^^^^^

Bluetooth
---------
# download the driver for usb/serial adapter : CP210x USB to UART Bridge VCP Drivers
.. seealso::

	https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers

# usefull documentation 
.. seealso::
   
   https://www.dfrobot.com/wiki/index.php/DF-BluetoothV3_Bluetooth_module_(SKU:TEL0026)
   
# full AT commands
.. seealso::
   
   http://image.dfrobot.com/image/data/TEL0026/TEL0026_Datasheet.pdf

# How to configure the bluetooth module: 
# Default serial software configuration is : baudrate 38400 + echo local + line mode + AT swith ON (see bluetooth module swith)  

.. code-block:: bash

   # Reset module to factory settings

   > AT+ORG
   OK
   
   # Config robot side

   > AT
   OK
   
   > AT+PSWD?
   +PSWD:1234
 
   > AT+NAME=ROBOT_FRANCOIS
   OK
   
   > AT+UART=115200,0,0
   OK

   > AT+ROLE=0
   OK


.. code-block:: bash

   # Config PC side

   > AT
   OK
   
   > AT+PSWD?
   +PSWD:1234
 
   > AT+NAME=PC_FRANCOIS
   OK
   
   > AT+UART=115200,0,0
   OK

   > AT+ROLE=1
   OK

Gyroscope
---------

Infrared LEDS and photo-sensitive transistors
---------------------------------------------

Buzzer
------

Battery
-------
