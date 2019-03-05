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
# download the driver (to find)
.. seealso::
   
   https://www.dfrobot.com/wiki/index.php/DF-BluetoothV3_Bluetooth_module_(SKU:TEL0026)

# baudrate:38400 + echo local + line mode + AT swith ON 


.. code-block:: bash

   # Config robot

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

   # Config PC

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
