.. _rz_g3s:

Renesas RZ G3S
##############

Overview
********

Hardware
********
The RZ G3S includes:

* 1.1 GHz Arm Cortex-A55 MPCore
* Two 250 MHz Arm Cortex-M33 cores (One Cortex®-M33 has FPU function)
* 1-Mbyte RAM
* IEEE 1588 PTP compliant Ethernet MAC controller
* USB 2.0 host/function module
* SD/MMC host interface
* Octa-Flash/Octa-RAM interface
* CAN interface
* PCI Express Gen 2.0 interface (option)

Connections and IOs
===================

RZ G3S board
------------

Vies of the EVK board:

.. figure:: img/rzg3s.jpg
   :align: center

Supported Features
==================
The Renesas ``rz_g3s`` board configuration supports the following
hardware features:

+-----------+------------------------------+--------------------------------+
| Interface | Driver/components            | Support level                  |
+===========+==============================+================================+
| PINCTRL   | pinctrl                      |                                |
+-----------+------------------------------+--------------------------------+
| CLOCK     | clock_control                |                                |
+-----------+------------------------------+--------------------------------+
| UART      | uart                         | serial port-polling            |
+-----------+------------------------------+--------------------------------+

Other hardware features have not been enabled yet for this board.

The default configuration can be found in the defconfig file:

        ``boards/arm/rz_g3s/rz_g3s_defconfig``

Programming and Debugging
*************************

Flashing
========

The flash on board is not supported by Zephyr at this time.
