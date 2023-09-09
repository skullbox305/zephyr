.. _fdc2x1x:

Atlas Scientific OEM Sensors (pH, EC, ORP, D.O. and/or RTD)
###########################################################

Overview
********




Wiring
*******

This sample uses the Atlas Scientific OEM-pH/EC/ORP/D.O. and/or RTD sensor 
controlled using the I2C interface.
Connect supply **VDD** and **GND**. The supply voltage can be in
the 3.0 to 5.5V range.

Connect Interface: **SDA**, **SCL** and optionally connect **INT** to a
interrupt capable GPIO.

For detailed description refer to the `OEM-pH datasheet`_ / `OEM-ORP datasheet`_ / 
`OEM-DO datasheet`_ / `OEM-EC datasheet`_ / `OEM-RTD datasheet`_ 
at pages 5 or 6.


Building and Running
********************

This sample outputs sensor data to the console and can be read by any serial
console program. It should work with any platform featuring a I2C interface.
The platform in use requires a custom devicetree overlay.
In this example the :ref:`nrf52840_mdk` board is used. The devicetree
overlay of this board provides example settings for evaluation, which
you can use as a reference for other platforms.

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/atlas_scientific_oem
   :board: nrf52840_mdk
   :goals: build flash
   :compact:

Sample Output: pH
==========================================

.. code-block:: console



        <repeats endlessly>


Sample Output: ORP
==========================================

.. code-block:: console



        <repeats endlessly>


Sample Output: Dissolved Oxygen (D.O.)
==========================================

.. code-block:: console



        <repeats endlessly>


Sample Output: Conductivity (EC)
==========================================

.. code-block:: console



        <repeats endlessly>


Sample Output: Temperature (RTD)
==========================================

.. code-block:: console



        <repeats endlessly>


References
**********

Atlas Scientific pH OEM Datasheet and Product Info:
 https://atlas-scientific.com/embedded-solutions/ph-oem-circuit/

Atlas Scientific ORP OEM Datasheet and Product Info:
 https://atlas-scientific.com/embedded-solutions/orp-oem-circuit/

Atlas Scientific Dissolved Oxygen OEM Datasheet and Product Info:
 https://atlas-scientific.com/embedded-solutions/do-oem-circuit/

Atlas Scientific Conductivity OEM Datasheet and Product Info:
 https://atlas-scientific.com/embedded-solutions/conductivity-oem-circuit/

Atlas Scientific RTD 2-Wire Temperature OEM Datasheet and Product Info:
 https://atlas-scientific.com/product/rtd-2-temperature-oem-circuit/

.. _OEM-pH datasheet: https://files.atlas-scientific.com/oem_pH_datasheet.pdf

.. _OEM-ORP datasheet: https://files.atlas-scientific.com/ORP_oem_datasheet.pdf

.. _OEM-DO datasheet: https://files.atlas-scientific.com/ORP_oem_datasheet.pdf

.. _OEM-EC datasheet: https://files.atlas-scientific.com/EC_oem_datasheet.pdf

.. _OEM-RTD datasheet: https://files.atlas-scientific.com/RTD_2_oem_datasheet.pdf
