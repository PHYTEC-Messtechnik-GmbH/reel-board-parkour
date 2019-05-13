Sensor Beacon Sample
####################

Overview
********

The sample will periodically send out advertising packets containing
sensor data. This sample includes the *discovery* utility and
a unit file for systemd *sensorbeacon.service*.

The utility (running on the host) scans for advertising packets,
extracts the sensor values, and publishes them to the local
MQTT server.


Requirements
************

* reel board
* Linux host
* BlueZ 5.50 running on the host
* Paho MQTT Python Client (python3-paho-mqtt)
* Mosquitto server: any version that supports MQTT v3.1.1
* Mosquitto subscriber

Building and Running
********************

Build and flash the sample for the reel board:

.. code-block:: console

   cd sensor_beacon
   mkdir build && cd build
   cmake ..
   make flash

If necessary, start MQTT server:

.. code-block:: console

   sudo mosquitto -v -p 1883

Add unit file *sensorbeacon.service* to systemd User Unit Search Path,
e.g. /home/user/.config/systemd/user/ (can be different on your system),
and correct ExecStart according directory structure on your system.
Then enable *sensorbeacon.service*:

.. code-block:: console

   systemctl --user enable sensorbeacon.service

It is also possilbe to edit the unit file and reenable the service:

.. code-block:: console

   systemctl --user reenable sensorbeacon.service

Start the servce and make sure that it works:

.. code-block:: console

   systemctl --user start sensorbeacon.service
   systemctl --user status sensorbeacon.service


Testing
*******

Open a terminal window on your Linux host and type:

.. code-block:: console

   $ mosquitto_sub -t sensor

To stop and disable the service, execute:

.. code-block:: console

   systemctl --user stop sensorbeacon.service
   systemctl --user disable sensorbeacon.service

