.. _comms:

=====================
Communication devices
=====================

Communication devices were included in the *Stonefish* library to account for the delays and directional character of different communication interfaces, in different mediums.
They can be attached to all kinds of bodies, as well as directly to the world frame. All communication devices share the following properties:

1) **Name**: unique string

2) **Device ID**: unique number identifying the device

3) **Type**: type of the communication device

4) **Origin**: position and orientation of the device frame with respect to the parent frame

5) **Link name**: name of the link of the robot the device is attached to (for robots)

.. code-block:: xml
    
    <comm name="{1}" device_id="{2}" type="{3}">
        <!-- specific definitions here -->
        <origin xyz="{4a}" rpy="{4b}"/>
        <link name="{5}"/>
    </comm>

.. note::

    In the following sections, description of each specific implementation of a communication device is accompanied with an example of its instantiation through the XML syntax and the C++ code. It is assumed that the XML snippets are located inside the definition of a robot. In case of the C++ code, it is assumed that a ``std::unique_ptr<Robot> robot`` was defined before the definition of the device, and the robot object was properly instantiated. 

Acoustic modem
==============

An acoustic modem is an underwater communication device based on an acoustic transducer. When creating an acoustic modem it is required to specify an id of the acoustic node it will be connected to.
During the acoustic communication the directional characteristics of both the sender and the receiver are used to determine if both nodes can see each other. 
Moreover, an occlusion test is performed as default, to take into account the obstacles located on the path of the acoustic beam. The occlusion test can be disabled (it has to be done for both communicating nodes).

.. code-block:: xml

    <comm name="Modem" device_id="5" type="acoustic_modem">
        <specs min_vertical_fov="0.0" max_vertical_fov="220.0" range="1000.0"/>
        <connect device_id="9" occlusion_test="true"/>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <link name="Link1"/>
    </comm>
    
.. code-block:: cpp

    #include <Stonefish/comms/AcousticModem.h>

    std::unique_ptr<sf::AcousticModem> modem = std::make_unique<sf::AcousticModem>("Modem", 5, 0.0, 220.0, 1000.0);
    modem->Connect(9);
    modem->setOcclusionTest(true);
    robot->AddComm(std::move(modem), "Link1", sf::I4());

USBL
====

The ultra short baseline (USBL) is a device based on a tightly packed array of underwater acoustic transducers. It shares the same properties as the acoustic modem and extends upon them.
It can be used for underwater communication as well as for localization of the signal source in 3D space. User can optionally define the standard deviation of the measurements of slant range, horizontal angle and vertical angle. Moreover, the resolution of the range and angle measurements can be set.
Another feature of the USBL implementation is an automatic ping function used to update the measurements at a specified rate.
Two models of the USBL are available: a simple model (``simple_usbl``), which corrupts the ideal measurements with noise of the specified standard deviation, and a realistic model (``usbl``), which simulates the underlying time-of-flight and phase measurements.

.. note::

    In previous versions of the library ``type="usbl"`` selected the simple model and ``type="usbl2"`` the realistic one. Scenario files using the simple model have to be updated to ``type="simple_usbl"``, otherwise they will silently get the realistic model.

Simple USBL
-----------

.. code-block:: xml    

    <comm name="USBL" device_id="5" type="simple_usbl">
        <specs min_vertical_fov="0.0" max_vertical_fov="220.0" range="1000.0"/>
        <connect device_id="9" occlusion_test="false"/>
        <autoping rate="1.0"/>
        <noise range="0.05" horizontal_angle="0.2" vertical_angle="0.5"/>
        <resolution range="0.1" angle="0.1"/>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <link name="Link1"/>
    </comm>

.. code-block:: cpp

    #include <Stonefish/comms/SimpleUSBL.h>

    std::unique_ptr<sf::SimpleUSBL> usbl = std::make_unique<sf::SimpleUSBL>("USBL", 5, 0.0, 220.0, 1000.0);
    usbl->Connect(9);
    usbl->EnableAutoPing(1.0);
    usbl->setOcclusionTest(false);
    usbl->setNoise(0.05, 0.2, 0.5);
    usbl->setResolution(0.1, 0.1);
    robot->AddComm(std::move(usbl), "Link1", sf::I4());

Realistic USBL
--------------

The realistic USBL model requires the carrier frequency of the acoustic signal [Hz] and the baseline, i.e., the distance between the transducers forming one pair [m]. The optional noise is defined as the standard deviation of the time-of-flight measurement [s], the sound velocity in water [m/s], the phase measurement and the depth measurement [m], as well as the error in the baseline [m].

.. code-block:: xml

    <comm name="USBL" device_id="5" type="usbl">
        <specs min_vertical_fov="0.0" max_vertical_fov="220.0" range="1000.0" frequency="25000.0" baseline="0.05"/>
        <connect device_id="9" occlusion_test="false"/>
        <autoping rate="1.0"/>
        <noise time_of_flight="0.00001" sound_velocity="1.0" phase="0.01" depth="0.1" baseline_error="0.0001"/>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <link name="Link1"/>
    </comm>

.. code-block:: cpp

    #include <Stonefish/comms/RealUSBL.h>

    std::unique_ptr<sf::RealUSBL> usbl = std::make_unique<sf::RealUSBL>("USBL", 5, 0.0, 220.0, 1000.0, 25000.0, 0.05);
    usbl->Connect(9);
    usbl->EnableAutoPing(1.0);
    usbl->setOcclusionTest(false);
    usbl->setNoise(0.00001, 1.0, 0.01, 0.0001, 0.1);
    robot->AddComm(std::move(usbl), "Link1", sf::I4());

Optical modem
=============

An optical modem, sometimes called VLC (visual light communication) device, is a communication device based on a combination of strong LEDs and photodiodes. When creating an optical modem it is required to specify an id of the optical node it will be connected to.
During the optical communication the directional characteristics of both the sender and the receiver are used to determine if both nodes can see each other. 
Moreover, an occlusion test is performed, to take into account the obstacles located on the path of the optical beam. Apart from geometrical limitations the optical modem is also implementing reception quality estimation and range limitation
based on water turbidity, depth, and ambient light intensity. User can define a factor specifying how much the ambient light is affecting the reception quality.

.. code-block:: xml

    <comm name="Modem" device_id="5" type="optical_modem">
        <specs fov="120.0" range="50.0" ambient_light_sensitivity="0.5"/>
        <connect device_id="9"/>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <link name="Link1"/>
    </comm>
    
.. code-block:: cpp

    #include <Stonefish/comms/OpticalModem.h>
    
    std::unique_ptr<sf::OpticalModem> modem = std::make_unique<sf::OpticalModem>("Modem", 5, 120.0, 50.0, 0.5);
    modem->Connect(9);
    robot->AddComm(std::move(modem), "Link1", sf::I4());