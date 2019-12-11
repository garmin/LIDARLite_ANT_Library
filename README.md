# LIDARLite_ANT_Library

This library should be used when developing applications to run directly on the nRF52840 Nordic Semiconductor System on Chip (SoC) within the LIDAR-Lite v4 LED.

[Product Page: LIDAR-Lite v4 LED](https://buy.garmin.com/en-US/US/p/610275/pn/010-02022-00) - **See product page for operating manual**

For libraries to communicate with the LIDAR-Lite v4 LED via I2C using an Arduino, see [LIDAR-Lite Arduino Library](https://github.com/garmin/LIDARLite_Arduino_Library).

**Important:** Before reprogramming the LIDAR-Lite v4 LED, ensure you are aware of the following:
* [ANT-ID](#notice-removal-of-ant-id)
* [Bluetooth LE secure DFU bootloader](#notice-removal-of-bluetooth-le-secure-dfu-bootloader)

# Table of Contents

1. [Overview](#overview)
2. [Required Hardware](#required-hardware)
3. [ANT developer account](#ant-developer-account)
4. [Developing Custom Applications on the nRF52840](#developing-custom-applications-on-the-nrf52840)
5. [Development Environment Setup](#development-environment-setup)
6. [nRF SoC LIDAR-Lite Library](#nrf-soc-lidar-lite-library)
7. [Example – LIDAR Lite Demo App](#lidar_lite_demo_app)
8. [Example – Default LIDAR Lite App](#default_lidar_lite_app)
9. [Preserving the ANT ID](#preserving-the-ant-id)
10. [Related Documents](#related-documents)
11. [Garmin PC Simulator](#garmin-pc-simulator)
12. [License](#license)

## Overview

The LIDAR-Lite v4 LED consists of a nRF52840 and a FPGA. Simplified, when taking a distance measurement, the FPGA is responsible for taking acquisitions by transmitting and receiving light pulses to create a correlation record that represents the measured object at a given distance.
The nRF communicates with the FPGA over SPI and is responsible for controlling the FPGA to take acquisitions and to calculate distances from the correlation record.

The complexity of this measurement process on the LIDAR-Lite v4 LED is encapsulated in a [static library](#nrf-soc-lidar-lite-library) that a developer can then include into their projects to make simple API calls that take distance measurements.

Any functionality that is not related to the taking of distance measurements is left in the application so that you can customize how the device operates so that you can adjust any feature to fit your projects needs. This includes:
* ANT transmissions
* I2C interface
* Non volatile memory storage
* Mode control pins

## Required Hardware

* To reprogram the LIDAR-Lite v4 LED’s nRF52840, you will need a **J-Link debugging probe** that supports interfacing with a Cortex-M4 CPU architecture. There are a wide variety of compatible devices ranging from professional 20 pin J-Link devices to cheaper solutions suited to non-commercial solutions. For hobbyists looking to get started programming the LIDAR-Lite v4 LED, a recommended part is the [SEGGER J-Link EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/).

* The [Garmin PC Simulator](#garmin-pc-simulator) requires a [USB ANT Stick](https://buy.garmin.com/en-US/US/p/10997/pn/010-01058-00) to connect to the LIDAR-Lite v4 LED wirelessly.


## ANT developer account

To download some additional project dependencies such as the S340 SoftDevice, the Garmin PC simulator, or to view addition ANT documentation, you will need to signup and create an account on the [ANT website](https://www.thisisant.com/) and sign up as an **ANT+ Adopter**.

## Developing Custom Applications on the nRF52840

You can customize nRF applications as you see fit and bring in additional nRF SDK functionality to meet your project requirements.
Here are just a few possibilities:
* Configure the GPIOs to use a SPI interface, either as a master or slave
* Configure a single GPIO to trigger measurements from a button press
* Configure the GPIOs as outputs to control servo motors
* Use an ADC input to measure potentiometer values

For more information about the capabilities of the nRF52840, go the [nRF52840 Product Page](https://www.nordicsemi.com/nrf52840)

**For support on using the nRF5 SDK and reprogramming the nRF52840 SoC, go to the Nordic Semiconductor [developer page](https://devzone.nordicsemi.com/).**

See [Development Environment Setup](#development-environment-setup) and [the examples](#repo-contents) for getting started with nRF52840 development


## Development Environment Setup

### IDE

Download [Embedded Studio for ARM](https://www.segger.com/downloads/embedded-studio#ESforARM)
After installation, you will be required to activate your license. Since you will be developing on the nRF52840 this is [free](https://www.segger.com/news/segger-embedded-studio-ide-now-free-for-nordic-sdk-users/). 
Request your license activation [here](https://license.segger.com/Nordic.cgi).

### nRF Command Line Tools

Download and install the [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Command-Line-Tools/Download#infotabs)

### nRF MDK

Download the latest nRF MDK from [here](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-MDK/Download#infotabs). Choose either the **SES 5-clause Nordic license** or **SES 3-clause BSD license**

### nRF Connect for Desktop

Download and install [nRF Connect for Desktop](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Connect-for-desktop/Download#infotabs)

### nRF SoC LIDAR-Lite Library with examples

Clone or download this LIDARLite_ANT_Library repository and place it in a directory of your choice.

These instructions assume that when this repo is downloaded, its contents are in a folder named LIDARLite_ANT_Library-master.

### nRF5 SDK

* Download the nRF5 SDK version [15.3.0](https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/)
* Create a **nrf5-sdk** folder within this downloaded repo.
* Extract the contents of the nRF5 SDK zip file into the newly created  **nrf5-sdk** folder. When you are finished, your directory structure should look like this:

```
LIDARLite_ANT_Library-master
│   LICENSE
│   README.md    
│
└───app
└───bin   
└───inc
└───lidar_lite_demo_app
└───nrf5-sdk
│   └───components
│   └───config
│   └───documentation
│   └───examples
│   └───external
│   |   ...

```

### SoftDevice

The SoftDevice is a precompiled and linked binary software implementing a wireless protocol. This is required when building the [default_lidar_lite_app](#default_lidar_lite_app) example.

* Download the [S340 v6.1.1](https://www.thisisant.com/developer/components/nrf52832#tab_protocol_stacks_tab) SoftDevice.
* Extract the contents of the S340 SoftDevice zip file, and place the header files in this directory:
**LIDARLite_ANT_Library-master\nrf5-sdk\components\softdevice\s340\headers**
When you are finished, your directory structure should look like this:

```
LIDARLite_ANT_Library-master
│   LICENSE
│   README.md    
│   …    
│   …   
│
└───lidar_lite_demo_app
└───nrf5-sdk
│   └───components
│   │   └───softdevice
│   │   │   └───s340
│   │   │   │   └───headers
│   │   │   │   │   │   ant_error.h
│   │   │   │   │   │   ant_interface.h
│   │   │   │   │   │   ant_parameters.h
│   │   │   │   │   │   ble.h
│   │   │   │   │   │   …
│   │   │   │   │   └───nrf52

```
For more information regarding the SoftDevice and available APIs see [here](https://infocenter.nordicsemi.com/topic/struct_nrf52/struct/nrf52_softdevices.html)

**Note:** When compiling the project for the first time, the compilation will be halted because of an #error directive. Read and follow the instructions associated with the error to resolve any licensing keys dependencies that are required for the project.

## Repo Contents

### nRF SoC LIDAR-Lite Library

The LIDAR-Lite static library is a pre-built library that is used by custom applications to enable the distance measurement functionality on the LIDAR-Lite v4 LED. The **bin** folder includes a version of the LIDAR-Lite Library compiled with SES.
**API Headers:** These files are used by your application to access the LIDAR-Lite static library, and to provide documentation on the available API calls.
* **inc\lidar_lite_interface.h**: Function definitions and API documentation for the LIDAR-Lite Library.
* **inc\lidar_lite_defines.h**: Includes constants and data structures that are used by the LIDAR-Lite library.

### lidar_lite_demo_app

This example is a simplified application that only shows how to use nRF SoC LIDAR-Lite Library API to take measurements on the LIDAR-Lite v4 LED. From this project the developer can start to add in their own functionality as needed.
This example requires that there is **no SoftDevice** on the nRF52840.

#### Installation Instructions

**Save ANT-ID**
Before continuing, see [this warning](#notice-removal-of-ant-id) regarding the ANT-ID
* If needed, [save the ANT-ID](#preserving-the-ant-id)

**Erase nRF52840**
Run this command to erase all contents of the nRF52840:
* nrfjprog.exe -f NRF52 –recover

**Program ANT-ID**
* If needed, reprogram the [ANT-ID](#preserving-the-ant-id)

**Demo application**
* Open downloaded_repo\lidar_lite_demo_app\ses\lidar_lite_demo_app.emProject
* Select Build->Build Solution (Shift+F7)
* Select Target->Download default_lidar_lite_app (Ctrl+T,L)
* Select Debug->Go (F5)


### default_lidar_lite_app

This example provides you with most of the functionality provided from the factory, this includes:
* ANT transmissions
* I2C interface
* NVM memory storage
* Mode control pins

Functionality can be added, removed, or modified to fit your projects use case.
This example requires having the **SoftDevice S340 v6.1.1** installed to run properly.

#### Installation Instructions

**Save ANT-ID**
Before continuing, see [this warning](#notice-removal-of-ant-id) regarding the ANT-ID
* If needed, [save the ANT-ID](#preserving-the-ant-id)

**Erase nRF52840**
Run this command to erase all contents of the nRF52840:
* nrfjprog.exe -f NRF52 –recover

**Program S340 SoftDevice**
The S340 SoftDevice is required for this demo application to use the ANT wireless protocol. Run this command to program the SoftDevice onto the nRF52840:
* nrfjprog.exe -f NRF52 --program ANT_s340_nrf52840_6.1.1.hex

**Program ANT-ID**
* If needed, reprogram the [ANT-ID](#preserving-the-ant-id)

**Demo application**
* Open downloaded_repo\app\default_lidar_lite_app\ses\default_lidar_lite_app.emProject
* Select Build->Build Solution (Shift+F7)
* Select Target->Download default_lidar_lite_app (Ctrl+T,L)
* Select Debug->Go (F5)

## NOTICE removal of ANT-ID

**Reprogramming the nRF52840 SoC removes all pre-programmed factory software. The device comes preprogrammed with a unique ANT ID to ensure each device can be uniquely identified over the ANT wireless protocol. When reprogramming the device, special precautions should be taken to [preserve the ANT ID value](#preserving-the-ant-id).** If the ANT-ID is not preserved, this will not affect the operation of the LIDAR-Lite v4 LED but the device will no longer be uniquely identifiable over the ANT wireless protocol.

## NOTICE removal of Bluetooth LE secure DFU bootloader

**The LIDAR-Lite v4 LED device comes preprogrammed from the factory with a Bluetooth LE secure DFU bootloader for receiving wireless software updates. When reprogramming the nRF52840, the bootloader is removed. Garmin is unable to provide developers with the Bluetooth LE secure DFU bootloader after it is erased. If you require a bootloader after reprogramming the device, you can follow the Bluetooth LE Secure DFU Bootloader reference design in the Nordic nRF5 SDK.**

## Preserving the ANT-ID

Before the default SW is erased from the nRF52840, if it is required, the ANT-ID should be saved so that it can be reprogrammed after the custom application is programmed.

**Save the ANT-ID**
Perform a four-byte I2C read on register 0x16 to retrieve the four-byte ANT-ID. Save this value for later.

**Device Reset**
Erase and reprogram the nRF52840 as required.

**Reprogram the ANT-ID**
Use the following command to reprogram the ANT-ID:
**nrfjprog.exe -f NRF52 --memwr 0x10001090 --val \<ANT-ID>**
For example, nrfjprog.exe -f NRF52 --memwr 0x10001090 --val 0x00012345

## Related Documents

For details on the ANT message protocol and its capabilities, download [ANT Message Protocol Usage](https://www.thisisant.com/developer/resources/downloads/#documents_tab) from the **ANT Documentation** section.

For details of the wireless transmission of the LIDAR-Lite v4 LED, download the [ANT Ranging profile](https://www.thisisant.com/developer/resources/downloads/#documents_tab) from the **ANT+ device profiles** section.

## Garmin PC Simulator

To connect wirelessly to the LIDAR-Lite v4 LED, download [SimulANT (Garmin Developer version)](https://www.thisisant.com/developer/resources/downloads/#software_tab) from the **ANT+ simulators and tools** section. For details on connecting the PC to the LIDAR-Lite v4 LED see the readme contained within the **simulANT (Garmin Developer version)** download package.

**Note**: simulANT (Garmin Developer version) requires: 
* Windows 7 Service Pack 1 or higher on your PC
* Microsoft .Net Framework 4.5 or higher on your PC
* Visual C++ 2008 SP1 Redistributable Package or higher on your PC
* USB ANT Stick, see [Required Hardware](#required-hardware).

## License

Copyright (c) 2019 Garmin Ltd. or its subsidiaries. Distributed under the Apache 2.0 License.
See [LICENSE](LICENSE) for further details.
