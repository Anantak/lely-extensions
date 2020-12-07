# Bizerba Lely Extensions

* Extends Lely Core coapp C++ CANOpen framework with the ability to configure CANOpen devices with textual DCF files.
* Adds a CiA-402 motor driver (tested with AuxInd and Servotronix)
* The functionality was developed for and with Bizerba SE & Co KG Balingen.
* Copyright 2019-2020 Bizerba SE & Co KG, Balingen - Germany.

# Prequesites

* To get this working, Lely Core master, at least [this revision](https://gitlab.com/lely_industries/lely-core/-/commit/ceee298045367e19b2cab1427ef05ec960c0489a) is needed
* ~~You have to apply `patches/0001-Add-flag-to-determine-which-parameters-were-set-expl.patch` to Lely-Core.~~
  * This patch is needed for the textual DCF configuration to determine which DCFs were configured with `ParameterValue=` in the .dcf file.
  * Only those DCFs are transferred to the slaves.
* As build system, cmake is used
* configure `include-lely-core.cmake` according to your system setup.

# Overview on the code

* This [UML diagram](doc/Classes Public.png) gives an overview on the classes provided by this project
* For the `MotorDriver`, we designed a state machine which is described [here](doc/MotorDriver State Machine.png)
* The static library `LelyIntegration` contains our DCF loader and CiA-402 motor driver.
* The executable project `LelyTest` is an example how to use the motor driver and textual configuration.
  
# The Demo Application

* Since the classes in `LelyIntegration` are highly configurable through dependency injection, a bit configuration is needed before they can be used.
  * The main goal of the demo application is to provide useful code snippets as starting point for your own application.
* `LelyTest` expects 3 motors (tested with AuxInd FD1) on the IDs 2, 3 and 4
* The master itself, provided by Lely-Core, runs on ID 16
* The system can either be configured through the YAML file `LelyTest/demo.yml`. Read [here](https://opensource.lely.com/canopen/docs/dcf-tools/) for a reference.
  * the corresponding `master.dcf` and `node_x.bin` files are placed in the subfolder `demo`
  * they are built by `LelyTest/CMakeLists.txt`
* Or through textual DCF files (`LelyTest/master.dcf`, `LelyTest/motor.dcf` and `LelyTest/motor_4.dcf`)
* It contains three initialisation functions for the three ways to contol the motors:
  * `initializeMasterForPdoControl()`: Used together with the YAML configuration. Uses the [remote PDO mapping feature](https://opensource.lely.com/canopen/release/v2.1.0/#remote-pdo-mapping-in-c) of Lely Core 
  * `initializeMasterForPdoControlWithManualMapping()`: Uses the texual DCF configuration + [manual mapping](doc/manual-PDO-mapping-example.md) of the PDO configuration to SDOs on the master.
  * `initializeMasterForSdoControl()`: Uses the texual DCF configuration + control of the motor's movements through SDO communication. The the status word (SDO 0x6041) updates from the motor to the driver, a PDO is still needed.
  
# The Pseudo Machine for the Demo Application

* Imagine a machine with the following drive setup:
* The motor with ID 2 runs independant and is equipped with a homing sensor.
* Motors 3 and 4 drive together one feeding unit, each motor drives a belt which hold one side of the material to feed. 
  * The have to run 100% synchronous, else the material gets damaged.
  * Both motors react on the same PDOs
  * The motor with the lower node ID is the main unit to control from the application side.
  * The motor with the higher node ID is the follower which does exactly the same as the main motor.
  * This is used to explain the follower relationship in the `MotorDriver`.
  * It is automatically determined from the PDO configuration.
    
