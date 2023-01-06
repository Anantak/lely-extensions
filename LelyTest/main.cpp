/**@file
 * This file is part of the LelyIntegration library;
 * it contains a test/demo application for the LelyIntegration library.
 *
 * @copyright 2019-2020 Bizerba SE & Co. KG
 *
 * @author Jonas Lauer <Jonas.Lauer@bizerba.com>
 * @author Florian Mayer <info@sans-ltd.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <lely/util/diag.h>

#include <lely/ev/loop.hpp>

#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/timer.hpp>

#include <lely/coapp/master.hpp>
#include <lely/coapp/driver.hpp>

#include "MotorDriver.h"
#include "DCFConfigMaster.h"

void onResult(uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec, uint32_t value)
{
	diag(DIAG_INFO, 0, "received 0x%08X when reading object %04X:%02X from %02X",
			value, idx, subidx, id);
}

void onWriteResult(uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec)
{
	diag(DIAG_INFO, 0, "Sent object %04X:%02X to %02X", idx, subidx, id);
}

void demoFollowerMove(std::shared_ptr<DCFConfigMaster> master, std::function<void()> callback)
{
	// System layout:
	// - Drive 2 has a homing sensor.
	// - Drive 3 and 4 drive control each side of a belt conveyor and have to run absolutely synchronous, so drive 4 follows drive 3.
	// This demo does the following:
	// 1) Clear the belt conveyor: turn drive 3 and 4 in relative movement in both directions
	//    Since the follower relationship is not possible in SDO mode, only drive 3 will turn in this mode.
	master->getDriver(3)->GetExecutor().post([master,callback]()
	{
		// Step 1:
		auto motor3and4 = std::dynamic_pointer_cast<MotorDriver>(master->getDriver(3));
		motor3and4->GetExecutor().post([motor3and4,master,callback]()
		{
			motor3and4->move(MotorDriver::MoveMode::RELATIVE, 100000, 10000, 1000, 1000, [motor3and4,master,callback]()
			{
				motor3and4->GetExecutor().post([motor3and4,master,callback]()
				{
					motor3and4->move(MotorDriver::MoveMode::RELATIVE, -100000, 10000, 1000, 1000, [master,callback]()
					{
						callback();
					});
				});
			});
		});
	});
}

void demoHomingAndMove(std::shared_ptr<DCFConfigMaster> master)
{
	// System layout:
	// - Drive 2 has a homing sensor.
	// - Drive 3 and 4 drive control each side of a belt conveyor and have to run absolutely synchronous, so drive 4 follows drive 3.
	// This demo does the following:
	// 1) Home drive 2
	// 2) Move drive 2 to zero position.
	// 3) Move drive 2 absolute.
	master->getDriver(3)->GetExecutor().post([master]()
	{
		// Step 1:
		auto motor2 = std::dynamic_pointer_cast<MotorDriver>(master->getDriver(2));
		motor2->GetExecutor().post([motor2]()
		{
			motor2->home(MotorDriver::PredefinedHomingMethod::HOMING_FORWARD_RISING_EDGE, 5000, 10000, 1000, 5000, [motor2]()
			{
				// Step 2:
				motor2->GetExecutor().post([motor2]()
				{
					motor2->move(MotorDriver::MoveMode::ABSOLUTE, 0, 20000, 1000, 1000, [motor2]()
					{
						// Step 3:
						motor2->GetExecutor().post([motor2]()
						{
							motor2->move(MotorDriver::MoveMode::ABSOLUTE, 10000, 2000, 1000, 1000);
						});
					});
				});
			});
		});
	});
}




// Initialize for the following scenario:
// Motors are controlled through PDOs (fast, follower relationships possible: two motors do exactly the same at the same time.)
// The reverse PDO mapping feature of Lely 2.1 + YAML configuration of Lely 2.2 is used.
std::shared_ptr<DCFConfigMaster> initializeMasterForPdoControl(lely::io::Timer& timer, lely::ev::Executor& exec, lely::io::CanChannel& channel)
{
	auto master = std::make_shared<DCFConfigMaster>(timer, channel, /* dcf description of the master */ "demo/master.dcf", exec);
	master->setDriverFactory([exec,master](std::shared_ptr<DCFDriverConfig> config)
	{
		// TODO if multiple different devices are in use: decide up on the config which driver to create.
		std::shared_ptr<MotorDriver> driver = std::make_shared<MotorDriver>(exec, *master, config);

		MotorDriver::CommunicationConfig commConfig;

		// send PDO when the setter is called? ----------------------------------------------------------------------------v
		// This depends on the PDO layout and has to be configured here.
		commConfig.setMotorOperationModeSetter(driver->createMappedTpdoSetter<int8_t>  (MotorDriver::MOTOR_OPERATIONMODE, false));
		commConfig.setMotorControlWordSetter  (driver->createMappedTpdoSetter<uint16_t>(MotorDriver::MOTOR_CONTROLWORD,   true));
		commConfig.setMotorPositionSetter     (driver->createMappedTpdoSetter<int32_t> (MotorDriver::MOTOR_POSITION,      false));
		commConfig.setMotorVelocitySetter     (driver->createMappedTpdoSetter<uint32_t>(MotorDriver::MOTOR_VELOCITY,      true));
		commConfig.setMotorAccelerationSetter (driver->createMappedTpdoSetter<uint32_t>(MotorDriver::MOTOR_ACCELERATION,  false));
		commConfig.setMotorDecelerationSetter (driver->createMappedTpdoSetter<uint32_t>(MotorDriver::MOTOR_DECELERATION,  true));
		driver->setCommunicationConfig(commConfig);

		return driver;
	});

	master->setBootCompletedCallback([master](uint8_t nodeID)
	{
		if (nodeID == 0)
		{
			diag(DIAG_INFO, 0, "Performing a move with two following motors:");
			demoFollowerMove(master, [master]()
			{
				diag(DIAG_INFO, 0, "Performing a homing + move:");
				demoHomingAndMove(master);
			});
		}
	});

	return master;
}

// Initialize for the following scenario:
// Motors are controlled through PDOs (fast, follower relationships possible)
// implementation with manual mapping of the motor SDO registers on the master through manual PDO configuration.
enum MasterSDO : uint16_t
{
	MOTOR_CONTROLWORD = 0x2000,
	MOTOR_OPERATIONMODE = 0x2001,
	MOTOR_POSITION = 0x2002,
	MOTOR_VELOCITY = 0x2003,
	MOTOR_ACCELERATION = 0x2004,
	MOTOR_DECELERATION = 0x2005,
	MOTOR_STATUSWORD = 0x2010
};

enum PDOGroup {
	MOTOR_CONTROL_PDO           = 0x00,
	MOTOR_POSITION_VELOCITY_PDO = 0x10,
	MOTOR_DE_ACCELERATION_PDO   = 0x20
};

std::shared_ptr<DCFConfigMaster> initializeMasterForPdoControlWithManualMapping(lely::io::Timer& timer, lely::ev::Executor& exec, lely::io::CanChannel& channel)
{
	auto master = std::make_shared<DCFConfigMaster>(timer, channel, /* dcf description of the master */ "master.dcf", exec);
	master->setDriverFactory([exec,master](std::shared_ptr<DCFDriverConfig> config)
	{
		// TODO if multiple different devices are in use: decide up on the config which driver to create.
		std::shared_ptr<MotorDriver> driver = std::make_shared<MotorDriver>(exec, *master, config);

		MotorDriver::CommunicationConfig commConfig;
		// Textual Configuration with control through the master SDOs:
		// master TPDO to trigger when set -------------------------------------------------------------------------------------------v
		// master SDO sub-index ----------------------------------------------------------------------------------------v
		// master SDO index ------------------------------------------------------------v
		commConfig.setMotorOperationModeSetter(driver->createMasterSDOSetter<int8_t>  (MasterSDO::MOTOR_OPERATIONMODE, driver->id(), -1));
		commConfig.setMotorControlWordSetter  (driver->createMasterSDOSetter<uint16_t>(MasterSDO::MOTOR_CONTROLWORD,   driver->id(), PDOGroup::MOTOR_CONTROL_PDO + driver->id()));
		commConfig.setMotorPositionSetter     (driver->createMasterSDOSetter<int32_t> (MasterSDO::MOTOR_POSITION,      driver->id(), -1));
		commConfig.setMotorVelocitySetter     (driver->createMasterSDOSetter<uint32_t>(MasterSDO::MOTOR_VELOCITY,      driver->id(), PDOGroup::MOTOR_POSITION_VELOCITY_PDO + driver->id()));
		commConfig.setMotorAccelerationSetter (driver->createMasterSDOSetter<uint32_t>(MasterSDO::MOTOR_ACCELERATION,  driver->id(), -1));
		commConfig.setMotorDecelerationSetter (driver->createMasterSDOSetter<uint32_t>(MasterSDO::MOTOR_DECELERATION,  driver->id(), PDOGroup::MOTOR_DE_ACCELERATION_PDO + driver->id()));
		commConfig.setIsStatusWordCheckForMasterSDOChange([](uint16_t masterIndex, uint8_t masterSubIndex, uint8_t nodeID) -> bool
		{
			// This lambda is called each time a SDO on the master changes from an external source. But driver is only interested
			// in changes of the status word.
			return masterIndex == MasterSDO::MOTOR_STATUSWORD && masterSubIndex == nodeID;
		});
		driver->setCommunicationConfig(commConfig);

		master->setBootCompletedCallback([master](uint8_t nodeID)
		{
			if (nodeID == 0)
				demoFollowerMove(master, [master]()
				{
					demoHomingAndMove(master);
				});
		});

		return driver;
	});
	return master;
}

// Initialize for the following scenario:
// Motors are controlled through SDOs:
// simple from code point of view, but with overhead on the CAN bus
// --> so slow and save, but no follower relationships possible
// --> for the return channel from the motors to the master PDO communication is still needed.
std::shared_ptr<DCFConfigMaster> initializeMasterForSdoControl(lely::io::Timer& timer, lely::ev::Executor& exec, lely::io::CanChannel& channel)
{
	auto master = std::make_shared<DCFConfigMaster>(timer, channel, /* dcf description of the master */ "master.dcf", exec);
	master->setDriverFactory(
			[exec,master](std::shared_ptr<DCFDriverConfig> config)
	{
		// TODO if multiple different devices are in use: decide up on the config which driver to create.
		std::shared_ptr<MotorDriver> driver = std::make_shared<MotorDriver>(exec, *master, config);

		MotorDriver::CommunicationConfig commConfig;
		commConfig.setMotorOperationModeSetter(driver->createSDOSetter<int8_t>  (MotorDriver::MOTOR_OPERATIONMODE));
		commConfig.setMotorControlWordSetter  (driver->createSDOSetter<uint16_t>(MotorDriver::MOTOR_CONTROLWORD  ));
		commConfig.setMotorPositionSetter     (driver->createSDOSetter<int32_t> (MotorDriver::MOTOR_POSITION     ));
		commConfig.setMotorVelocitySetter     (driver->createSDOSetter<uint32_t>(MotorDriver::MOTOR_VELOCITY     ));
		commConfig.setMotorAccelerationSetter (driver->createSDOSetter<uint32_t>(MotorDriver::MOTOR_ACCELERATION ));
		commConfig.setMotorDecelerationSetter (driver->createSDOSetter<uint32_t>(MotorDriver::MOTOR_DECELERATION ));
		commConfig.setIsStatusWordCheckForMasterSDOChange([](uint16_t masterIndex, uint8_t masterSubIndex, uint8_t nodeID) -> bool
		{
			// This lambda is called each time a SDO on the master changes from an external source. But driver is only interested
			// in changes of the status word.
			return masterIndex == MasterSDO::MOTOR_STATUSWORD && masterSubIndex == nodeID;
		});
		driver->setCommunicationConfig(commConfig);

		master->setBootCompletedCallback([master](uint8_t nodeID)
		{
			if (nodeID == 0)
				demoHomingAndMove(master);
		});

		return driver;
	});
	return master;
}

int main()
{
	std::cout << "Please select the type of communication:" << std::endl;
	std::cout << "----------------------------------------" << std::endl;
	std::cout << " 1) PDO communication with Reverse PDO mappings from the YAML config" << std::endl;
	std::cout << " 2) SDO communication (still using a PDO for staus word changes)" << std::endl;
	std::cout << " 3) PDO communication with manual PDO mappings on the master and textual DCF config for the slaves" << std::endl;

	int input = std::getchar();

	lely::io::Context ctx;
	lely::io::Poll poll(ctx);
	lely::ev::Loop loop(poll.get_poll());
	auto exec = loop.get_executor();
	lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);

	lely::io::CanController ctrl("can0");
	lely::io::CanChannel channel(poll, exec);
	channel.open(ctrl);

	std::shared_ptr<DCFConfigMaster> master = nullptr;


	if (input == '1')
		master = initializeMasterForPdoControl(timer, exec, channel);
	else if (input == '2')
		master = initializeMasterForSdoControl(timer, exec, channel);
	else if (input == '3')
		master = initializeMasterForPdoControlWithManualMapping(timer, exec, channel);
	else
		exit(0);

	master->SetTimeout(std::chrono::milliseconds(1000));

	master->configureDrivers();
	master->Reset();
	loop.run();

	return 0;
}
