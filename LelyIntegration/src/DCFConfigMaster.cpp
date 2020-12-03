/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the implementation of a CANOpen master which can be configured with textual DCF files.
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

#include <lely/co/dev.hpp>
#include <lely/co/obj.hpp>
#include <lely/util/diag.h>

#include "MotorDriver.h"
#include "DCFConfigMaster.h"
#include "DCFDriverConfig.h"


DCFConfigMaster::DCFConfigMaster(lely::io::TimerBase &timer, lely::io::CanChannelBase &chan, const std::string &dcf_txt, ev_exec_t *exec) :
	lely::canopen::AsyncMaster(timer, chan, dcf_txt),
	m_exec(exec)
{
	diag(DIAG_INFO, 0, "Master runnning on node ID 0x%02x, configured from %s", id(), dcf_txt.c_str());

	// Forward SDO changes of the master, which were probably triggered by PDOs from the slaves.
	OnWrite([this](uint16_t idx, uint8_t subidx)
	{
		for (auto& driver : m_drivers)
		{
			driver.second->onMasterSDOChanged(idx, subidx);
		}
	});
}

void DCFConfigMaster::configureDrivers()
{
	initializeDevicesFromTextualDCF();
	initializeDevicesForBinaryDCF();
}

void DCFConfigMaster::registerDriver(std::shared_ptr<DCFDriver> driver)
{
	m_drivers[driver->id()] = driver;
	m_devicesToBoot.insert(driver->id());
}

std::shared_ptr<DCFDriver> DCFConfigMaster::getDriver(uint8_t nodeID)
{
	auto result = m_drivers.find(nodeID);
	if (result == m_drivers.end())
		return nullptr;
	else
		return result->second;
}

uint8_t DCFConfigMaster::getFirstNodeIDUsing_RPDO_COB_ID(uint32_t cobID)
{
	auto mapping = m_firstNodeIDUsing_RPDO_COB_ID.find(cobID);
	if (mapping == m_firstNodeIDUsing_RPDO_COB_ID.end())
		return 0;
	else
		return mapping->second;
}

void DCFConfigMaster::setFirstNodeIDUsing_RPDO_COB_ID(uint8_t nodeID, uint32_t cobID)
{
	m_firstNodeIDUsing_RPDO_COB_ID[cobID] = nodeID;
}

void DCFConfigMaster::reset()
{
	for (const auto& driver : m_drivers)
	{
		m_devicesToBoot.insert(driver.first);
	}
	Command(lely::canopen::NmtCommand::RESET_NODE);  // Let the other nodes listen again. Also triggers the reconfiguration.
}

const char* DCFConfigMaster::getSoftwareFileForSlave(uint8_t nodeID, std::error_code &error)
{
	return GetUploadFile(0x1F58, nodeID, error);
}

void DCFConfigMaster::OnBoot(uint8_t id, lely::canopen::NmtState st, char es, const std::string &what) noexcept
{
	lely::canopen::AsyncMaster::OnBoot(id, st, es, what);
	if (m_bootCompletedCallback != nullptr)
		m_bootCompletedCallback(id);

	// Ensurce that the bootCompletedCallback is only called at the first time. m_devicesToBoot will not be refilled.
	if (es == 0 && m_devicesToBoot.size() > 0)
	{
		auto dev = m_devicesToBoot.find(id);
		if (dev != m_devicesToBoot.end())
			m_devicesToBoot.erase(dev);
		else
			diag(DIAG_WARNING, 0, "Node ID 0x%02x is not in m_devicesToBoot.", id);

		// TODO: Use internal (inherited) map to check if all devices have been booted.
		if (m_devicesToBoot.size() == 0 && m_bootCompletedCallback != nullptr)
		{
			for (const auto& driver : m_drivers)
				driver.second->onSystemBootCompleted();
			m_bootCompletedCallback(0);
		}
	}
}

void DCFConfigMaster::OnCommand(lely::canopen::NmtCommand cs) noexcept
{
	if (cs == lely::canopen::NmtCommand::RESET_COMM)
	{
		for (const auto& driver : m_drivers)
		{
			std::error_code error;
			// Disable automatic textual upload in any case since this is broken in Lely Core for PDO configuration.
			SetUploadFile(0x1F20, driver.first, "", error);
			// Disable automatic binary upload if a custom clear configuration strategy has been set.
			// In this case, DCFDriver will trigger the binary upload after the configuration was cleared.
			if (driver.second->hasCustomClearConfigurationStrategy())
				SetUploadFile(0x1F22, driver.first, "", error);
			// Ignore the error since 1F20 or 1F22 might not be set depending on the system configuration.
		}
	}

	// Let the parent class forward the event to the drivers.
	lely::canopen::AsyncMaster::OnCommand(cs);
}

void DCFConfigMaster::OnConfig(uint8_t id) noexcept
{

	lely::canopen::AsyncMaster::OnConfig(id);
	if (m_nodeConfigStartedCallback != nullptr)
		m_nodeConfigStartedCallback(id);
}

void DCFConfigMaster::OnState(uint8_t id, lely::canopen::NmtState st) noexcept
{
	lely::canopen::AsyncMaster::OnState(id, st);
	// TODO: Due to a bug in lely-core 2.0 we cannot track when the config of a motor has been finished
	// because we are not called here during the configuration. This will be fixed in lely-core 2.1
}

void DCFConfigMaster::initializeDevicesFromTextualDCF()
{
	for (uint8_t subIndex = 1; subIndex <= 127; subIndex++)
	{
		std::error_code error;
		auto* filenameBlob = GetUploadFile(0x1F20, subIndex, error);
		// Errors are expected at this point since the SDO might not exist.
		if (filenameBlob != nullptr)
		{
			std::string filename(filenameBlob);
			if (filename.size() > 0)
			{
				diag(DIAG_INFO, 0, "0x1F20:0x%02x: Loading textual slave DCF %s ...", subIndex, filename.c_str());
				if (m_loadConfigStartedCallback != nullptr)
					m_loadConfigStartedCallback(subIndex);
				auto driverConfig = std::make_shared<DCFDriverConfig>(filename, /* binary DCF */ "", subIndex);
				registerDriver(m_driverFactory(driverConfig));
			}
		}
		else
		{
			// break;  // No more slave entries.
		}
	}
}

void DCFConfigMaster::initializeDevicesForBinaryDCF()
{
	for (uint8_t subIndex = 1; subIndex <= 127; subIndex++)
	{
		std::error_code error;
		auto* filenameBlob = GetUploadFile(0x1F22, subIndex, error);
		// Errors are expected at this point since the SDO might not exist.
		if (filenameBlob != nullptr)
		{
			std::string filename(filenameBlob);
			if (filename.size() > 0)
			{
				diag(DIAG_INFO, 0, "0x1F20:0x%02x: Create device driver for binary slave DCF %s ...", subIndex, filename.c_str());
				if (m_loadConfigStartedCallback != nullptr)
					m_loadConfigStartedCallback(subIndex);
				auto driverConfig = std::make_shared<DCFDriverConfig>("dummy.dcf", filename, subIndex);
				registerDriver(m_driverFactory(driverConfig));
			}
		}
		else
		{
			// break;  // No more slave entries.
		}
	}
}

