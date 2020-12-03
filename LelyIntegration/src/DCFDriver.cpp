/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the implementation of a generic driver for a CANOpen slave which can be configured with textual DCF files.
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

#include <algorithm>
#include <sstream>

#include <boost/format.hpp>

#include <lely/co/type.h>
#include <lely/coapp/master.hpp>
#include <lely/coapp/sdo_error.hpp>
#include <lely/util/diag.h>

#include "DCFConfigMaster.h"
#include "DCFDriverConfig.h"

#include "DCFDriver.h"

DCFDriver::DCFDriver(ev_exec_t *exec, lely::canopen::BasicMaster &m, std::shared_ptr<DCFDriverConfig> config) :
	lely::canopen::BasicDriver(exec, m, config->getDefaultNodeID()),  // the node ID of master.dcf always wins since we reuse the DCF for multiple drivers and our tooling wants to set explicitely the Node ID in the DCF.
	m_followingNodeID(0),
	m_followsNodeID(0),
	m_emergencyOccured(false)
{
	m_config = config;
	m_sdosToConfigure = m_config->getSDOIndicesForDriverConfiguration();
}

void DCFDriver::OnConfig(::std::function<void (std::error_code)> res) noexcept
{
	if (!m_config->getBinaryDcfFile().empty())
		configureFollowerRelationship();

	if (m_clearConfigurationStrategy == nullptr)
	{
		configure(res);
	}
	else
	{
		m_clearConfigurationStrategy([res,this](std::error_code ec)
		{
			if (ec == std::errc::operation_canceled)
				res(std::error_code());  // Cancel configuration without an error.
			else if (ec)
				res(ec);  // no configuration due to error during m_clearConfigurationStrategy, just call the callback
			else
				configure([res,this](std::error_code error)
				{
					if (!error && !m_config->getBinaryDcfFile().empty())
						SubmitWriteDcf(m_config->getBinaryDcfFile().c_str(), [res](uint8_t /* id */, uint16_t /* idx */, uint8_t /* subidx */, ::std::error_code ec)
						{
							res(ec);
						});
					else
						res(error);
				});
		});
	}
	// In case of binary config and no m_clearConfigurationStrategy, the lely::coapp::BasicMaster does the config.
}

void DCFDriver::configure(std::function<void (std::error_code)> res)
{
	// recursively configure all SDOs.
	if (m_sdosToConfigure.size() > 0)
	{
		configureSDO(m_sdosToConfigure.begin(), res);
	}
	else
	{
		res(std::error_code());  // nothing to do.
	}
}

void DCFDriver::configureSDO(const DCFDriverConfig::ObjectsList::iterator objectToSend, ::std::function<void (std::error_code)> onCompletedFunction)
{
	if (objectToSend == m_sdosToConfigure.end())
	{
		// nothing to do anymore, upwards recursion
		onCompletedFunction(std::error_code());
		return;
	}

	auto index = std::get<0>(*objectToSend);
	if (index >= StandardSDO::RECEIVE_PDO_CONTROL_START && index <= StandardSDO::RECEIVE_PDO_CONTROL_END)
	{
		configureFollowerRelationship(objectToSend);
		configurePDO(objectToSend, onCompletedFunction);
	}
	else if (index >= StandardSDO::TRANSMIT_PDO_CONTROL_START && index <= StandardSDO::TRANSMIT_PDO_CONTROL_END)
	{
		configurePDO(objectToSend, onCompletedFunction);
	}
	else if (index >= StandardSDO::RECEIVE_PDO_MAPPING_START && index <= StandardSDO::TRANSMIT_PDO_MAPPING_END)
	{
		// ignore mappings, they are handled by the PDO control functions above
		// --> Nothing to do here, recursively take the next SDO.
		configureSDO(objectToSend + 1, onCompletedFunction);
	}
	else
	{
		// send parameter
		auto& subIndices = std::get<1>(*objectToSend);
		configureParameterSDO(objectToSend, subIndices.begin(), onCompletedFunction);
	}

}

template<typename T>
void copyObject(uint16_t index, uint8_t subIndex, std::shared_ptr<DCFDriverConfig> config, DCFDriver* driver,
				std::function<void(::std::error_code ec)> onCompletedFunction,
				std::function<void(::std::error_code ec)> onErrorFunction = nullptr,
				bool ignoreMissingSourceSDO = false)
{
	std::error_code ec;
	T value = config->Read<T>(index, subIndex, ec);

	if (ignoreMissingSourceSDO && (ec.value() == 0x6020000 || ec.value() == 0x6090011))
		onCompletedFunction(std::error_code());  // Ignore local read error, just continue.
	else if (ec)
		if (onErrorFunction != nullptr)
			onErrorFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::READ_LOCAL_VALUE, index, subIndex, ec)));
		else
			onCompletedFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::READ_LOCAL_VALUE, index, subIndex, ec)));
	else
		driver->SubmitWrite<T>(index, subIndex, std::forward<T>(value), [onCompletedFunction, onErrorFunction](uint8_t /* id */, uint16_t idx, uint8_t subidx, ::std::error_code ec)
		{
			// DCFDriver::ConfigErrorCategory cat(DCFDriver::ConfigErrorCategory::WRITE_REMOTE_SDO, idx, subidx, ec);
			if (ec && onErrorFunction != nullptr)
				onErrorFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::WRITE_REMOTE_SDO, idx, subidx, ec)));
			else
				onCompletedFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::WRITE_REMOTE_SDO, idx, subidx, ec)));
		});
}

template<typename T>
void setObject(uint16_t index, uint8_t subIndex, T value, DCFDriver* driver,
				std::function<void(::std::error_code ec)> onCompletedFunction,
				std::function<void(::std::error_code ec)> onErrorFunction)
{
	std::error_code ec;
	driver->SubmitWrite<T>(index, subIndex, std::forward<T>(value), [onCompletedFunction, onErrorFunction](uint8_t /* id */, uint16_t idx, uint8_t subidx, ::std::error_code ec)
	{
		if (ec)
			onErrorFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::WRITE_REMOTE_SDO, idx, subidx, ec)));
		else
			onCompletedFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::WRITE_REMOTE_SDO, idx, subidx, ec)));
	});
}


void DCFDriver::configurePDO(const DCFDriverConfig::ObjectsList::iterator objectToSend, ::std::function<void (::std::error_code ec)> onCompletedFunction)
{
	auto writeMappings = [this](uint16_t pdoMappingIndex,
			std::function<void(::std::error_code ec)> onCompletedFunction,
			std::function<void(::std::error_code ec)> onErrorFunction)
	{
		// find the SDO which contains the mapping (we need the iterators for configureParameterSDO())
		auto mappingSDO = std::find_if(m_sdosToConfigure.begin(), m_sdosToConfigure.end(), [pdoMappingIndex](const std::tuple<uint16_t, std::vector<uint8_t>>& sdo)
		{
			return std::get<0>(sdo) == pdoMappingIndex;
		});

		if (mappingSDO == m_sdosToConfigure.end())
		{
			onCompletedFunction(std::error_code());
			return;  // no mappings found, nothing to do.
		}

		auto& mappingIndices = std::get<1>(*mappingSDO);
		// skip the first sub index, according to the PDO protocol this has to be written as last one
		if (mappingIndices.size() > 1)
		{
			configureParameterSDO(mappingSDO, mappingIndices.begin() + 1, [pdoMappingIndex, this, onCompletedFunction, onErrorFunction](std::error_code ec)  // Recursively write PDO mappings
			{
				if (ec)
					onErrorFunction(ec);
				else
					copyObject<uint8_t>(pdoMappingIndex, 0, m_config, this, onCompletedFunction, onErrorFunction);  // PDO has x mappings (commit the mappings)
			}, /* iterateSubIndicesOnly = */ true);
		}
		else if (mappingIndices.size() > 0)
		{
			copyObject<uint8_t>(pdoMappingIndex, 0, m_config, this, onCompletedFunction, onErrorFunction);          // PDO has probably no mappings (commit)
		}
		else
		{
			onCompletedFunction(std::error_code());
		}
	};

	auto resultHandler = [objectToSend, onCompletedFunction, this](std::error_code ec)
	{
		if (ec)
		{
			onCompletedFunction(ec);  // Error occured, cancel recursion
		}
		else
		{
			// all PDO stuff for this PDO is done, recursively take the next SDO.
			configureSDO(objectToSend + 1, onCompletedFunction);
		}
	};

	auto index = std::get<0>(*objectToSend);
	SubmitRead<uint32_t>(index, 1,
						 [=](uint8_t /* id */, uint16_t index, uint8_t subIndex, ::std::error_code ec, uint32_t valueFromDevice)
	{
		if (ec)
		{
			onCompletedFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::READ_REMOTE_SDO, index, subIndex, ec)));
			return;
		}

		setObject<uint32_t>(index, 1, valueFromDevice | 0x80000000, this, [=](std::error_code)  // PDOx is invalid (prepare for setup)
		{
			copyObject<uint8_t>(index, 2, m_config, this, [=](std::error_code)                  // PDOx takes PDO type from DCF config
			{
				copyObject<uint8_t>(index, 3, m_config, this, [=](std::error_code)              // PDOx inhibit time if available.
				{
					setObject<uint8_t> (index + 0x200, 0, 0x0, this, [=](std::error_code)       // PDOx has no mappings (prepare for setup)
					{
						writeMappings(index + 0x200, [=](std::error_code)                       // Copy PDO mappings from DCF
						{
							copyObject<uint32_t>(index, 1, m_config, this, resultHandler);		// PDOx is valid (enable), use COB ID from DCF config.
						}, /* onErrorFunction = */ onCompletedFunction);
					}, /* onErrorFunction = */ onCompletedFunction);
				}, /* onErrorFunction = */ onCompletedFunction, /* ignoreMissingSourceSDO = */ true);
			}, /* onErrorFunction = */ onCompletedFunction);
		}, /* onErrorFunction = */ onCompletedFunction);
	});
}

void DCFDriver::configureFollowerRelationship(const DCFDriverConfig::ObjectsList::iterator objectToSend)
{
	// The follower relationship is detected through the COB IDs in the RPDO config:
	// If two nodes share the same COB ID, the unit with the higher Node ID follows the node with the lower Node ID.
	auto index = std::get<0>(*objectToSend);
	auto cobID = m_config->Read<uint32_t>(index, 1) & 0x1FFFFFFF;
	auto* dcfConfigMaster = dynamic_cast<DCFConfigMaster*>(&master);
	if (dcfConfigMaster != nullptr)
	{
		auto firstNodeIDforCOB_ID = dcfConfigMaster->getFirstNodeIDUsing_RPDO_COB_ID(cobID);
		if (firstNodeIDforCOB_ID == 0)
		{
			dcfConfigMaster->setFirstNodeIDUsing_RPDO_COB_ID(id(), cobID);
		}
		else
		{
			if (firstNodeIDforCOB_ID < id())
			{
				// This instance becomes the following motor.
				m_followsNodeID = firstNodeIDforCOB_ID;
				dcfConfigMaster->getDriver(firstNodeIDforCOB_ID)->setFollowingNodeID(id());
				diag(DIAG_INFO, 0, "configureFollowerRelationship: 0x%02x follows 0x%02x", id(), firstNodeIDforCOB_ID);
			}
			else if (firstNodeIDforCOB_ID > id())
			{
				// This instance becomes the main motor.
				m_followingNodeID = firstNodeIDforCOB_ID;
				dcfConfigMaster->getDriver(firstNodeIDforCOB_ID)->setFollowsNodeID(id());
				diag(DIAG_INFO, 0, "configureFollowerRelationship: 0x%02x follows 0x%02x", firstNodeIDforCOB_ID, id());
			}
		}
	}
}

void DCFDriver::configureParameterSDO(const DCFDriverConfig::ObjectsList::iterator objectToSend, const std::vector<uint8_t>::iterator subIndexToSend, ::std::function<void (std::error_code)> onCompletedFunction, bool iterateSubIndicesOnly)
{
	auto writeResultHandler = [objectToSend, subIndexToSend, onCompletedFunction, iterateSubIndicesOnly, this](std::error_code ec)
	{
		const auto& subIndices = std::get<1>(*objectToSend);
		if (ec)
		{
			onCompletedFunction(ec);  // Error occured, cancel recursion
		}
		else if (subIndexToSend + 1 != subIndices.end())
		{
			// we have still sub indices to write...
			this->configureParameterSDO(objectToSend, subIndexToSend + 1, onCompletedFunction, iterateSubIndicesOnly);
		}
		else if (iterateSubIndicesOnly)
		{
			// all sub-indices are written, nothing to do anymore, upwards recursion
			onCompletedFunction(std::error_code());
		}
		else
		{
			// all sub indices done, recursively take the next SDO.
			configureSDO(objectToSend + 1, onCompletedFunction);
		}
	};

	auto index = std::get<0>(*objectToSend);
	auto subIndex = *subIndexToSend;
	auto type = m_config->getTypeOfObject(index, subIndex);
	switch(type)
	{
	   // see lely/co/types.h for all available types
	case CO_DEFTYPE_BOOLEAN:
		copyObject<bool>(index, subIndex, m_config, this, writeResultHandler);
		break;
	case CO_DEFTYPE_INTEGER8:
		copyObject<int8_t>(index, subIndex, m_config, this, writeResultHandler);
		break;
	case CO_DEFTYPE_INTEGER16:
		copyObject<int16_t>(index, subIndex, m_config, this, writeResultHandler);
		break;
	case CO_DEFTYPE_INTEGER32:
		copyObject<int32_t>(index, subIndex, m_config, this, writeResultHandler);
		break;
	case CO_DEFTYPE_UNSIGNED8:
		copyObject<uint8_t>(index, subIndex, m_config, this, writeResultHandler);
		break;
	case CO_DEFTYPE_UNSIGNED16:
		copyObject<uint16_t>(index, subIndex, m_config, this, writeResultHandler);
		break;
	case CO_DEFTYPE_UNSIGNED32:
		copyObject<uint32_t>(index, subIndex, m_config, this, writeResultHandler);
		break;
	default:
		diag(DIAG_ERROR, 0, "cannot transfer data type 0x%04x for SDO 0x%04x/0x%02x, this data type is not supported.", type, index, subIndex);
		auto ec = std::error_code(lely::canopen::SdoErrc::DATA);
		onCompletedFunction(std::error_code(ec.value(), DCFDriver::ConfigErrorCategory(DCFDriver::ConfigErrorCategory::WRITE_REMOTE_SDO, index, subIndex, ec)));
	}
}

void DCFDriver::configureFollowerRelationship()
{
	// For YAML / DCF .bin based configuration:
	// Check if two motors with the same COB ID for RPDOs for each motor exist

	auto* dcfConfigMaster = dynamic_cast<DCFConfigMaster*>(&master);
	if (dcfConfigMaster != nullptr)
	{

		// 1) Find node ID entry for first RPDO:
		for (uint16_t nodeConfigIndex = 0x5C00; nodeConfigIndex <= 0x5DFF; nodeConfigIndex++)
		{
			std::error_code error;
			uint32_t value = master.Read<uint32_t>(nodeConfigIndex, 0, error);
			if (error)
				return;  // no entry found, no config available -> return;

			if ((0x00000100 + id()) == (value & 0x0000ffff))  // bit 0-7: node ID, bit 8 - 17 RPDO number
			{
				// 2) Get the corresponding COB ID from master PDO config:
				uint16_t cobConfigIndexForNode = nodeConfigIndex - 0x5C00 + 0x1800;
				uint32_t cobID = master.Read<uint32_t>(cobConfigIndexForNode, 1, error) & 0x000007ff;
				if (error)
					return;  // no PDO config on master side -> no COB ID.

				// 3) Check if another PDO config for the same COB ID exists:
				for (uint16_t otherCobConfigIndex = 0x1800; otherCobConfigIndex <= 0x19ff; otherCobConfigIndex++)
				{
					if (otherCobConfigIndex == cobConfigIndexForNode)
						continue;

					uint32_t otherCobID = master.Read<uint32_t>(otherCobConfigIndex, 1, error) & 0x000007ff;
					if (error)
						return;

					if (otherCobID == cobID)
					{
						// 4) Found another COB ID, read it's node config:
						uint16_t otherNodeConfigIndex = otherCobConfigIndex - 0x1800 + 0x5c00;
						uint32_t otherNodeConfig = master.Read<uint32_t>(otherNodeConfigIndex, 0 , error) & 0x0000ffff;
						if (error)
							return;

						if ((otherNodeConfig & 0x0000ff00) != 0x0100)
							return; // Entry is for a different RPDO index.

						// 5) determine which motor is the follower (the motor with the higher node ID)
						uint8_t otherNodeID = otherNodeConfig & 0x000000ff;
						if (otherNodeID < id())
						{
							// This instance becomes the following motor.
							m_followsNodeID = otherNodeID;
							dcfConfigMaster->getDriver(otherNodeID)->setFollowingNodeID(id());
							diag(DIAG_INFO, 0, "configureFollowerRelationship: 0x%02x follows 0x%02x", id(), otherNodeID);
						}
						else if (otherNodeID > id())
						{
							// This instance becomes the main motor.
							m_followingNodeID = otherNodeID;
							dcfConfigMaster->getDriver(otherNodeID)->setFollowsNodeID(id());
							diag(DIAG_INFO, 0, "configureFollowerRelationship: 0x%02x follows 0x%02x", otherNodeID, id());
						}
						return;
					}
				}
			}
		}
	}
}

void DCFDriver::OnState(lely::canopen::NmtState st) noexcept
{
	diag(DIAG_INFO, 0, "OnState: node: 0x%02x NMT state: 0x%02x", id(), st);
	if (m_nmtStateChangedCallback != nullptr)
		m_nmtStateChangedCallback(st);
}

void DCFDriver::OnEmcy(uint16_t emergencyErrorCode, uint8_t errorRegister, uint8_t manufSpecificError[]) noexcept
{
	m_emergencyOccured = (emergencyErrorCode != 0);
	if (m_errorCallback != nullptr && m_emergencyOccured)
	{
		std::stringstream message;
		message << boost::format("EMERGENCY: code: 0x%04x error register: 0x%02x manufacturer specific message (hex): ") % emergencyErrorCode % static_cast<int>(errorRegister);
		for (int i = 0; i < 5; i++)
			message << boost::format("%02x ") % static_cast<int>(manufSpecificError[i]);
		message << " string: ";
		for (int i = 0; i < 5; i++)
		{
			if (manufSpecificError[i] >= 32 && manufSpecificError[i] <= 127)
			{
				message << manufSpecificError[i];
			}
			else
			{
				message << ".";
			}
		}
		m_errorCallback(emergencyErrorCode, message.str());
	}
}

void DCFDriver::OnBoot(lely::canopen::NmtState st, char es, const std::string &what) noexcept
{
	diag(DIAG_INFO, 0, "OnBoot: NMT node: 0x%02x state: 0x%02x es: 0x%02x", id(), st, es);

	// check for boot errors and report via callback.
	if (es != 0 && m_errorCallback != nullptr)
	{
		std::stringstream message;
		message << boost::format("In NMT state 0x%02x: CiA-302 slave boot error status: %c (%s)") % static_cast<int>(st) % es % what;
		if (es == 'B')
			m_errorCallback(AdditionalErrorCode::NODE_MISSING, message.str());
		else
			m_errorCallback(AdditionalErrorCode::NODE_BOOT_FAILED, message.str());
	}
}

void DCFDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
	// Execute on_rpdo_mapped callback if registered.
	const auto& idxIter = on_rpdo_mapped.find(idx);
	if (idxIter != on_rpdo_mapped.end())
	{
		const auto& subidxIter = idxIter->second.find(subidx);
		if (subidxIter != idxIter->second.end())
		{
			const auto& function = subidxIter->second;
			if (function != nullptr)
				function();
		}
	}

	// Handle follower relationship.
	if (m_followsNodeID > 0)
	{
		auto* dcfConfigMaster = dynamic_cast<DCFConfigMaster*>(&master);
		if (dcfConfigMaster != nullptr)
		{
			dcfConfigMaster->getDriver(m_followsNodeID)->onFollowerRpdoWrite(idx, subidx);
		}
	}
}

std::string DCFDriver::ConfigErrorCategory::message(int condition) const
{
	std::stringstream result;
	switch(m_step)
	{
	case DCFDriver::ConfigErrorCategory::READ_LOCAL_VALUE:
		result << "While reading the local SDO value from ";
		break;
	case DCFDriver::ConfigErrorCategory::READ_REMOTE_SDO:
		result << "While Reading from SDO ";
		break;
	case DCFDriver::ConfigErrorCategory::WRITE_REMOTE_SDO:
		result << "While Writing to SDO ";
		break;

	}
	result << boost::format("0x%04x/0x%02x: 0x:%x (%s)") % m_index % static_cast<int>(m_subIndex) % m_originalErrorCode.value() % m_originalErrorCode.message();

	return result.str();
}
