/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the declaration of the configuration class for a DCFDriver.
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

#pragma once
#include <lely/coapp/device.hpp>
#include <lely/coapp/driver.hpp>
#include <lely/coapp/sdo.hpp>

/**
 * @brief The DCFDriverConfig class contains a configuration for a certain node. This config is read from a dcf file.
 */
class DCFDriverConfig : public lely::canopen::Device
{
public:
	/**
	 * @brief ObjectsList represents the SDOs which are explicitly set in the DCF config.
	 */
	typedef std::vector<std::tuple<uint16_t, std::vector<uint8_t>>> ObjectsList;

	/**
	 * @brief Creates a new config by reading the given dcf file.
	 * @param textualDcfFileName The textual DCF file to parse.
	 * @param binaryDcfFileName The binary DCF file to parse.
	 * @param defaultNodeID The node ID in the master.dcf for which the file was configured.
	 */
	DCFDriverConfig(const std::string& textualDcfFileName, const std::string &binaryDcfFileName, uint8_t defaultNodeID);
	~DCFDriverConfig() {}

	/**
	 * @brief getSDOIndicesForDriverConfiguration return the explicitly configured SDOs (ParameterValue is set)
	 * @return
	 */
	ObjectsList getSDOIndicesForDriverConfiguration() const;
	/**
	 * @brief getTypeOfObject returns the type of the given SDO.
	 * @param sdoIndex
	 * @param sdoSubIndex
	 * @return
	 */
	uint16_t getTypeOfObject(uint16_t sdoIndex, uint8_t sdoSubindex) const;

	/**
	 * @brief getDefaultNodeID returns the node ID for which this configuration was set in the master.dcf
	 * @return
	 */
	uint8_t getDefaultNodeID() const {return m_defaultNodeID;}


	// const lely::canopen::SdoDownloadRequest<std::function<void(std::error_code)>>* getBinaryDCF();
	const std::string& getBinaryDcfFile() {return m_binaryDcfFile;}

private:
	uint8_t m_defaultNodeID;
	// std::unique_ptr<lely::canopen::SdoDownloadRequest<std::function<void(std::error_code)>>> m_binaryDCF;
	std::string m_binaryDcfFile;
};

