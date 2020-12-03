/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the declaration of a CANOpen master which can be configured with textual DCF files.
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
#include <map>
#include <set>
#include <lely/coapp/master.hpp>

class DCFDriver;
class DCFDriverConfig;

/**
 * @brief DCFDriverFactoryFunction is used to create concrete drivers depending on the config.
 */
typedef std::function<std::shared_ptr<DCFDriver>(std::shared_ptr<DCFDriverConfig>)> DCFDriverFactoryFunction;

/**
 * @brief The DCFConfigMaster class is the master of a system which is configured by DCF files.
 */
class DCFConfigMaster : public lely::canopen::AsyncMaster
{
public:
	friend DCFDriver;

	/**
	 * @brief Creates a new master.
	 * @param timer
	 * @param chan
	 * @param dcf_txt
	 * @param exec
	 */
	DCFConfigMaster(lely::io::TimerBase& timer, lely::io::CanChannelBase& chan,	const ::std::string& dcf_txt, ev_exec_t* exec);

	/**
	 * @brief Configures the drivers given by the master config.
	 */
	void configureDrivers();

	/**
	 * Set the factory to create new drivers.
	 * @brief setDriverFactory
	 * @param factory
	 */
	void setDriverFactory(DCFDriverFactoryFunction factory) {m_driverFactory = factory;}

	/**
	 * @brief Get the driver for the given node ID or nullptr if it was not registered.
	 * @param nodeID
	 * @return
	 */
	std::shared_ptr<DCFDriver> getDriver(uint8_t nodeID);

	/**
	 * @brief setBootCompletedCallback sets a callback wich is called once the boot up of each node is completed and with ID=0 when all nodes have been booted.
	 * @param callback
	 */
	void setBootCompletedCallback(std::function<void(uint8_t)> callback) {m_bootCompletedCallback = callback;}

	/**
	 * @brief setLoadConfigStartedCallback sets a callback function which is called when the loading of the configuration begins.
	 * @param callback
	 */
	void setLoadConfigStartedCallback(std::function<void(uint8_t)> callback) {m_loadConfigStartedCallback = callback;}

	/**
	 * @brief setNodeConfigStartedCallback sets a callback function which is called when the configuration of the node begins.
	 * @param callback
	 */
	void setNodeConfigStartedCallback(std::function<void(uint8_t)> callback) {m_nodeConfigStartedCallback = callback;}

	/**
	 * @brief Needed by DCFDriver to resolve the follower/following relationships.
	 * @param cobID
	 * @return
	 */
	uint8_t getFirstNodeIDUsing_RPDO_COB_ID(uint32_t cobID);
	/**
	 * @brief Needed by DCFDriver to resolve the follower/following relationships.
	 * @param nodeID
	 * @param cobID
	 */
	void setFirstNodeIDUsing_RPDO_COB_ID(uint8_t nodeID, uint32_t cobID);

	/**
	 * @brief resets all slaves after an error occured with NMT RESET, when the system is already initialized.
	 */
	void reset();

	/**
	 * @brief getSoftwareFileForSlave returns the firmware file name stored in SDO 1F58 for the given slave ID (the sub index to 1F58)
	 * @param nodeID The node ID to get the information for.
	 * @param error Passed error channel to GetUploadFile() which actually reads the file name.
	 * @return
	 */
	const char *getSoftwareFileForSlave(uint8_t nodeID, std::error_code& error);

protected:
	void OnBoot(uint8_t id, lely::canopen::NmtState st, char es,
				const ::std::string& what) noexcept override;

	void OnCommand(lely::canopen::NmtCommand cs) noexcept override;
	void OnConfig(uint8_t id) noexcept override;
	void OnState(uint8_t id, lely::canopen::NmtState st) noexcept override;

private:
	void initializeDevicesFromTextualDCF();
	void initializeDevicesForBinaryDCF();
	void registerDriver(std::shared_ptr<DCFDriver> driver);

	std::map<uint8_t, std::shared_ptr<DCFDriver>> m_drivers;
	std::map<uint32_t /* COB ID */, uint8_t /* node ID */> m_firstNodeIDUsing_RPDO_COB_ID;
	std::set<uint8_t> m_devicesToBoot;
	std::function<void(uint8_t)> m_bootCompletedCallback;
	DCFDriverFactoryFunction m_driverFactory;
	ev_exec_t *m_exec;
	std::function<void(uint8_t)> m_loadConfigStartedCallback;
	std::function<void(uint8_t)> m_nodeConfigStartedCallback;
};


