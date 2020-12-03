/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the declaration of a generic driver for a CANOpen slave which can be configured with textual DCF files.
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
#include <lely/can/net.hpp>
#include <lely/coapp/driver.hpp>
#include "DCFDriverConfig.h"

class DCFDriverConfig;

/**
 * from CiA-301: Some standard SDO adresses.
 */
enum StandardSDO : uint16_t
{
	RECEIVE_PDO_CONTROL_START  = 0x1400,
	RECEIVE_PDO_CONTROL_END    = 0x15FF,
	RECEIVE_PDO_MAPPING_START  = 0x1600,
	RECEIVE_PDO_MAPPING_END    = 0x17FF,
	TRANSMIT_PDO_CONTROL_START = 0x1800,
	TRANSMIT_PDO_CONTROL_END   = 0x19FF,
	TRANSMIT_PDO_MAPPING_START = 0x1A00,
	TRANSMIT_PDO_MAPPING_END   = 0x1BFF,
};

/**
 * @brief The AdditionalErrorCode enum defines additional error codes in the mnufacturer specific range.
 */
enum AdditionalErrorCode: uint16_t
{
	NODE_CONFIGURATION_FAILED = 0xAF01,
	NODE_BOOT_FAILED = 0xAF02,
	READ_ERROR_FAILED = 0xAF03,
	NODE_MISSING = 0xAF04,
	WRTIE_TO_NODE_ERROR = 0xAF05,
	FIRMWARE_UPDATE_FAILED = 0xAF06,
	OTHER_MOTOR_HAD_ERROR = 0xAFFF
};


/**
 * @brief The DCFDriver class represents a driver which represants a node that is configured by a DCF file.
 */
class DCFDriver : public lely::canopen::BasicDriver
{
public:
	/**
	 * @brief A function with this signature can be called in case of an error.
	 * The uint16_t contains e.g. the CANopen emergency error code or some internal error code (0xAF00 - 0xAFFF).
	 * The string contains an error message.
	 */
	typedef std::function<void(uint16_t, const std::string&)> ErrorCallback;

	/**
	 * @brief ClearConfigurationStrategy defines a process to reset the configuration values of a node to default values.
	 * Once this is done, the strategy should call the given callback.
	 */
	typedef std::function<void (std::function<void (::std::error_code)> callback)> ClearConfigurationStrategy;

	/**
	 * @brief NmtStateChangedCallback is called when OnState() is called.
	 */
	typedef std::function<void (lely::canopen::NmtState)> NmtStateChangedCallback;

	/**
	 * @brief Creates a new DCFDriver from the given config.
	 * @param exec The execution stuff to use
	 * @param m the Master to run with.
	 * @param config The config.
	 */
	DCFDriver(ev_exec_t *exec, lely::canopen::BasicMaster &m, std::shared_ptr<DCFDriverConfig> config);

	virtual void OnState(lely::canopen::NmtState st) noexcept override;

	virtual void OnConfig(::std::function<void (::std::error_code)> res) noexcept override;

	virtual void OnEmcy(uint16_t emergencyErrorCode, uint8_t errorRegister, uint8_t manufSpecificError[5]) noexcept override;

	virtual void OnBoot(lely::canopen::NmtState st, char es, const std::string &what) noexcept override;

	virtual void onSystemBootCompleted() noexcept {}

	/**
	 * @brief Configures the error callback in case of an error.
	 * @param callback to call on an error.
	 */
	void setErrorCallback(ErrorCallback callback) {m_errorCallback = callback;}

	/**
	 * @brief Injects an external strategy to clear the configuration, e.g. write to Object 0x1011 (see CiA-301) + node reset.
	 * @param strategy The strategy with a callback which is called once the configuration is back on the default values.
	 */
	void setClearConfigurationStrategy(ClearConfigurationStrategy strategy) {m_clearConfigurationStrategy = strategy;}

	/**
	 * @brief Returns wether a custom clearConfigurationStrategy was configured (see setClearConfigurationStrategy)
	 */
	bool hasCustomClearConfigurationStrategy() const {return m_clearConfigurationStrategy != nullptr;}

	/**
	 * @brief setNmtStateChangedCallback sets a callback which is called when OnState() is called / when the NMT state changes.
	 * @param callback
	 */
	void setNmtStateChangedCallback(NmtStateChangedCallback callback) {m_nmtStateChangedCallback = callback;}

	/**
	 * @brief The ConfigErrorCategory class adds information about the SDO index/subindex which caused an error.
	 */
	class ConfigErrorCategory : public std::error_category
	{
	public:
		enum Step
		{
			READ_LOCAL_VALUE,
			READ_REMOTE_SDO,
			WRITE_REMOTE_SDO
		};

		ConfigErrorCategory(Step step, uint16_t index, uint8_t subIndex, std::error_code originalErrorCode) :
			std::error_category(),
			m_step(step),
			m_index(index),
			m_subIndex(subIndex),
			m_originalErrorCode(originalErrorCode) {}

		ConfigErrorCategory(const ConfigErrorCategory& other) :
			std::error_category(),
			m_step(other.m_step),
			m_index(other.m_index),
			m_subIndex(other.m_subIndex),
			m_originalErrorCode(other.m_originalErrorCode) {}


		virtual const char* name() const noexcept {return "Config Error";}
		virtual std::string message( int condition ) const;

	private:
		Step m_step;
		uint16_t m_index;
		uint8_t m_subIndex;
		std::error_code m_originalErrorCode;
	};

	/**
	 * @brief onSDOChanged is called when a SDO of the master has changed, e.g. by PDO communication.
	 * @param index    The index of the changing SDO.
	 * @param subIndex The sub index of the changing SDO.
	 */
	virtual void onMasterSDOChanged(uint16_t index, uint8_t subIndex) = 0;

	/// Add a function for a certain master SDO index and sub index here to get notified when it changes.
	std::map<uint16_t /* master SDO index */, std::map<uint8_t /* master SDO sub index */, std::function<void()>>> on_rpdo_mapped;

protected:
	/// The config.
	std::shared_ptr<DCFDriverConfig> m_config;

	/// Set if another node reacts on the same PDOs as the represented node.
	uint8_t m_followingNodeID;
	/// Set if the represented node reacts on the same PDOs as the given node ID.
	uint8_t m_followsNodeID;

	void setFollowingNodeID(uint8_t nodeID) {m_followingNodeID = nodeID;}
	void setFollowsNodeID(uint8_t nodeID) {m_followsNodeID = nodeID;}

	/// Callback to report errors.
	ErrorCallback m_errorCallback;

	/// Set to true if an EMCY was received.
	bool m_emergencyOccured;

	virtual void OnRpdoWrite (uint16_t idx, uint8_t subidx) noexcept override;

	/// Called by OnRpdoWrite() if a write to the follower was detected.
	virtual void onFollowerRpdoWrite  (uint16_t /* idx */, uint8_t /* subidx */) noexcept {}

private:
	// DCF Text File based configuration:
	void configure(std::function<void (std::error_code)> res);
	DCFDriverConfig::ObjectsList m_sdosToConfigure;
	void configureSDO(const DCFDriverConfig::ObjectsList::iterator objectToSend, ::std::function<void(std::error_code)> onCompletedFunction);
	void configurePDO(const DCFDriverConfig::ObjectsList::iterator objectToSend, ::std::function<void(std::error_code)> onCompletedFunction);
	void configureFollowerRelationship(const DCFDriverConfig::ObjectsList::iterator objectToSend);
	void configureParameterSDO(const DCFDriverConfig::ObjectsList::iterator objectToSend, const std::vector<uint8_t>::iterator subIndexToSend, ::std::function<void(std::error_code)> onCompletedFunction, bool iterateSubIndicesOnly = false);

	// YAML / DCF BIN File based configuration
	void configureFollowerRelationship();

	ClearConfigurationStrategy m_clearConfigurationStrategy;
	NmtStateChangedCallback m_nmtStateChangedCallback;
};
