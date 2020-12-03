/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the declaration of a CANOpen CiA-402 motor driver.
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
#include <deque>
#include "DCFDriver.h"

/**
 * @brief The MotorDriver class controls a CiA-402 compiant motor.
 */
class MotorDriver : public DCFDriver
{

public:
	MotorDriver(ev_exec_t *exec, lely::canopen::BasicMaster &m, std::shared_ptr<DCFDriverConfig> config);

	/// Constants for the homing method. See SDO 0x6098 in the CiA-402 spec.
	/// The home() method accepts integers for it since custom homing modes are possible too.
	enum PredefinedHomingMethod: int8_t
	{
		HOMING_BACKWARD_RISING_EDGE    = 19, // HM_UP_HS_1
		HOMING_BACKWARD_FALLING_EDGE   = 20, // HM_UP_HS_2
		HOMING_FORWARD_RISING_EDGE     = 21, // HM_DW_HS_1
		HOMING_FORWARD_FALLING_EDGE    = 22, // HM_DW_HS_2
		HOMING_FORWARD_MARKER_CYCLE    = 33, // Marker cycle CCW towards decreasing positions
		HOMING_BACKWARD_MARKER_CYCLE   = 34, // Marker cycle CW towards increasing positions
		UNDEFINED_HOMING_METHOD        = 0
	};

	/// Constants for the move mode. See SDO 0x6040 bit 6 in the CiA-402 spec.
	/// The move() method accepts integers for it since custom move modes are possible too.
	enum MoveMode : uint16_t
	{
		ABSOLUTE            = 0,
		RELATIVE            = 0x0040,
		UNDEFINED_MOVE_MODE = 0xFFFF
	};

	/// The predefined SDO constants for various motor operations.
	enum MotorSDO: uint16_t
	{
		MOTOR_CONTROLWORD = 0x6040,
		MOTOR_OPERATIONMODE = 0x6060,
		MOTOR_POSITION = 0x607a,
		MOTOR_VELOCITY = 0x6081,
		MOTOR_ACCELERATION = 0x6083,
		MOTOR_DECELERATION = 0x6084,
		MOTOR_STATUSWORD = 0x6041
	};

	/**
	 *  This strategy is used to set an SDO on the motor side, e.g. via SDO communication (slow) or PDO communicaion.
	 */
	template<typename T>
	using SetterStrategy = std::function<void (T value, std::function<void (std::error_code)> callback)>;

	/**
	 * This function is used to determine if a change of a master SDO means that a new status word has arrived.
	 */
	using IsStatusWordCheck = std::function<bool (uint16_t masterIndex, uint8_t masterSubIndex, uint8_t nodeID)>;

	/**
	 * This struct defines the strategy to communicate with the drive.
	 */
	struct CommunicationConfig
	{
		friend MotorDriver;
	public:
		/// Sets the method to configure the control word (See SDO 0x6040 in the CiA 402 spec)
		void setMotorControlWordSetter(const SetterStrategy<uint16_t> &MotorControlwordSetter);
		/// Sets the method to configure the operation mode (See SDO 0x6060 in the CiA 402 spec)
		void setMotorOperationModeSetter(const SetterStrategy<uint8_t> &MotorOperationmodeSetter);
		/// Sets the method to configure the operation mode (See SDO 0x607A in the CiA 402 spec)
		void setMotorPositionSetter(const SetterStrategy<int32_t> &MotorPositionSetter);
		/// Sets the method to configure the operation mode (See SDO 0x6081 in the CiA 402 spec)
		void setMotorVelocitySetter(const SetterStrategy<uint32_t> &MotorVelocitySetter);
		/// Sets the method to configure the operation mode (See SDO 0x6083 in the CiA 402 spec)
		void setMotorAccelerationSetter(const SetterStrategy<uint32_t> &MotorAccelerationSetter);
		/// Sets the method to configure the operation mode (See SDO 0x6084 in the CiA 402 spec)
		void setMotorDecelerationSetter(const SetterStrategy<uint32_t> &MotorDecelerationSetter);
		/// Sets a function which checks if the change of a certain master SDO means that the staus word (See SDO 0x6041 in the CiA 402 spec) on the corresponding motor has changed.
		/// Only necessary in screnarios where a custom PDO mapping (not generated by dcfgen) is used.
		void setIsStatusWordCheckForMasterSDOChange(const IsStatusWordCheck &isStatusWordCheckForMasterSDOChange);

	private:
		SetterStrategy<uint16_t> motorControlWordSetter;
		SetterStrategy<int8_t>   motorOperationModeSetter;
		SetterStrategy<int32_t>  motorPositionSetter;
		SetterStrategy<uint32_t> motorVelocitySetter;
		SetterStrategy<uint32_t> motorAccelerationSetter;
		SetterStrategy<uint32_t> motorDecelerationSetter;
		IsStatusWordCheck        isStatusWordCheckForMasterSDOChange;
	};

	/**
	 * @brief home Triggers the homing of the motor.
	 * The motor firmware looks for it's reference sensor with the given method. When found, the motor is on the given offset position.
	 * A move to 0 is necessary to move to the offset.
	 * @param method The homing method to use. (See SDO 0x6098 in the CiA 402 spec)
	 * @param researchSpeed The speed to search for the homing marker. (See SDO 0x6099 in the CiA 402 spec)
	 * @param releaseSpeed The speed to search for the zero. (See SDO 0x6099 in the CiA 402 spec)
	 * @param accel The acceleration to use. (See SDO 0x609A in the CiA 402 spec)
	 * @param offset The offset position after the homing. (See SDO 0x607C in the CiA 402 spec)
	 * @param callbackOnIDLE The callback to call after the homing procedure has finished.
	 */
	void home(int8_t method, uint32_t researchSpeed, uint32_t releaseSpeed, uint32_t accel, int32_t offset, std::function<void()> callbackOnIDLE = nullptr);

	/**
	 * @brief move Triggers the movement of the motor.
	 * It will use the configured strategies to move into a certain position. The strategies define the way to communicate with the motor.
	 * @param mode The mode to use: See MoveMode (ABSOLUTE or RELATIVE), might be even vendor specific since it is ORed with the control word (See SDO 0x6040 in the CiA 402 spec).
	 * @param position The traget position in steps to move to. (See SDO 0x607A in the CiA 402 spec)
	 * @param speed The velocity to use. (See SDO 0x6081 in the CiA 402 spec)
	 * @param accel The acceleration to use. (See SDO 0x6083 in the CiA 402 spec)
	 * @param deaccel The deacceleration to use. (See SDO 0x6084 in the CiA 402 spec)
	 * @param callbackOnIDLE The callback to call after the move procedure has finished.
	 */
	void move(uint16_t mode, int32_t position, uint32_t speed, uint32_t accel, uint32_t deaccel, std::function<void()> callbackOnIDLE = nullptr);

	/**
	* @brief recoverFromFault Brings the motor back to normal operation after a fault.
	* @param callbackOnIDLE The callback to call when the motor is back.
	*/
	void recoverFromFault(std::function<void()> callbackOnIDLE = nullptr);

	/**
	 * @brief setCommunicationConfig Configures how to communicate with the motors for a certain action.
	 * @param config The configuration to use.
	 */
	void setCommunicationConfig(CommunicationConfig config) {m_communicationConfig = config;}

	/**
	 * Create a strategy which sets an SDO on the motor side via SDO communication. Not suitable for follower relationships.
	 */
	template<typename T>
	SetterStrategy<T> createSDOSetter(MotorSDO sdo)
	{
		return [sdo, this](T value, std::function<void (std::error_code)> callback)
		{
			SubmitWrite<T>(sdo, 0, std::forward<T>(value), [callback](uint8_t /*id */, uint16_t /* idx */, uint8_t /* subidx */, ::std::error_code ec)
			{
				if (callback != nullptr)
					callback(ec);
			});
		};
	}

	/**
	 * Creates a strategy which sets an SDO on the motor side via PDO communication.
	 * The contents of the PDO are filled from the given SDO of the master.
	 * If tpdo >= 0, the given TPDO is triggered.
	 */
	template<typename T>
	SetterStrategy<T> createMasterSDOSetter(uint16_t masterIndex, uint8_t masterSubIndex, int tpdo)
	{
		return [masterIndex, masterSubIndex, tpdo, this](T value, std::function<void (std::error_code)> callback)
		{
			std::error_code error;
			master.Write<T>(masterIndex, masterSubIndex, value, error);
			if (!error && tpdo >= 0)
				master.TpdoEvent(tpdo);
			if (callback != nullptr)
				callback(error);
		};
	}

	/**
	 * Create a strategy which sets an SDO on the motor side via PDO communication through mapped TPDOs.
	 */
	template<typename T>
	SetterStrategy<T> createMappedTpdoSetter(MotorSDO sdo, bool writeEvent)
	{
		return [sdo, writeEvent, this](T value, std::function<void (std::error_code)> callback)
		{
			std::error_code error;
			tpdo_mapped[sdo][0].Write(value, error);
			if (writeEvent)
				tpdo_mapped[sdo][0].WriteEvent(error);
			if (callback != nullptr)
				callback(error);
		};
	}

	virtual void OnCommand (lely::canopen::NmtCommand cs) noexcept override;
	virtual void OnState(lely::canopen::NmtState st) noexcept override;

	virtual void onSystemBootCompleted() noexcept override;

protected:
	virtual void OnConfig(::std::function<void (::std::error_code)> res) noexcept override;

	virtual void OnBoot(lely::canopen::NmtState st, char es, const ::std::string &what) noexcept override;

	virtual void onMasterSDOChanged(uint16_t index, uint8_t subIndex) override;
	virtual void OnRpdoWrite (uint16_t idx, uint8_t subidx) noexcept override;
	virtual void onFollowerRpdoWrite  (uint16_t idx, uint8_t subidx) noexcept override;

private:
	/**
	 * @brief The State enum represents the internal state of the driver. This state has nothing to do with the CiA-402 state.
	 */
	enum State
	{
		INITIAL_STATE,
		INITIAL_POWER_ON,
		INITIAL_POWER_OFF,
		CYCLE_POWER_SHUTDOWN,
		POWER_ON_DISABLE_OPERATION,
		IDLE,
		PREPARE_MOVE,
		READY_TO_MOVE,
		MOVING,
		PREPARE_HOMING,
		READY_FOR_HOMING,
		HOMING,
		FAULT_STATE,
		FAULT_RESET,
		NODE_RESET
	};

	enum StatusWordFlags
	{
		READY_TO_SWITCH_ON       = 0x0001,
		SWITCHED_ON              = 0x0002,
		OPERATION_ENABLED        = 0x0004,
		FAULT                    = 0x0008,
		VOLTAGE_ENABLED          = 0x0010,
		QUICK_STOP               = 0x0020,
		SWITCH_ON_DISABLED       = 0x0040,
		WARNING                  = 0x0080,
		MANUFACTURER_SPECIFIC1   = 0x0100,
		REMOTE                   = 0x0200,
		TARGET_REACHED           = 0x0400,
		INTERNAL_LIMIT_ACTIVE    = 0x0800,
		OPERATION_MODE_SPECIFIC1 = 0x1000,
		OPERATION_MODE_SPECIFIC2 = 0x2000,
		MANUFACTURER_SPECIFIC2   = 0x4000,
		MANUFACTURER_SPECIFIC3   = 0x8000
	};

	void prepareHoming(int8_t method, uint32_t researchSpeed, uint32_t releaseSpeed, uint32_t accel, int32_t offset);

	void prepareMove();
	void executeMove();

	bool isSetterOK(const std::error_code& ec, const std::string& message);

	CommunicationConfig m_communicationConfig;

	std::chrono::high_resolution_clock::time_point m_jobStartedAt;
	std::chrono::high_resolution_clock::time_point m_jobFinishedAt;

	/// The state of this node if the node is not a following node, else IDLE.
	State m_mainNodeState = IDLE;
	/// The state of the following node if the node has a following node, else IDLE.
	State m_followingNodeState = IDLE;
	/// The aggregated state (identical with m_mainNodeState if the node has no follower).
	State m_state = INITIAL_STATE;
	/// The original CiA-402 state
	uint16_t m_statusWord = 0;

	void handleStatusWordChange(uint16_t statusWord, bool statusWordOfFollowerChanged);

	State determineStateFromStatusWord(State currentState, uint16_t statusWord, uint8_t nodeID);
	void setState(State newState);
	std::string stateToString(State state);
	uint16_t m_currentMoveMode = 0;
	int32_t m_moveToPosition = 0;
	uint32_t m_moveSpeed = 0;
	uint32_t m_moveAcceleration = 0;
	uint32_t m_moveDeacceleration = 0;

	void handleFault();
	void performFaultReset();
	void retriggerFaultReset() noexcept;

	lely::canopen::NmtCommand m_masterNmtState = lely::canopen::NmtCommand::STOP;
	lely::canopen::NmtState m_nodeNmtState = lely::canopen::NmtState::STOP;
	void handleInitialStateSwitching();

	std::deque<std::function<void()>> m_callbackOnIDLE;
	std::mutex m_callbacksOnIdleMutex;
	void addCallbackOnIdle(std::function<void()> callback);
	void processOldestCallbackOnIdle();

};