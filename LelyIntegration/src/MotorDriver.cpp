/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the implementation of a CANOpen CiA-402 motor driver.
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

#include <sstream>

#include <boost/format.hpp>

#include <lely/coapp/master.hpp>
#include <lely/util/diag.h>

#include "DCFConfigMaster.h"

#include "MotorDriver.h"

MotorDriver::MotorDriver(ev_exec_t *exec, lely::canopen::BasicMaster &m, std::shared_ptr<DCFDriverConfig> config) :
	DCFDriver(exec, m, config)
{
}

void MotorDriver::home(int8_t method, uint32_t researchSpeed, uint32_t releaseSpeed, uint32_t accel, int32_t offset, std::function<void ()> callbackOnIDLE)
{
	if (m_state == IDLE)
	{
		addCallbackOnIdle(callbackOnIDLE);
		prepareHoming(method, researchSpeed, releaseSpeed, accel, offset);
	}
	else
	{
		addCallbackOnIdle([this, method, researchSpeed, releaseSpeed, accel, offset]()
		{
			GetExecutor().post([this, method, researchSpeed, releaseSpeed, accel, offset]()  // No recursion for setState if the callback itself changes the state.
			{
				prepareHoming(method, researchSpeed, releaseSpeed, accel, offset);
			});
		});
		addCallbackOnIdle(callbackOnIDLE);
	}
}

void MotorDriver::move(uint16_t mode, int32_t position, uint32_t speed, uint32_t accel, uint32_t deaccel, std::function<void ()> callbackOnIDLE)
{
	m_currentMoveMode = mode;
	m_moveToPosition = position;
	m_moveSpeed = speed;
	m_moveAcceleration = accel;
	m_moveDeacceleration = deaccel;

	auto prepareMove = [this]()
	{
		if (m_followingNodeID != 0)
			m_mainNodeState = PREPARE_MOVE;
		setState(PREPARE_MOVE);
	};

	if (m_state == IDLE)
	{
		addCallbackOnIdle(callbackOnIDLE);
		prepareMove();
	}
	else
	{
		addCallbackOnIdle([this,prepareMove]()
		{
			GetExecutor().post([prepareMove]()  // No recursion for setState if the callback itself changes the state.
			{
				prepareMove();
			});
		});
		addCallbackOnIdle(callbackOnIDLE);
	}
}

void MotorDriver::recoverFromFault(std::function<void ()> callbackOnIDLE)
{
	diag(DIAG_INFO, 0, "recoverFromFault: Node 0x%02x: Recovering in state %s", id(), stateToString(m_state).c_str());
	addCallbackOnIdle(callbackOnIDLE);
	if (m_state == FAULT_STATE)
	{
		setState(NODE_RESET);
		// The fault reset of the CiA-402 state machine is done in OnBoot() once the NMT reset was done.
	}
	else if (m_state == FAULT_RESET)
	{
		// The fault reset is already in progress. When done, the motor becomes IDLE. This triggers the callback.
		// Watchdog if the motor is hanging in the fault reset:
		SubmitWait(std::chrono::milliseconds(1000),  [this](std::error_code error)
		{
			if (!error)
				retriggerFaultReset();
		});
	}
	else
	{
		if (m_state != IDLE)
		{
			// TODO: do nothing or determine if a power cylce is needed?
			// setState(CYCLE_POWER_SHUTDOWN);  // Ensure that we are in IDLE state. Call the callback from setState() in this case.
		}
		else
		{
			// Already in IDLE state: nothing to do.
			processOldestCallbackOnIdle();
		}
	}
}

void MotorDriver::performFaultReset()
{
	// The logic here is similar to handleInitialStateSwitching():
	State recoveryFrom = determineStateFromStatusWord(INITIAL_STATE, m_statusWord, id());
	if (recoveryFrom == FAULT_STATE)
	{
		SubmitWrite<int16_t> (0x6040, 0, 0x0080, nullptr);  // Fault Reset, CYCLE_POWER_SHUTDOWN is triggered through determineStateFromStatusWord().
	}
	else if (recoveryFrom == INITIAL_POWER_ON)
	{
		GetExecutor().post([this]()  // No recursion for setState, let the currently running setState() reach it's end.
		{
			setState(MotorDriver::CYCLE_POWER_SHUTDOWN);
		});
	}
	else if (recoveryFrom == INITIAL_POWER_OFF)
	{
		GetExecutor().post([this]()  // No recursion for setState, let the currently running setState() reach it's end.
		{
			setState(MotorDriver::POWER_ON_DISABLE_OPERATION);
		});
	}
}


void MotorDriver::retriggerFaultReset() noexcept
{
	if (m_state == FAULT_RESET)
	{
		setState(NODE_RESET);
	}
}

void MotorDriver::handleInitialStateSwitching()
{
	if (m_masterNmtState == lely::canopen::NmtCommand::START && m_nodeNmtState == lely::canopen::NmtState::START)
	{
		diag(DIAG_INFO, 0, "handleInitialStateSwitching: Node 0x%02x", id());
		// m_state == FAULT_STATE: try fault recovery directly from here
		// m_state == NODE_RESET:  continue with fault reset if a node reset was necessary.
		if (m_state == FAULT_STATE || m_state == NODE_RESET)
		{
			setState(FAULT_RESET);  // command to clear the fault flag of the motor is sent from performFaultReset().
		}
		else if (m_state == INITIAL_POWER_ON)
		{
			setState(CYCLE_POWER_SHUTDOWN);
		}
		else if (m_state == INITIAL_POWER_OFF)
		{
			setState(POWER_ON_DISABLE_OPERATION);
		}
		// A similar logic exists in performFaultReset()
	}
}

void MotorDriver::addCallbackOnIdle(std::function<void ()> callback)
{
	std::unique_lock<std::mutex> lock(m_callbacksOnIdleMutex);
	m_callbackOnIDLE.push_front(callback);
}

void MotorDriver::processOldestCallbackOnIdle()
{
	std::unique_lock<std::mutex> lock(m_callbacksOnIdleMutex);
	if (!m_callbackOnIDLE.empty())
	{
		if (m_callbackOnIDLE.back() != nullptr)
			m_callbackOnIDLE.back()();
		m_callbackOnIDLE.pop_back();
	}
}

void MotorDriver::OnConfig(::std::function<void (std::error_code)> res) noexcept
{
	// Synchronize the motor state with the internal state before recoverFromFault() is called.
	DCFDriver::OnConfig([this,res](std::error_code ec)
	{
		if (!ec)
		{
			if (m_state == INITIAL_STATE)
			{
				// Read the initial motor state and set the internal state accordingly.
				SubmitRead<uint16_t>(0x6041, 0,
									 [this,res](uint8_t id, uint16_t /* idx */, uint8_t /* subidx */, ::std::error_code ec, uint16_t value)
				{
					m_statusWord = value;
					setState(determineStateFromStatusWord(m_state, value, id));
					res(ec);
				});
			}
			else
			{
				res(ec);
			}
		}
		else
		{
			std::stringstream message;
			message << "Failed to send the configuration to the motor: " << ec.message();
			if (m_errorCallback != nullptr)
				m_errorCallback(AdditionalErrorCode::NODE_CONFIGURATION_FAILED, message.str());
			res(ec);
		}
	});
}

void MotorDriver::OnBoot(lely::canopen::NmtState st, char es, const std::string &what) noexcept
{
	DCFDriver::OnBoot(st, es, what);


	if (es == 0)
	{
		// Work Around since OnState(NmtState::START) is currently not called.
		diag(DIAG_INFO, 0, "OnBoot: Node 0x%02x, cs: 0x%02x", id(), st);
		m_nodeNmtState = lely::canopen::NmtState::START;
		handleInitialStateSwitching();
	}
}

void MotorDriver::onSystemBootCompleted() noexcept
{
}

void MotorDriver::OnCommand(lely::canopen::NmtCommand cs) noexcept
{
	DCFDriver::OnCommand(cs);
	diag(DIAG_INFO, 0, "OnCommand: Node 0x%02x, cs: 0x%02x", id(), cs);
	m_masterNmtState = cs;
	handleInitialStateSwitching();
}

void MotorDriver::OnState(lely::canopen::NmtState st) noexcept
{
	DCFDriver::OnState(st);
	diag(DIAG_INFO, 0, "OnState: Node 0x%02x, cs: 0x%02x", id(), st);
	m_nodeNmtState = st;
	handleInitialStateSwitching();
}

void MotorDriver::prepareHoming(int8_t method, uint32_t researchSpeed, uint32_t releaseSpeed, uint32_t accel, int32_t offset)
{
	setState(PREPARE_HOMING);
	// master.Command(lely::canopen::NmtCommand::ENTER_PREOP, id());
	SubmitWrite<uint8_t> (0x6060, 0, 1,      nullptr);                                 // Profile is Position mode (needed for setting the homing offset)
	SubmitWrite<int8_t>  (0x6098, 0, std::forward<int8_t>(method), nullptr);           // Homing Method
	SubmitWrite<uint32_t>(0x6099, 1, std::forward<uint32_t>(researchSpeed),  nullptr); // Geschwindigkeit setzen: Suche nach Schalter
	SubmitWrite<uint32_t>(0x6099, 2, std::forward<uint32_t>(releaseSpeed),  nullptr);  // Geschwindigkeit setzen: Nullpunkt anfahren
	SubmitWrite<uint32_t>(0x609A, 0, std::forward<uint32_t>(accel),  nullptr);         // Beschleunigung während des Homing
	SubmitWrite<int32_t> (0x607C, 0, std::forward<int32_t>(offset),  nullptr);         // Offset nach Homing
	SubmitWrite<uint8_t> (0x6060, 0, 6,      nullptr);                                 // Profile is Homing Mode
	SubmitWrite<int16_t> (0x6040, 0, 0x000f, nullptr);                                 // Enable Operation (for some reason we need to cycle the operation for the homing to work reliable, and in IDLE operation is disabled)
}

void MotorDriver::prepareMove()
{
	// The following node is automagically triggered by the PDOs, so we have to set it's state manually here.
	if (m_followingNodeID != 0)
		m_followingNodeState = PREPARE_MOVE;

	m_communicationConfig.motorOperationModeSetter(1, [this](std::error_code error)
	{
		if (!isSetterOK(error, "While setting operation mode to 'Profile Position Mode'"))
			return;

		m_communicationConfig.motorControlWordSetter(7, [this](std::error_code error)
		{
			if (!isSetterOK(error, "While setting the control word to 'Disable Operation'"))
				return;

			m_communicationConfig.motorPositionSetter(std::forward<int32_t>(m_moveToPosition), [this](std::error_code error)
			{
				if (!isSetterOK(error, "While setting the position"))
					return;

				m_communicationConfig.motorVelocitySetter(std::forward<uint32_t>(m_moveSpeed), [this](std::error_code error)
				{
					if (!isSetterOK(error, "While setting the velocity"))
						return;

					m_communicationConfig.motorAccelerationSetter(std::forward<uint32_t>(m_moveAcceleration),[this](std::error_code error)
					{
						if (!isSetterOK(error, "While setting the acceleration"))
							return;

						m_communicationConfig.motorDecelerationSetter(std::forward<uint32_t>(m_moveDeacceleration), [this](std::error_code error)
						{
							if (!isSetterOK(error, "While setting the deceleration"))
								return;

							//Auxind starts on falling edge of bit4, servotronix on rising edge. If we put both on halt, they start at the same time when halt-bit gets falling edge
							m_communicationConfig.motorControlWordSetter(m_currentMoveMode | 0x011f, [this](std::error_code error)
							{
								isSetterOK(error, "While setting the control word to 'Enable Operation' + move mode");
							});
						});
					});
				});
			});
		});
	});
}

void MotorDriver::executeMove()
{
	// Trigger movement with PDO (needed to start two motors at the same time) after all SDOs were sent.
	// remove halt bit to start servortronix and auxind at the same time
	m_communicationConfig.motorControlWordSetter(m_currentMoveMode | 0x000f, [this](std::error_code error)
	{
		isSetterOK(error, "While switching the motor through the control word");
	});

	std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - m_jobStartedAt;
	diag(DIAG_INFO, 0, "submit SDOs callbacks finished after %fms", elapsed.count());
}

bool MotorDriver::isSetterOK(const std::error_code &error, const std::string &message)
{
	if (!error)
	{
		return true;
	}
	else
	{
		std::stringstream msg;
		msg << message << ": " << error << ": " << error.message();
		m_errorCallback(AdditionalErrorCode::WRTIE_TO_NODE_ERROR, message);
		return false;
	}
}

void MotorDriver::CommunicationConfig::setIsStatusWordCheckForMasterSDOChange(const IsStatusWordCheck &isStatusWordCheckForMasterSDOChange)
{
	this->isStatusWordCheckForMasterSDOChange = isStatusWordCheckForMasterSDOChange;
}

void MotorDriver::CommunicationConfig::setMotorDecelerationSetter(const SetterStrategy<uint32_t> &MotorDecelerationSetter)
{
	motorDecelerationSetter = MotorDecelerationSetter;
}

void MotorDriver::CommunicationConfig::setMotorAccelerationSetter(const SetterStrategy<uint32_t> &MotorAccelerationSetter)
{
	motorAccelerationSetter = MotorAccelerationSetter;
}

void MotorDriver::CommunicationConfig::setMotorVelocitySetter(const SetterStrategy<uint32_t> &MotorVelocitySetter)
{
	motorVelocitySetter = MotorVelocitySetter;
}

void MotorDriver::CommunicationConfig::setMotorPositionSetter(const SetterStrategy<int32_t> &MotorPositionSetter)
{
	motorPositionSetter = MotorPositionSetter;
}

void MotorDriver::CommunicationConfig::setMotorOperationModeSetter(const SetterStrategy<uint8_t> &MotorOperationmodeSetter)
{
	motorOperationModeSetter = MotorOperationmodeSetter;
}

void MotorDriver::CommunicationConfig::setMotorControlWordSetter(const SetterStrategy<uint16_t> &MotorControlwordSetter)
{
	motorControlWordSetter = MotorControlwordSetter;
}

void MotorDriver::onMasterSDOChanged(uint16_t index, uint8_t subIndex)
{
	if (m_communicationConfig.isStatusWordCheckForMasterSDOChange != nullptr &&
			(m_communicationConfig.isStatusWordCheckForMasterSDOChange(index, subIndex, id()) || m_communicationConfig.isStatusWordCheckForMasterSDOChange(index, subIndex, m_followingNodeID)))
	{
		uint16_t statusWord = master.Read<uint16_t>(index, subIndex);
		diag(DIAG_INFO, 0, "onMasterSDOChanged: main SDO 0x%04x/0x%02x = 0x%x", index, subIndex, statusWord);
		handleStatusWordChange(statusWord, m_communicationConfig.isStatusWordCheckForMasterSDOChange(index, subIndex, m_followingNodeID));
	}
}

void MotorDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
	DCFDriver::OnRpdoWrite(idx, subidx);
	if (idx == MOTOR_STATUSWORD && subidx == 0)
	{
		uint16_t statusWord = rpdo_mapped[idx][subidx];
		handleStatusWordChange(statusWord, /* statusWordOfFollowerChanged */ false);
	}
}

void MotorDriver::onFollowerRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
	if (idx == MOTOR_STATUSWORD && subidx == 0)
	{
		uint16_t statusWord = master.RpdoMapped(m_followingNodeID)[idx][subidx];
		handleStatusWordChange(statusWord, /* statusWordOfFollowerChanged */ true);
	}
}

void MotorDriver::handleStatusWordChange(uint16_t statusWord, bool statusWordOfFollowerChanged)
{
#if 0
	if (!statusWordOfFollowerChanged)
		diag(DIAG_INFO, 0, "handleStatusWordChange: status word for 0x%02x: 0x%04x", id(), statusWord);
	else
		diag(DIAG_INFO, 0, "handleStatusWordChange: status word for follower of 0x%02x: 0x%04x", id(), statusWord);
#endif

	if (!statusWordOfFollowerChanged)
		m_statusWord = statusWord;

	auto isRelevantStateForFollowerRelationship = [](State s)
	{
		return s == PREPARE_MOVE || s == READY_TO_MOVE || s == MOVING || s == IDLE;
	};

	if (m_followsNodeID == 0)
	{
		// Track the states (except FAULT state) only in the main motor

		if (m_followingNodeID == 0)
		{
			// no follower (fault handling is done in setState)
			setState(determineStateFromStatusWord(m_state, statusWord, id()));
		}
		else
		{
			// we have a following node --> aggregate the state from the main motor and the following motor
			if (!statusWordOfFollowerChanged)
			{
				m_mainNodeState = determineStateFromStatusWord(m_mainNodeState, statusWord, id());
			}
			else
			{
				m_followingNodeState = determineStateFromStatusWord(m_followingNodeState, statusWord, m_followingNodeID);
			}

			diag(DIAG_INFO, 0, "handleStatusWordChange: (aggregate) state for 0x%02x: main: %s, follow: %s, current: %s", id(), stateToString(m_mainNodeState).c_str(), stateToString(m_followingNodeState).c_str(), stateToString(m_state).c_str());
			if ((m_mainNodeState == READY_TO_MOVE && m_followingNodeState == READY_TO_MOVE) && m_state == PREPARE_MOVE)
				setState(READY_TO_MOVE);
			else if ((m_mainNodeState == MOVING || m_followingNodeState == MOVING) && m_state == READY_TO_MOVE)
				setState(MOVING);
			else if ((m_mainNodeState == IDLE && m_followingNodeState == IDLE) && m_state == POWER_ON_DISABLE_OPERATION)
				setState(IDLE);
			else if (!statusWordOfFollowerChanged && !isRelevantStateForFollowerRelationship(m_mainNodeState))
				setState(m_mainNodeState);
		}
	}
	else
	{
		if (!statusWordOfFollowerChanged)
		{
			// Handle the local state machine of a follower:
			auto nextState = determineStateFromStatusWord(m_state, statusWord, id());
			if (!isRelevantStateForFollowerRelationship(nextState) || (m_state == POWER_ON_DISABLE_OPERATION && nextState == IDLE))
			{
				diag(DIAG_INFO, 0, "handleStatusWordChange: local follower handling 0x%02x: 0x%04x %s --> %s",
					 id(), statusWord, stateToString(m_state).c_str(), stateToString(nextState).c_str());

				setState(nextState);
			}
		}
	}
}

MotorDriver::State MotorDriver::determineStateFromStatusWord(MotorDriver::State currentState, uint16_t statusWord, uint8_t nodeID)
{
	if (statusWord & FAULT)
	{
		diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Entering FAULT_STATE, status word: 0x%04x", nodeID, statusWord);
		return FAULT_STATE;
	}
	else if (statusWord & READY_TO_SWITCH_ON &&
		!(statusWord & SWITCHED_ON) &&
		!(statusWord & OPERATION_ENABLED))
	{
		// Drive switched off
		if (currentState == INITIAL_STATE)
		{
			diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching to INITIAL_POWER_OFF, status word: 0x%04x", nodeID, statusWord);
			return INITIAL_POWER_OFF;
		}
		else
		{
			diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching to POWER_ON_DISABLE_OPERATION, status word: 0x%04x", nodeID, statusWord);
			return POWER_ON_DISABLE_OPERATION;
		}
	}
	else
	{
		if (currentState == INITIAL_STATE)
		{
			diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching to INITIAL_POWER_ON, status word: 0x%04x", nodeID, statusWord);
			return INITIAL_POWER_ON;
		}
		else if (statusWord & READY_TO_SWITCH_ON &&
			statusWord & SWITCHED_ON &&
			statusWord & VOLTAGE_ENABLED)
		{
			// Drive switched on
			if (!(statusWord & OPERATION_ENABLED))
			{
				// Operation not enabled
				if (currentState == POWER_ON_DISABLE_OPERATION)
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching POWER_ON_DISABLE_OPERATION --> IDLE, status word: 0x%04x", nodeID, statusWord);
					return IDLE;
				}
				else if (currentState == FAULT_STATE &&
						 !(statusWord & MANUFACTURER_SPECIFIC1))
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching FAULT_STATE --> FAULT_RESET (auto recovery on motor side), status word: 0x%04x", nodeID, statusWord);
					return FAULT_RESET; // In the fault reset state is decided, how to preceed with recovery.
				}
				else if (currentState == FAULT_RESET &&
						 !(statusWord & MANUFACTURER_SPECIFIC1))
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching FAULT_STATE --> CYCLE_POWER_SHUTDOWN, status word: 0x%04x", nodeID, statusWord);
					return CYCLE_POWER_SHUTDOWN;
				}
			}
			else
			{
				// Operation enabled
				if (currentState == PREPARE_HOMING)
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching PREPARE_HOMING--> READY_FOR_HOMING, status word: 0x%04x", nodeID, statusWord);
					return READY_FOR_HOMING;
				}
				else if (currentState == READY_FOR_HOMING &&
						 !(statusWord & TARGET_REACHED) &&
						 !(statusWord & OPERATION_MODE_SPECIFIC1) &&  // Homing Attained
						 !(statusWord & OPERATION_MODE_SPECIFIC2))    // Homing Error
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching READY_FOR_HOMING --> HOMING, status word: 0x%04x", nodeID, statusWord);
					return HOMING;
				}
				else if (currentState == HOMING &&
						 statusWord & TARGET_REACHED)
				{
					if (statusWord & OPERATION_MODE_SPECIFIC1)
					{
							diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching HOMING --> POWER_ON_DISABLE_OPERATION, status word: 0x%04x", nodeID, statusWord);
							return POWER_ON_DISABLE_OPERATION;
					}
					else if (statusWord & OPERATION_MODE_SPECIFIC2)
					{
						diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching HOMING --> FAULT_STATE, status word: 0x%04x", nodeID, statusWord);
						return FAULT_STATE;
					}
				}
				else if (currentState == PREPARE_MOVE &&
					// !(statusWord & TARGET_REACHED) &&
					statusWord & OPERATION_MODE_SPECIFIC1)
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching PREPARE_MOVE --> READY_TO_MOVE, status word: 0x%04x", nodeID, statusWord);
					return READY_TO_MOVE;
				}
				else if (currentState == READY_TO_MOVE &&
					!(statusWord & TARGET_REACHED) &&
					!(statusWord & OPERATION_MODE_SPECIFIC1))
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching READY_TO_MOVE --> MOVING, status word: 0x%04x", nodeID, statusWord);
					return MOVING;
				}
				else if (currentState == MOVING &&
						 statusWord & TARGET_REACHED)
				{
					diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: Switching MOVING --> POWER_ON_DISABLE_OPERATION, status word: 0x%04x", nodeID, statusWord);
					return POWER_ON_DISABLE_OPERATION;
				}
			}
		}
	}
	diag(DIAG_INFO, 0, "determineStateFromStatusWord node 0x%02x: cannot determine state switch, status word: 0x%04x", nodeID, statusWord);
	return currentState;
}

void MotorDriver::setState(MotorDriver::State newState)
{
	if (m_state != newState)
	{
		diag(DIAG_INFO, 0, "setState: Node 0x%02x: Switching %s --> %s", id(), stateToString(m_state).c_str(), stateToString(newState).c_str());
		std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - m_jobStartedAt;

		switch (newState)
		{
		case MotorDriver::INITIAL_STATE:
			break;
		case MotorDriver::INITIAL_POWER_ON:
			m_jobStartedAt = std::chrono::high_resolution_clock::now();
			break;
		case MotorDriver::INITIAL_POWER_OFF:
			m_jobStartedAt = std::chrono::high_resolution_clock::now();
			break;
		case MotorDriver::CYCLE_POWER_SHUTDOWN:
			diag(DIAG_INFO, 0, "Node 0x%02x: Entering CYCLE_POWER_SHUTDOWN after %.6fms", id(), elapsed.count());
			SubmitWrite<int16_t> (0x6040, 0, 0x0006, nullptr);
			break;
		case MotorDriver::POWER_ON_DISABLE_OPERATION:
			// Since this is triggered after every move we want to have faster PDO communication if configured.
			// m_communicationConfig.motorControlWordSetter(0x0007, nullptr);
			diag(DIAG_INFO, 0, "Node 0x%02x: Entering POWER_ON_DISABLE_OPERATION after %.6fms", id(), elapsed.count());
			m_communicationConfig.motorControlWordSetter(0x0007, nullptr);
			break;
		case MotorDriver::PREPARE_MOVE:
			m_jobStartedAt = std::chrono::high_resolution_clock::now();
			prepareMove();
			break;
		case MotorDriver::READY_TO_MOVE:
			diag(DIAG_INFO, 0, "Node 0x%02x: READY_TO_MOVE after %.3fms", id(), elapsed.count());
			executeMove();
			break;
		case MotorDriver::MOVING:
			diag(DIAG_INFO, 0, "Node 0x%02x: Start MOVING after %.3fms", id(), elapsed.count());
			break;
		case MotorDriver::PREPARE_HOMING:
			m_jobStartedAt = std::chrono::high_resolution_clock::now();
			break;
		case MotorDriver::READY_FOR_HOMING:
			// Start Homing
			SubmitWrite<int16_t> (0x6040, 0, 0x001f, [this](uint8_t /*id */, uint16_t /* idx */, uint8_t /* subidx */, ::std::error_code /* ec */)
			{
				// TODO: remove once we can drop support for AuxInd firmwares < 8.47
				setState(MotorDriver::HOMING);  // Work-Around for older firmware versions: switch automatically into the homing mode.
			});
			break;
		case MotorDriver::HOMING:
			diag(DIAG_INFO, 0, "Node 0x%02x: Start HOMING after %.3fms", id(), elapsed.count());
			break;
		case MotorDriver::IDLE:
			diag(DIAG_INFO, 0, "Node 0x%02x: Entering IDLE after %.6fms", id(), elapsed.count());
			processOldestCallbackOnIdle();
			break;
		case MotorDriver::FAULT_STATE:
			m_callbacksOnIdleMutex.lock();
			m_callbackOnIDLE.clear();
			m_callbacksOnIdleMutex.unlock();
			if (m_state != INITIAL_STATE)
				handleFault();
			break;
		case MotorDriver::FAULT_RESET:
			performFaultReset();
			break;
		case MotorDriver::NODE_RESET:
			master.Command(lely::canopen::NmtCommand::RESET_NODE, id());
			break;
		}
		m_state = newState;
	}
	else
	{
		diag(DIAG_INFO, 0, "setState: Node 0x%02x: NOT Switching %s --> %s", id(), stateToString(m_state).c_str(), stateToString(newState).c_str());
	}
}

std::string MotorDriver::stateToString(MotorDriver::State state)
{
	switch (state)
	{
	case MotorDriver::INITIAL_POWER_ON:
		return("INITIAL_POWER_ON");
	case MotorDriver::INITIAL_POWER_OFF:
		return("INITIAL_POWER_OFF");
	case MotorDriver::CYCLE_POWER_SHUTDOWN:
		return("CYCLE_POWER_SHUTDOWN");
	case MotorDriver::POWER_ON_DISABLE_OPERATION:
		return("POWER_ON_DISABLE_OPERATION");
	case MotorDriver::INITIAL_STATE:
		return("INITIAL_STATE");
	case MotorDriver::PREPARE_HOMING:
		return("PREPARE_HOMING");
	case MotorDriver::READY_FOR_HOMING:
		return("READY_FOR_HOMING");
	case MotorDriver::HOMING:
		return("HOMING");
	case MotorDriver::PREPARE_MOVE:
		return ("PREPARE_MOVE");
	case MotorDriver::READY_TO_MOVE:
		return ("READY_TO_MOVE");
	case MotorDriver::MOVING:
		return ("MOVING");
	case MotorDriver::IDLE:
		return ("IDLE");
	case MotorDriver::FAULT_STATE:
		return("FAULT_STATE");
	case MotorDriver::FAULT_RESET:
		return("FAULT_RESET");
	case MotorDriver::NODE_RESET:
		return("NODE_RESET");
	}
}

void MotorDriver::handleFault()
{
	if (!m_emergencyOccured)
	{
		// Handle the fault only in CiA-402 style if it was not detected yet by an emergency.
		// else we get the error twice.
		SubmitRead<uint16_t>(0x603F, 0,
							 [this](uint8_t /*id*/, uint16_t /*idx*/, uint8_t /*subidx*/, ::std::error_code ec, uint16_t value)
		{
			if (!ec)
			{
				if (value != 0)
				{
					std::stringstream message;
					message << boost::format("Motor Fault: code: 0x%04x") % value;
					if (m_errorCallback != nullptr)
						m_errorCallback(value, message.str());
				}
			}
			else
			{
				// TODO: Retry here? Sometimes it is not possible to read the register, e.g. when the state was set shortly before.
				std::stringstream message;
				message << "Error while reading the Fault Register: " << ec << ":" << ec.message();
				if (m_errorCallback != nullptr)
					m_errorCallback(AdditionalErrorCode::READ_ERROR_FAILED, message.str());
			}
		});
	}
}

