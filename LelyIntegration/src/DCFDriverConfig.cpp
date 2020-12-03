/**@file
 * This header file is part of the LelyIntegration library;
 * it contains the implementation of the configuration class for a DCFDriver.
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
#include <lely/co/obj.h>
#include <lely/util/diag.h>

#include "DCFDriverConfig.h"

DCFDriverConfig::DCFDriverConfig(const std::string &textualDcfFileName, const std::string &binaryDcfFileName, uint8_t defaultNodeID) :
	lely::canopen::Device(textualDcfFileName, binaryDcfFileName, defaultNodeID),
	m_defaultNodeID(defaultNodeID),
	m_binaryDcfFile(binaryDcfFileName)
{

}

DCFDriverConfig::ObjectsList DCFDriverConfig::getSDOIndicesForDriverConfiguration() const
{
	ObjectsList result;
	std::vector<uint16_t> sdoIndices = reinterpret_cast<lely::CODev*>(dev())->getIdx();
	for (auto sdoIndex : sdoIndices)
	{
		lely::COObj* sdoObject = reinterpret_cast<lely::CODev*>(dev())->find(sdoIndex);
		std::vector<uint8_t> sdoSubIndices = sdoObject->getSubidx();
		result.push_back({sdoIndex, {}});
		auto& subIndexResult = std::get<1>(*result.rbegin());
		for (auto sdoSubIndex : sdoSubIndices)
		{
			lely::COSub* sdoSubObject = sdoObject->find(sdoSubIndex);
			if ((sdoSubObject->getAccess() & CO_ACCESS_READ) && (sdoSubObject->getAccess() & CO_ACCESS_WRITE) && (sdoSubObject->getFlags() & CO_OBJ_FLAGS_VAL_SET_EXPLICITLY))
			{
				diag(DIAG_INFO, 0, "    Adding SDO 0x%04x/0x%02x (%s / %s) ...", sdoObject->getIdx(), sdoSubObject->getSubidx(), sdoObject->getName(), sdoSubObject->getName());
				subIndexResult.push_back(sdoSubIndex);
			}
		}
		if (subIndexResult.empty())
			result.pop_back();
	}
	return result;
}

uint16_t DCFDriverConfig::getTypeOfObject(uint16_t sdoIndex, uint8_t sdoSubIndex) const
{
	lely::COObj* sdoObject = reinterpret_cast<lely::CODev*>(dev())->find(sdoIndex);
	if (sdoObject == nullptr)
		return 0;
	lely::COSub* sdoSubObject = sdoObject->find(sdoSubIndex);
	if (sdoSubObject == nullptr)
		return 0;
	return sdoSubObject->getType();
}





