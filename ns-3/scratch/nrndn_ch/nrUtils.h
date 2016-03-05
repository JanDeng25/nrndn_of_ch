/*
 * nrUtils.h
 *
 *  Created on: Dec 29, 2014
 *      Author: chen
 */

#ifndef NRUTILS_H_
#define NRUTILS_H_

#include "ns3/core-module.h"
#include "ns3/ndn-wire.h"
#include <unordered_map>
#include <vector>
#include <string>
#include <utility>

namespace ns3
{
namespace ndn
{
namespace nrndn
{

/**
 *
 * The class implements the universal Method for nodes to use the navigation route
 * And some global variables
 *
 */

class nrUtils
{
public:
	nrUtils();
	~nrUtils();

	typedef std::unordered_map<std::string, uint32_t> AppIndexType;
	static void IncreaseNodeCounter();
	static void IncreaseInterestedNodeCounter();
	static void IncreaseInterestSum();
	static void IncreaseDataSum();
	static void IncreaseDetectTimes();

	static void IncreaseForwardCounter();
	static void IncreaseInterestForwardCounter();
	static void IncreaseDataForwardCounter();

	static double GetAverageHitRate();
	static uint32_t GetForwardTimes();
	static double GetAverageInterestForwardTimes();
	static double GetAverageDataForwardTimes();
	static double GetAverageDelay();
	static void updateDelay(double d);
	static uint32_t GetInterestNum();
	static uint32_t GetDetectTimes();

	//4 . appIndex
	static AppIndexType appIndex;

	static uint32_t interestedNodeSum;
	static uint32_t interestedNodeReceivedSum;

	static uint32_t interestNum;
	static uint32_t dataNum;
	static uint32_t interestForwardSum;
	static uint32_t dataForwardSum;
	static uint32_t forwardSum;
	static uint32_t detectTimes;
	static double delay;
};

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* NRUTILS_H_ */
