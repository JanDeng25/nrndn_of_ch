/*
 * nrUtils.cpp
 *
 *  Created on: Dec 29, 2014
 *      Author: chen
 */

#include "nrUtils.h"
#include "nrProducer.h"

#include "ns3/network-module.h"

namespace ns3
{
namespace ndn
{
namespace nrndn
{

using namespace std;

nrUtils::AppIndexType nrUtils::appIndex;

uint32_t nrUtils::interestedNodeSum = 0;
uint32_t nrUtils::interestedNodeReceivedSum = 0;

uint32_t nrUtils::interestNum = 0;
uint32_t nrUtils::dataNum = 0;
uint32_t nrUtils::interestForwardSum = 0;
uint32_t nrUtils::dataForwardSum = 0;
uint32_t nrUtils::forwardSum = 0;
uint32_t nrUtils::detectTimes = 0;
double nrUtils::delay = 0;

nrUtils::nrUtils()
{
	// TODO Auto-generated constructor stub

}

nrUtils::~nrUtils()
{
	// TODO Auto-generated destructor stub
}

void nrUtils::IncreaseNodeCounter()
{
	interestedNodeSum++;
}

void nrUtils::IncreaseDetectTimes()
{
	detectTimes++;
}

void nrUtils::IncreaseInterestedNodeCounter()
{
	interestedNodeReceivedSum ++;
}

void nrUtils::IncreaseForwardCounter()
{
	forwardSum++;
}

void nrUtils::IncreaseInterestForwardCounter()
{
	interestForwardSum++;
}

void nrUtils::IncreaseDataForwardCounter()
{
	dataForwardSum++;
}

void nrUtils::IncreaseInterestSum()
{
	interestNum++;
}

void nrUtils:: IncreaseDataSum()
{
	dataNum++;
}

uint32_t nrUtils::GetForwardTimes()
{
	return forwardSum;
}

double nrUtils::GetAverageInterestForwardTimes()
{
	return interestForwardSum/interestNum;
}

double nrUtils::GetAverageDataForwardTimes()
{
	return dataForwardSum/dataNum;
}

void nrUtils::updateDelay(double d)
{
	delay += d;
}

double nrUtils::GetAverageDelay()
{
	return delay/interestedNodeReceivedSum;
}

double nrUtils::GetAverageHitRate()
{
	return interestedNodeReceivedSum / interestedNodeSum;
}

uint32_t nrUtils:: GetInterestNum()
{
	return interestNum;
}
uint32_t nrUtils::GetDetectTimes()
{
	return detectTimes;
}

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */
