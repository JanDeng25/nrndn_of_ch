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
	static void IncreaseInterestedNodeSum();
	static void IncreaseInterestedNodeReceivedSum();
	static void IncreaseInterestNum();
	static void IncreaseDataNum();
	static void IncreaseDetectNum();
	static void IncreaseConfirmNum();
	static void IncreaseTableNum();

	static void IncreaseInterestForwardCounter();
	static void IncreaseDataForwardCounter();
	static void IncreaseDetectForwardCounter();
	static void IncreaseConfirmForwardCounter();
	static void IncreaseResourceForwardCounter();

	static uint32_t GetResourceForwardSum();
	static uint32_t GetTableSum();//表格维护的总开销：asktableNum + tableNum
	static double GetAverageHitRate();
	static double GetAverageDetectRate();//平均探测率：detectNum/interestedNodeSum
	static double GetAverageConfirmRate();//平均确认率：confirmNum/detectNum
	static double GetAverageInterestForwardTimes();
	static double GetAverageDataForwardTimes();
	static double GetAverageForwardSum();//平均请求数据包的开销：（interestForwardSum+dataForwardSum+detectForwardSum+confirmForwardSum）/interestedNodeSum
	static void GetDelaySum(double d);//发送interest后收到data的总延时（有的收不到data）
	static double GetAverageDelay();//平均延时：delaySum / interestedNodeSum
	static uint32_t GetInterestedNodeReceivedSum();

	static void SetFullFibNumZero();
	static void SetFullFibNum(uint32_t index, uint32_t num);
	static void CoutFullFibNum();

	//4 . appIndex
	static AppIndexType appIndex;

	static uint32_t InterestedNodeSum;//发送interest的节点数量
	static uint32_t InterestedNodeReceivedSum;//发送interest的节点中收到data的节点数量

	static uint32_t InterestNum;//发送interest包的数量
	static uint32_t DataNum;//发送data包的数量
	static uint32_t DetectNum;//发送detect的数量
	static uint32_t ConfirmNum;//发送confirm包的数量

	static uint32_t InterestForwardSum;//interest总的转发次数
	static uint32_t DataForwardSum;//data总的转发次数
	static uint32_t DetectForwardSum;//detect总的转发次数
	static uint32_t ConfirmForwardSum;//confirm总的转发次数
	static uint32_t ResourceForwardSum;//resource总的转发次数  ！！！

	static double DelaySum;//发送interest后收到data的总延时（有的收不到data）

	static uint32_t ForwardSum;//所有包的发送及转发次数
	static uint32_t TableSum;//表格维护的总开销：asktableNum + tableNum
	static double AverageHitRate;//平均命中率：interestedNodeReceivedSum / interestedNodeSum
	static double AverageDetectRate;//平均探测率：detectNum/interestedNodeSum
	static double AverageConfirmRate;//平均确认率：confirmNum/detectNum
	static double AverageForwardSum;//平均请求数据包的开销：（interestForwardSum+dataForwardSum+detectForwardSum+confirmForwardSum）/interestedNodeSum
	static double AverageDelay;//平均延时：delaySum / interestedNodeSum

	static uint32_t FullFibNum[801];
};

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* NRUTILS_H_ */
