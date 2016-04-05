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

uint32_t nrUtils::InterestedNodeSum = 0;
uint32_t nrUtils::InterestedNodeReceivedSum = 0;

uint32_t nrUtils::InterestNum = 0;
uint32_t nrUtils::DataNum = 0;
uint32_t nrUtils::DetectNum = 0;
uint32_t nrUtils::ConfirmNum = 0;//发送confirm包的数量

uint32_t nrUtils::InterestForwardSum = 0;
uint32_t nrUtils::DataForwardSum = 0;
uint32_t nrUtils::DetectForwardSum= 0;//detect总的转发次数
uint32_t nrUtils::ConfirmForwardSum= 0;//confirm总的转发次数
uint32_t nrUtils::ResourceForwardSum= 0;//resource总的转发次数  ！！！
double nrUtils::DelaySum = 0;
uint32_t nrUtils::ForwardSum = 0;
uint32_t nrUtils::TableSum= 0;//表格维护的总开销：asktableNum + tableNum
double nrUtils::AverageHitRate= 0;//平均命中率：interestedNodeReceivedSum / interestedNodeSum
double nrUtils::AverageDetectRate= 0;//平均探测率：detectNum/interestedNodeSum
double nrUtils::AverageConfirmRate = 0;
double nrUtils::AverageForwardSum= 0;//平均请求数据包的开销：（interestForwardSum+dataForwardSum+detectForwardSum+confirmForwardSum）/interestedNodeSum
double nrUtils::AverageDelay= 0;//平均延时：delaySum / interestedNodeSum
uint32_t nrUtils::FullFibNum[801];
nrUtils::nrUtils()
{
	// TODO Auto-generated constructor stub

}

nrUtils::~nrUtils()
{
	// TODO Auto-generated destructor stub
}

void nrUtils::SetFullFibNumZero()
{
	for(uint32_t i = 0; i<801; ++i)
		FullFibNum[i] = 0;
}

void nrUtils::SetFullFibNum(uint32_t index, uint32_t num)
{
	FullFibNum[index] = num;
}

void nrUtils::CoutFullFibNum()
{
	uint32_t one = 0, two = 0, three = 0, zero = 0;
	for(uint32_t i = 0; i<801; ++i)
	{
		if(FullFibNum[i] == 0)
			zero++;
		else if(FullFibNum[i] == 1)
			one++;
		else if(FullFibNum[i] == 2)
			two++;
		else if(FullFibNum[i] == 3)
			three++;
	}
	cout<<"one fib:"<<one<<" two fib:"<<two<<" three fib:"<<three<<" zero fib:"<<zero<<endl;
	cout<<"resource forward times:"<<ResourceForwardSum<<endl;
	cout<<"hitrate:"<<AverageHitRate<<endl;
}

void nrUtils::IncreaseInterestedNodeSum()
{
	InterestedNodeSum++;
}
void nrUtils::IncreaseInterestedNodeReceivedSum()
{
	InterestedNodeReceivedSum++;
}
void nrUtils::IncreaseInterestNum()
{
	InterestNum++;
}
void nrUtils::IncreaseDataNum()
{
	DataNum++;
}
void nrUtils::IncreaseDetectNum()
{
	DetectNum++;
}
void nrUtils::IncreaseConfirmNum()
{
	ConfirmNum++;
}
void nrUtils::IncreaseTableNum()
{
	TableSum++;
}

void nrUtils::IncreaseInterestForwardCounter()
{
	InterestForwardSum++;
}
void nrUtils::IncreaseDataForwardCounter()
{
	DataForwardSum++;
}
void nrUtils::IncreaseDetectForwardCounter()
{
	DetectForwardSum++;
}
void nrUtils::IncreaseConfirmForwardCounter()
{
	ConfirmForwardSum++;
}
void nrUtils::IncreaseResourceForwardCounter()
{
	ResourceForwardSum++;
}
uint32_t nrUtils::GetResourceForwardSum()
{
	return ResourceForwardSum;
}
uint32_t nrUtils::GetTableSum()//表格维护的总开销：asktableNum + tableNum
{
	return TableSum;
}
double nrUtils::GetAverageHitRate()
{
	double AverageHitRate = InterestedNodeReceivedSum / InterestedNodeSum;
	std::cout<<"InterestedNodeReceivedSum:"<<InterestedNodeReceivedSum<<" InterestedNodeSum:"<<InterestedNodeSum<<" nrutils::hitrate: "<<AverageHitRate<<" 15/21:"<<15.0/21<<std::endl;
	return  1.0*InterestedNodeReceivedSum /InterestedNodeSum;
}
double nrUtils::GetAverageDetectRate()//平均探测率：detectNum/interestedNodeSum
{
	double DetectRate = 1.0*DetectNum/InterestedNodeSum;
	return DetectRate;
}
double nrUtils::GetAverageConfirmRate()
{
	if(DetectNum == 0)
		return 0;
	double ConfirmRate =1.0* ConfirmNum/DetectNum;
	return ConfirmRate;
}
double nrUtils::GetAverageInterestForwardTimes()
{
	return 1.0*InterestForwardSum/InterestedNodeReceivedSum;
}
double nrUtils::GetAverageDataForwardTimes()
{
	return 1.0*DataForwardSum/InterestedNodeReceivedSum;
}
double nrUtils::GetAverageForwardSum()//平均请求数据包的开销：（interestForwardSum+dataForwardSum+detectForwardSum+confirmForwardSum）/interestedNodeSum
{
	double AverageForwardSum=1.0* (InterestForwardSum + DataForwardSum + DetectForwardSum + ConfirmForwardSum) /InterestedNodeReceivedSum;
	return AverageForwardSum;
}
void nrUtils::GetDelaySum(double d)//发送interest后收到data的总延时（有的收不到data）
{
	DelaySum +=d;
}
double nrUtils::GetAverageDelay()//平均延时：delaySum / interestedNodeSum
{
	if(InterestedNodeReceivedSum == 0)
	{
		std::cout<<"InterestedNodeReceivedSum = 0"<<std::endl;
		return 0;
	}
	double AverageDelay = DelaySum / InterestedNodeReceivedSum;
	return AverageDelay;
}

uint32_t  nrUtils::GetInterestedNodeReceivedSum()
{
	return InterestedNodeReceivedSum;
}

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */
