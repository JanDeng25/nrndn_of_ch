/*
 * ndn-navigation-route-heuristic-forwarding.h
 *
 *  Created on: Jan 14, 2015
 *      Author: chen
 */

#ifndef NDN_NAVIGATION_ROUTE_HEURISTIC_FORWARDING_H_
#define NDN_NAVIGATION_ROUTE_HEURISTIC_FORWARDING_H_

#include "ns3/ndnSIM/model/fw/green-yellow-red.h"
#include "ns3/timer.h"
#include "ns3/simulator.h"
#include "ns3/random-variable.h"
#include "ns3/string.h"
#include "ns3/random-variable-stream.h"
#include "ns3/ndn-header-helper.h"

#include "LRUCache.h"
#include "NodeSensor.h"
#include "ndn-nr-pit-impl.h"
#include "ndn-nr-fib-impl.h"
#include "ndn-nr-cs-impl.h"
#include "nrndn-Neighbors.h"

#include <vector>
#include <map>
#include <unordered_set>

namespace ns3
{
namespace ndn
{
namespace fw
{
namespace nrndn
{

/**
 * @brief This forward strategy is Navigation Route Heuristic forwarding strategy
 *        Different with traditional forwarding strategy, You should execute
 *        Start() to make the forwarding strategy start to work, and stop it using Stop().
 *        It will NOT start/stop automatically!
 */
class NavigationRouteHeuristic: public GreenYellowRed
{
public:
	static TypeId
	GetTypeId(void);

	void Start ();

	void Stop ();

	virtual void OnInterest(Ptr<Face> face, Ptr<Interest> interest);

	virtual void OnData(Ptr<Face> face, Ptr<Data> data);

	NavigationRouteHeuristic ();

	virtual ~NavigationRouteHeuristic ();

	const Ptr<ndn::nrndn::NodeSensor>& getSensor() const
	{
		return m_sensor;
	}

	virtual void AddFace(Ptr<Face> face);

	virtual void RemoveFace(Ptr<Face> face);

	//void laneChange(std::string, std::string);

	enum
	{
  		RESOURCE_PACKET = 1,        //资源包，data packet
  		DATA_PACKET = 2,                   //数据包，data packet
  		DETECT_PACKET = 3,              //探测包，interest packet
  		INTEREST_PACKET = 4,          //兴趣包，interest packet
  		CONFIRM_PACKET = 5,         //确认包，data packet
  		HELLO_PACKET = 6,            //心跳包，interest packet
  		MOVE_TO_NEW_LANE = 7,   //消费者移动到新路段，通知上一跳是否转发数据包，interest packet
  		ASK_FOR_TABLE = 8,               //车辆移动到新路段，向邻居请求表格，interest packet
  		TABLE_PACKET = 9,                  //回复新到的车辆本路段表格，data packet
	};

protected:
	virtual void
	WillSatisfyPendingInterest(Ptr<Face> inFace, Ptr<pit::Entry> pitEntry);

	virtual bool
	DoPropagateInterest(Ptr<Face> inFace, Ptr<const Interest> interest,Ptr<pit::Entry> pitEntry);

	virtual void
	WillEraseTimedOutPendingInterest(Ptr<pit::Entry> pitEntry);

	virtual void
	DidReceiveValidNack(Ptr<Face> incomingFace, uint32_t nackCode,Ptr<const Interest> nack, Ptr<pit::Entry> pitEntry);

	virtual void NotifyNewAggregate (); ///< @brief Even when object is aggregated to another Object

	virtual void DoInitialize(void);

private:

	void ProcessHello (Ptr<Interest> interest);

	void HelloTimerExpire ();

	void FindBreaksLinkToNextHop(uint32_t BreakLinkNodeId);

	bool OnTheWay(std::vector<std::string> laneList);

	void ToContentStore(Ptr<Data> data);

	void NotifyUpperLayer(Ptr<Data> data);


	void ExpireInterestPacketTimer(uint32_t nodeId,uint32_t seq);

	void ExpireDataPacketTimer(uint32_t nodeId,uint32_t signature);

	bool isDuplicatedInterest(uint32_t id, uint32_t nonce);

	bool isDuplicatedData(uint32_t id, uint32_t signature);


	void DropPacket();

	void DropDataPacket(Ptr<Data> data);

	void DropInterestePacket(Ptr<Interest> interest);


	void ForwardInterestPacket(Ptr<Interest>);

	void ForwardDetectPacket(Ptr<Interest>);

	void ForwardDataPacket(Ptr<Data> src);

	void ForwardConfirmPacket(Ptr<Data> src);

	void ForwardResourcePacket(Ptr<Data> src);


	void ReplyConfirmPacket(Ptr<Interest> interest);

	void ReplyTablePacket(Ptr<Interest> interest);

	void ReplyDataPacket(Ptr<Interest> interest);


	void PrepareInterestPacket(Ptr<Interest> interest);

	void PrepareDetectPacket(Ptr<Interest> interest);


	void SendInterestPacket(Ptr<Interest> interest);

	void SendDataPacket(Ptr<Data> data);

	void SendHello ();


private:
	typedef GreenYellowRed super;
	/**
	  * Every HelloInterval the node broadcast a  Hello message
	  */
	Time HelloInterval;
	uint32_t AllowedHelloLoss;  ///< \brief Number of hello messages which may be loss for valid link

	Timer m_htimer;			///< \brief Hello timer

	//It is use for finding a scheduled sending event for a paticular interest packet, nodeid+seq can locate a sending event
	std::map< uint32_t, std::map<uint32_t, EventId> > m_sendingInterestEvent;

	//It is use for finding a scheduled sending event for a paticular data packet, nodeid+seq can locate a sending event
	std::map< uint32_t, std::map<uint32_t, EventId> > m_sendingDataEvent;

	//when a packet is about to send, it should wait several time slot before it is actuall send
	//Default is 0.05, so the sending frequency is 20Hz
	Time m_timeSlot;

	//Provides uniform random variables.
	Ptr<UniformRandomVariable> m_uniformRandomVariable;

	Ptr<ndn::nrndn::NodeSensor> m_sensor;

	Ptr<ndn::pit::nrndn::NrPitImpl> m_pit;
	Ptr<ndn::fib::nrndn::NrFibImpl> m_fib;
	Ptr<ndn::cs::nrndn::NrCsImpl> m_cs;

	uint32_t				m_CacheSize;

	ndn::nrndn::cache::LRUCache<uint32_t,bool>
							m_interestNonceSeen;///< \brief record the randomly-genenerated byte string that is used to detect and discard duplicate Interest messages
	ndn::nrndn::cache::LRUCache<uint32_t,bool>
							m_dataSignatureSeen;///< \brief record the signature that is used to detect and discard duplicate data messages

	vector<Ptr<Face> >      m_outFaceList;///< \brief face list(NetDeviceFace) that interest/data packet will be sent down to
	vector<Ptr<Face> >      m_inFaceList;///< \brief face list(AppFace) that interest/data packet will be sent up to

	Ptr<Node>               m_node;///< \brief the node which forwarding strategy is installed

	Time	m_offset;  	///< \brief record the last random value in scheduling the HELLO timer

	Neighbors m_nb;  ///< \brief Handle neighbors

	bool m_running;// \brief Tell if it should be running, initial is false;

	uint32_t m_runningCounter;// \brief Count how many apps are using this fw strategy. running will not stop until the counter is ZERO

	bool m_HelloLogEnable;// \brief the switch which can turn on the log on Functions about hello messages

	uint32_t m_gap;// \brief the time gap between interested nodes and disinterested nodes for sending a data packet;

	uint32_t m_TTLMax;// \brief This value indicate that when a data is received by disinterested node, the max hop count it should be forwarded

	bool NoFwStop;// \brief When the PIT covers the nodes behind, no broadcast stop message

	uint32_t m_virtualPayloadSize;

	Time m_freshness;
};
} /* namespace nrndn */
} /* namespace fw */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* NDN_NAVIGATION_ROUTE_HEURISTIC_FORWARDING_H_ */
