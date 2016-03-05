/*
 * nrConsumer.h
 *
 *  Created on: Jan 4, 2015
 *      Author: chen
 */

#ifndef NRCONSUMER_H_
#define NRCONSUMER_H_

#include "NodeSensor.h"
#include "ns3/ndnSIM/apps/ndn-consumer-cbr.h"
#include <string>

#include "ndn-nr-pit-impl.h"
#include "ndn-nr-fib-impl.h"
#include "ndn-fib-entry-nrimpl.h"
#include "ndn-packet-type-tag.h"


namespace ns3
{
namespace ndn
{
namespace nrndn
{

class nrConsumer: public ndn::ConsumerCbr
{
public:
	//add by DJ Dec 23,2015
	// If the interest packet is detecting packet, its scope will be set as HELLO_MESSAGE = 3;
	enum
	{
  		RESOURCE_PACKET = 1,        //资源包，data packet
  		DATA_PACKET = 2,                   //数据包，data packet
  		DETECT_PACKET = 3,              //探测包，interest packet
  		INTEREST_PACKET = 4,          //兴趣包，interest packet
  		CONFIRM_PACKET = 5,         //确认包，data packet
  		HELLO_MESSAGE = 6,            //心跳包，interest packet
  		MOVE_TO_NEW_LANE = 7,   //消费者移动到新路段，通知上一跳是否转发数据包，interest packet
  		ASK_FOR_TABLE = 8,               //车辆移动到新路段，向邻居请求表格，interest packet
  		TABLE_PACKET = 9,                  //回复新到的车辆本路段表格，data packet
	};

	static TypeId GetTypeId ();

	nrConsumer();
	virtual ~nrConsumer();

protected:
	  // inherited from App base class. Originally they were private
	  virtual void
	  StartApplication ();    ///< @brief Called at time specified by Start

	  virtual void
	  StopApplication ();     ///< @brief Called at time specified by Stop

	  virtual void
	  OnData (Ptr<const Data> contentObject);

	  /**
	   * \brief Constructs the Interest packet and sends it using a callback to the underlying NDN protocol
	   */
	  virtual void
	  ScheduleNextPacket ();

	  /**
	   * \brief get the current route for the interests
	   */
	  //std::vector<std::string>
	 // GetCurrentInterest();

	  /**
	   * @brief It will do the similar action like ConsumerCbr::ScheduleNextPacket do
	   * 		However, I change some details of it, so use this function to replace it.
	   */
	  //void  doConsumerCbrScheduleNextPacket();

	  /**
	   * @brief Actually send packet, it will take place in Consumer::SendPacket
	   */
	  void SendPacket ();

	  virtual void
	  OnTimeout (uint32_t sequenceNumber);

	  /**
	    * @brief Get Customize data for navigation route heuristic forwarding
	    */
	  //Ptr<Packet> GetNrPayload();

	  // inherited from Object class
	  virtual void NotifyNewAggregate (); ///< @brief Even when object is aggregated to another Object

	  virtual void DoInitialize(void);

	  virtual void
	  OnInterest (Ptr<const Interest> interest);

	  //bool IsInterestData(const Name& name);
private:
	  typedef ConsumerCbr super;
	  Ptr<NodeSensor> m_sensor;
	  UniformVariable m_rand; ///< @brief nonce generator
	  Ptr<fw::nrndn::NavigationRouteHeuristic>		m_forwardingStrategy;

	  uint32_t m_virtualPayloadSize;
	  Name m_prefix;
	  //Ptr<ndn::pit::nrndn::NrPitImpl> m_pit;
	  //Ptr<ndn::fib::nrndn::NrFibImpl> m_fib;
	  //Ptr<ForwardingStrategy>		m_forwardingStrategy;
};

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* NRCONSUMER_H_ */
