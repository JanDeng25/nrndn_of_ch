/*
 * nrConsumer.cc
 *
 *  Created on: Jan 4, 2015
 *      Author: chen
 */

#include "ns3/core-module.h"
#include "ns3/log.h"

#include "ns3/ndn-interest.h"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.h"
#include "ns3/ndn-app.h"
#include "ns3/ndn-face.h"

#include "ndn-navigation-route-heuristic-forwarding.h"
#include "nrConsumer.h"
#include "NodeSensor.h"
#include "nrHeader.h"
#include "nrndn-Header.h"
#include "nrUtils.h"

#include <map>


NS_LOG_COMPONENT_DEFINE ("ndn.nrndn.nrConsumer");

namespace ns3
{
namespace ndn
{
namespace nrndn
{
NS_OBJECT_ENSURE_REGISTERED (nrConsumer);


TypeId nrConsumer::GetTypeId()
{
	static TypeId tid = TypeId ("ns3::ndn::nrndn::nrConsumer")
		    .SetGroupName ("Nrndn")
		    .SetParent<ConsumerCbr> ()
		    .AddConstructor<nrConsumer> ()
		    .AddAttribute ("iPrefix","Prefix, for which consumer has the data",
   			                           StringValue ("/"),
	    			                    MakeNameAccessor (&nrConsumer::m_prefix),
	    			                    MakeNameChecker ())
		    .AddAttribute("sensor", "The vehicle sensor used by the nrConsumer.",
	    	   	    		PointerValue (),
		    	   	    	MakePointerAccessor (&nrConsumer::m_sensor),
	    	   	    		MakePointerChecker<ns3::ndn::nrndn::NodeSensor> ())
		    .AddAttribute ("PayloadSize", "Virtual payload size for traffic Content packets",
		    		            UintegerValue (1024),
		    	                MakeUintegerAccessor (&nrConsumer::m_virtualPayloadSize),
		    		            MakeUintegerChecker<uint32_t> ())

//		    .AddAttribute ("Pit","pit of consumer",
	//	    		             PointerValue (),
//		    		             MakePointerAccessor (&nrConsumer::m_pit),
//		    		             MakePointerChecker<ns3::ndn::pit::nrndn::NrPitImpl> ())
//		    .AddAttribute ("Fib","fib of consumer",
//		    		 		     PointerValue (),
//		    		 		     MakePointerAccessor (&nrConsumer::m_fib),
//		    		 		     MakePointerChecker<ns3::ndn::fib::nrndn::NrFibImpl> ())
		    ;
		  return tid;
}

nrConsumer::nrConsumer():
		m_rand (0, std::numeric_limits<uint32_t>::max ()),
		m_virtualPayloadSize(1024)
{
	// TODO Auto-generated constructor stub
}

nrConsumer::~nrConsumer()
{
	// TODO Auto-generated destructor stub
}

void nrConsumer::StartApplication()
{
	NS_LOG_FUNCTION_NOARGS ();
	m_forwardingStrategy->Start();
	// retransmission timer expiration is not necessary here, just cancel the event
	//m_retxEvent.Cancel ();
	super::StartApplication();
}

void nrConsumer::StopApplication()
{
	NS_LOG_FUNCTION_NOARGS ();
	m_forwardingStrategy->Stop();
	super::StopApplication();
}

void nrConsumer::ScheduleNextPacket()
{
	if(GetNode()->GetId() > 35)
		return;

	double delay =GetNode()->GetId()  - 14  + 65;

	Simulator::Schedule (Seconds (delay), & nrConsumer::SendPacket, this);
}

void nrConsumer::SendPacket()
{

	  if (!m_active) return;
	 // cout<<"consumer send packet"<<endl;
	  if(isJuction(m_sensor->getLane()))
	  {
		  Simulator::Schedule (Seconds (5.0), & nrConsumer::SendPacket, this);
		  return;
	  }

	  uint32_t num = GetNode()->GetId() % 3 + 1;
	  Name prefix("/");
	  prefix.appendNumber(num);

	  Ptr<Interest> interest = Create<Interest> (Create<Packet>(m_virtualPayloadSize));
	  Ptr<Name> interestName = Create<Name> (prefix);
	  interest->SetName(interestName);
	  interest->SetNonce(m_rand.GetValue());//just generate a random number
	  //interest->SetInterestLifetime    (m_interestLifeTime);

	   //add header;
	  ndn::nrndn::nrndnHeader nrheader;
	  nrheader.setSourceId(GetNode()->GetId());
	  nrheader.setX(m_sensor->getX());
	  nrheader.setY(m_sensor->getY());
	  std::string lane = m_sensor->getLane();
	  nrheader.setPreLane(lane);
	  nrheader.setCurrentLane(lane);

	  Ptr<Packet> newPayload = Create<Packet> (m_virtualPayloadSize);
	  newPayload->AddHeader(nrheader);
	  interest->SetPayload(newPayload);

	  cout<<"node: "<<GetNode()->GetId()<<"  send interest packet,name: "<<prefix.toUri()<<" in consumer"<<endl;

	  m_transmittedInterests (interest, this, m_face);
	  m_face->ReceiveInterest (interest);

	  interestSent[interest->GetNonce()] = prefix.toUri() ;
	  msgTime[interest->GetNonce()] = Simulator::Now().GetSeconds();

	  nrUtils:: IncreaseInterestedNodeSum();

}

void nrConsumer::OnData(Ptr<const Data> data)
{
	 if (!m_active) return;
	NS_LOG_FUNCTION (this);
	 //cout<<"consumer on data"<<endl;
	Ptr<Packet> nrPayload	= data->GetPayload()->Copy();
	nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	uint32_t packetPayloadSize = nrPayload->GetSize();
	NS_ASSERT_MSG(packetPayloadSize == m_virtualPayloadSize,"packetPayloadSize is not equal to "<<m_virtualPayloadSize);
	const Name& name = data->GetName();

	uint32_t nodeId=nrheader.getSourceId();
	uint32_t signature=data->GetSignature();

	map<uint32_t, string>::iterator it = interestSent.find(signature);
	if(it != interestSent.end())
	{
		nrUtils::IncreaseInterestedNodeReceivedSum();
		double delay = Simulator::Now().GetSeconds() - msgTime[signature];
		nrUtils::GetDelaySum(delay);
		std::cout<<m_node->GetId()<<"\treceived data "<<name.toUri()<<" from "<<nodeId<<"\tSignature "<<signature<<endl;
		interestSent.erase(it);
	}

}

void nrConsumer::laneChange(std::string oldLane, std::string newLane)
{
	 if (!m_active) return;
	 //cout<<"consumer lane change"<<endl;
	if(interestSent.empty()) return;
	if(isJuction(newLane) ) return;
	if(oldLane == m_oldLane) return;

	m_oldLane = oldLane;

	  cout<<m_node->GetId()<<" lane changed from "<<oldLane<<" to "<<newLane<<endl;

	  uint32_t num = GetNode()->GetId() % 3 + 1;
	  Name prefix("/");
	  prefix.appendNumber(num);

	  Ptr<Interest> interest = Create<Interest> (Create<Packet>(m_virtualPayloadSize));
	  Ptr<Name> interestName = Create<Name> (prefix);
	  interest->SetName(interestName);
	  interest->SetNonce(m_rand.GetValue());//just generate a random number
	  interest->SetInterestLifetime    (m_interestLifeTime);
	  interest->SetScope(MOVE_TO_NEW_LANE);

	   //add header;
	  ndn::nrndn::nrndnHeader nrheader;
	  nrheader.setSourceId(GetNode()->GetId());
	  nrheader.setX(m_sensor->getX());
	  nrheader.setY(m_sensor->getY());
	  std::string lane = m_sensor->getLane();
	  nrheader.setPreLane(oldLane);
	  nrheader.setCurrentLane(lane);

	  Ptr<Packet> newPayload = Create<Packet> (m_virtualPayloadSize);
	  newPayload->AddHeader(nrheader);
	  interest->SetPayload(newPayload);

	  cout<<"node: "<<GetNode()->GetId()<<"  send MOVE_TO_NEW_LANE packet,name: "<<prefix.toUri()<<" in consumer"<<endl;

	  m_transmittedInterests (interest, this, m_face);
	  m_face->ReceiveInterest (interest);
}

bool nrConsumer::isJuction(std::string lane)
{
	for(uint32_t i = 0; i<lane.length(); ++i)
		if(lane[i] == 't')
			return false;
	return true;
}

void nrConsumer::NotifyNewAggregate()
{
  super::NotifyNewAggregate ();
}

void nrConsumer::DoInitialize(void)
{
	 //cout<<"consumer send packet"<<endl;
	if (m_forwardingStrategy == 0)
	{
		//m_forwardingStrategy = GetObject<fw::nrndn::NavigationRouteHeuristic>();
		Ptr<ForwardingStrategy> forwardingStrategy=m_node->GetObject<ForwardingStrategy>();
		NS_ASSERT_MSG(forwardingStrategy,"nrConsumer::DoInitialize cannot find ns3::ndn::fw::ForwardingStrategy");
		m_forwardingStrategy = DynamicCast<fw::nrndn::NavigationRouteHeuristic>(forwardingStrategy);
	}
	if (m_sensor == 0)
	{
		m_sensor =  m_node->GetObject<ndn::nrndn::NodeSensor>();
		NS_ASSERT_MSG(m_sensor,"nrConsumer::DoInitialize cannot find ns3::ndn::nrndn::NodeSensor");
		if(m_sensor != NULL)
				m_sensor->TraceConnectWithoutContext ("LaneChange", MakeCallback (&nrConsumer::laneChange,this));
	}
	super::DoInitialize();
}

void nrConsumer::OnTimeout(uint32_t sequenceNumber)
{
	return;
}

void nrConsumer::OnInterest(Ptr<const Interest> interest)
{
	NS_ASSERT_MSG(false,"nrConsumer should not be supposed to "
			"receive Interest Packet!!");
}
/*
//modify by DJ on Jan 8,2016
bool nrConsumer::IsInterestData(const Name& name)
{
	return (m_pit->Find(name)!=0);
}
*/

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */
