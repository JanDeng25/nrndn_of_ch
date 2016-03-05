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
//		    .AddAttribute("sensor", "The vehicle sensor used by the nrConsumer.",
//		    	   	    		PointerValue (),
//		    	   	    		MakePointerAccessor (&nrConsumer::m_sensor),
//		    	   	    		MakePointerChecker<ns3::ndn::nrndn::NodeSensor> ())
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
	if(GetNode()->GetId() > 30)
		return;

	double delay =( GetNode()->GetId() - 9 ) * 5 + 20;

	Simulator::Schedule (Seconds (delay), & nrConsumer::SendPacket, this);
}

void nrConsumer::SendPacket()
{
	  if (!m_active) return;

	  uint32_t num = GetNode()->GetId() % 3 + 1;
	  m_prefix.appendNumber(num);

	  Ptr<Interest> interest = Create<Interest> (Create<Packet>(m_virtualPayloadSize));
	  Ptr<Name> interestName = Create<Name> (m_prefix);
	  interest->SetName(interestName);
	  interest->SetNonce(m_rand.GetValue());//just generate a random number
	  interest->SetInterestLifetime    (m_interestLifeTime);

	  	 //add header;
	  	 ndn::nrndn::nrndnHeader nrheader;
	  	 nrheader.setSourceId(GetNode()->GetId());
	  	 nrheader.setX(m_sensor->getX());
	  	 nrheader.setY(m_sensor->getY());
	  	 std::string lane = m_sensor->getLane();
	  	 nrheader.setPreLane(lane);

	  	 Ptr<Packet> newPayload = Create<Packet> ();
	  	 newPayload->AddHeader(nrheader);
	  	 interest->SetPayload(newPayload);

	    m_transmittedInterests (interest, this, m_face);
	    m_face->ReceiveInterest (interest);
}

void nrConsumer::OnData(Ptr<const Data> data)
{
	/*
	NS_LOG_FUNCTION (this);
	Ptr<Packet> nrPayload	= data->GetPayload()->Copy();
	const Name& name = data->GetName();
	nrHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	uint32_t nodeId=nrheader.getSourceId();
	uint32_t signature=data->GetSignature();
	uint32_t packetPayloadSize = nrPayload->GetSize();

	NS_LOG_DEBUG("At time "<<Simulator::Now().GetSeconds()<<":"<<m_node->GetId()<<"\treceived data "<<name.toUri()<<" from "<<nodeId<<"\tSignature "<<signature<<"\t forwarded by("<<nrheader.getX()<<","<<nrheader.getY()<<")");
	NS_LOG_DEBUG("payload Size:"<<packetPayloadSize);
	std::cout<<"At time "<<Simulator::Now().GetSeconds()<<":"<<m_node->GetId()<<"\treceived data "<<name.toUri()<<" from "<<nodeId<<"\tSignature "<<signature<<"\t forwarded by("<<nrheader.getX()<<","<<nrheader.getY()<<")\n";

	NS_ASSERT_MSG(packetPayloadSize == m_virtualPayloadSize,"packetPayloadSize is not equal to "<<m_virtualPayloadSize);

	double delay = Simulator::Now().GetSeconds() - data->GetTimestamp().GetSeconds();
	nrUtils::InsertTransmissionDelayItem(nodeId,signature,delay);
	if(IsInterestData(data->GetName()))
		nrUtils::IncreaseInterestedNodeCounter(nodeId,signature);
	else
		nrUtils::IncreaseDisinterestedNodeCounter(nodeId,signature);
	//NS_LOG_UNCOND("At time "<<Simulator::Now().GetSeconds()<<":"<<m_node->GetId()<<"\treceived data "<<name.toUri()<<" from "<<nodeId<<"\tSignature "<<signature);
	 **/

}

void nrConsumer::NotifyNewAggregate()
{
  super::NotifyNewAggregate ();
}

void nrConsumer::DoInitialize(void)
{
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
