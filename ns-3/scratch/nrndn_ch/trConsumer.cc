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

#include "trndn-navigation-route-heuristic-forwarding.h"
#include "trConsumer.h"
#include "NodeSensor.h"
#include "nrHeader.h"
#include "trndn-Header.h"
#include "nrUtils.h"

#include <map>


NS_LOG_COMPONENT_DEFINE ("ndn.nrndn.trConsumer");

namespace ns3
{
namespace ndn
{
namespace nrndn
{
NS_OBJECT_ENSURE_REGISTERED (trConsumer);


TypeId trConsumer::GetTypeId()
{
	static TypeId tid = TypeId ("ns3::ndn::nrndn::trConsumer")
		    .SetGroupName ("Nrndn")
		    .SetParent<ConsumerCbr> ()
		    .AddConstructor<trConsumer> ()
		    .AddAttribute ("iPrefix","Prefix, for which consumer has the data",
   			                           StringValue ("/"),
	    			                    MakeNameAccessor (&trConsumer::m_prefix),
	    			                    MakeNameChecker ())
		    .AddAttribute("sensor", "The vehicle sensor used by the nrConsumer.",
	    	   	    		PointerValue (),
		    	   	    	MakePointerAccessor (&trConsumer::m_sensor),
	    	   	    		MakePointerChecker<ns3::ndn::nrndn::NodeSensor> ())
		    .AddAttribute ("PayloadSize", "Virtual payload size for traffic Content packets",
		    		            UintegerValue (1024),
		    	                MakeUintegerAccessor (&trConsumer::m_virtualPayloadSize),
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

trConsumer::trConsumer():
		m_rand (0, std::numeric_limits<uint32_t>::max ()),
		m_virtualPayloadSize(1024)
{
	// TODO Auto-generated constructor stub
}

trConsumer::~trConsumer()
{
	// TODO Auto-generated destructor stub
}

void trConsumer::StartApplication()
{
	NS_LOG_FUNCTION_NOARGS ();
	m_forwardingStrategy->Start();
	// retransmission timer expiration is not necessary here, just cancel the event
	//m_retxEvent.Cancel ();
	super::StartApplication();
}

void trConsumer::StopApplication()
{
	NS_LOG_FUNCTION_NOARGS ();
	m_forwardingStrategy->Stop();
	super::StopApplication();
}

void trConsumer::ScheduleNextPacket()
{
	if(GetNode()->GetId() >35)
		return;

	double delay =GetNode()->GetId()  - 14  + 70;

	Simulator::Schedule (Seconds (delay), & trConsumer::SendPacket, this);
}

void trConsumer::SendPacket()
{

	  if (!m_active) return;
	// cout<<"consumer send packet"<<endl;

	  uint32_t num = GetNode()->GetId() % 3 + 1;
	  Name prefix("/");
	  prefix.appendNumber(num);

	  Ptr<Interest> interest = Create<Interest> (Create<Packet>(m_virtualPayloadSize));
	  Ptr<Name> interestName = Create<Name> (prefix);
	  interest->SetName(interestName);
	  interest->SetNonce(m_rand.GetValue());//just generate a random number
	  //interest->SetInterestLifetime    (m_interestLifeTime);

	   //add header;
	  ndn::nrndn::trndnHeader nrheader;
	  nrheader.setSourceId(GetNode()->GetId());
	  nrheader.setCurrentId(GetNode()->GetId());
	  nrheader.setX(m_sensor->getX());
	  nrheader.setY(m_sensor->getY());

	  Ptr<Packet> newPayload = Create<Packet> (m_virtualPayloadSize);
	  newPayload->AddHeader(nrheader);
	  interest->SetPayload(newPayload);

	  //cout<<"node: "<<GetNode()->GetId()<<"  send interest packet,name: "<<prefix.toUri()<<" in consumer"<<endl;

	  m_transmittedInterests (interest, this, m_face);
	  m_face->ReceiveInterest (interest);

	  interestSent[interest->GetNonce()] = prefix.toUri() ;
	  msgTime[interest->GetNonce()] = Simulator::Now().GetSeconds();

	  nrUtils:: IncreaseInterestedNodeSum();
	  cout<<"now InterestedNodeSum = "<<nrUtils::InterestedNodeSum<<endl;

}

void trConsumer::OnData(Ptr<const Data> data)
{
	 if (!m_active) return;
	NS_LOG_FUNCTION (this);
	// cout<<"consumer on data"<<endl;
	Ptr<Packet> nrPayload	= data->GetPayload()->Copy();
	trndnHeader nrheader;
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
		std::cout<<m_node->GetId()<<"\treceived data "<<name.toUri()<<" from "<<nodeId<<"\tSignature "<<signature<<" delay"<<delay<<endl;
		 cout<<"now InterestedNodeReceivedSum = "<<nrUtils::InterestedNodeReceivedSum<<"  now InterestedNodeSum = "<<nrUtils::InterestedNodeSum<<endl;
		interestSent.erase(it);
	}

}


void trConsumer::NotifyNewAggregate()
{
  super::NotifyNewAggregate ();
}

void trConsumer::DoInitialize(void)
{
	 //cout<<"consumer send packet"<<endl;
	if (m_forwardingStrategy == 0)
	{
		//m_forwardingStrategy = GetObject<fw::nrndn::NavigationRouteHeuristic>();
		Ptr<ForwardingStrategy> forwardingStrategy=m_node->GetObject<ForwardingStrategy>();
		NS_ASSERT_MSG(forwardingStrategy,"trConsumer::DoInitialize cannot find ns3::ndn::fw::ForwardingStrategy");
		m_forwardingStrategy = DynamicCast<fw::nrndn::TrNavigationRouteHeuristic>(forwardingStrategy);
	}
	if (m_sensor == 0)
	{
		m_sensor =  m_node->GetObject<ndn::nrndn::NodeSensor>();
		NS_ASSERT_MSG(m_sensor,"trConsumer::DoInitialize cannot find ns3::ndn::nrndn::NodeSensor");

	}
	super::DoInitialize();
}

void trConsumer::OnTimeout(uint32_t sequenceNumber)
{
	return;
}

void trConsumer::OnInterest(Ptr<const Interest> interest)
{
	NS_ASSERT_MSG(false,"trConsumer should not be supposed to "
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
