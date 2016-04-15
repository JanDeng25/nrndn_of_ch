/*
 * nrProducer.cc
 *
 *  Created on: Jan 12, 2015
 *      Author: chen
 */


#include "djProducer.h"
#include "NodeSensor.h"
#include "nrUtils.h"
#include "nrndn-Header.h"
#include "ndn-packet-type-tag.h"
#include "ns3/ndn-interest.h"
#include "ns3/ndn-data.h"
#include "ns3/string.h"
#include "ns3/log.h"
//#include "ns3/core-module.h"

#include "ns3/ndn-app-face.h"
#include "ns3/ndn-fib.h"

#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.h"

NS_LOG_COMPONENT_DEFINE ("ndn.nrndn.djProducer");

namespace ns3
{
namespace ndn
{
namespace nrndn
{
NS_OBJECT_ENSURE_REGISTERED (djProducer);

TypeId djProducer::GetTypeId()
{
	static TypeId tid = TypeId ("ns3::ndn::nrndn::djProducer")
			    .SetGroupName ("Nrndn")
			    .SetParent<App> ()
			    .AddConstructor<djProducer> ()
			    .AddAttribute ("Prefix","Prefix, for which producer has the data",
			                    StringValue ("/"),
			                    MakeNameAccessor (&djProducer::m_prefix),
			                    MakeNameChecker ())
			    .AddAttribute ("Postfix", "Postfix that is added to the output data (e.g., for adding producer-uniqueness)",
			                    StringValue ("/"),
			                    MakeNameAccessor (&djProducer::m_postfix),
			                    MakeNameChecker ())
			    .AddAttribute ("PayloadSize", "Virtual payload size for Content packets",
			                    UintegerValue (1024),
			                    MakeUintegerAccessor (&djProducer::m_virtualPayloadSize),
			                    MakeUintegerChecker<uint32_t> ())
			    .AddAttribute ("Freshness", "Freshness of data packets, if 0, then unlimited freshness",
			                    TimeValue (Seconds (0)),
			                    MakeTimeAccessor (&djProducer::m_freshness),
			                    MakeTimeChecker ())
			    .AddAttribute ("Signature", "Fake signature, 0 valid signature (default), other values application-specific",
			                    UintegerValue (0),
			                    MakeUintegerAccessor (&djProducer::m_signature),
			                    MakeUintegerChecker<uint32_t> ())
			    .AddAttribute ("KeyLocator", "Name to be used for key locator.  If root, then key locator is not used",
			                    NameValue (),
			                    MakeNameAccessor (&djProducer::m_keyLocator),
			                    MakeNameChecker ())
			    ;
			  return tid;
}


djProducer::djProducer():
		m_rand (0, std::numeric_limits<uint32_t>::max ()),
		m_virtualPayloadSize(1024),
		m_signature(0)
{
	//NS_LOG_FUNCTION(this);
}

djProducer::~djProducer()
{
	// TODO Auto-generated destructor stub
}



void djProducer::StartApplication()
{
	NS_LOG_FUNCTION_NOARGS ();
	NS_ASSERT(GetNode()->GetObject<Fib>() != 0);//??????????????????????????????????

	if(m_forwardingStrategy)
		m_forwardingStrategy->Start();

	App::StartApplication();
	NS_LOG_INFO("NodeID: " << GetNode ()->GetId ());

	double delay= GetNode ()->GetId () / 2;

	Simulator::Schedule (Seconds (50.0+ delay), &djProducer::sendResourcePacket, this);
	std::cout<<"siu:"<<"Start producer Application: " << GetNode ()->GetId ()<<endl;
}

void djProducer::StopApplication()
{
	NS_LOG_FUNCTION_NOARGS ();
	NS_ASSERT(GetNode()->GetObject<Fib>() != 0);

	if(m_forwardingStrategy)
		m_forwardingStrategy->Stop();

	std::cout<<"siu:"<<"Stop producer Application:" << GetNode ()->GetId ()<<endl;
	App::StopApplication();
}

void djProducer::OnInterest(Ptr<const Interest> interest)
{
	NS_ASSERT_MSG(false,"djProducer should not be supposed to"
				" receive Interest Packet!!");
}

bool djProducer::isJuction(string lane)
{
	for(uint32_t i = 0; i<lane.length(); ++i)
		if(lane[i] == 't')
			return false;
	return true;
}

void djProducer::sendResourcePacket()
{
	////m_sensor->getLane();
	if (!m_active)  return;
	if( GetNode()->GetId() >=6) return;
	 //cout<<"producer send resource packet"<<endl;
	if(isJuction(m_sensor->getLane()))
	{
		 Simulator::Schedule (Seconds (3.0), &djProducer::sendResourcePacket, this);//十字路口的车辆不参与
		 return;
	}

	uint32_t num = GetNode()->GetId() % 3+ 1;
	Name prefix("/");
	//std::cout<<"siu:"<<GetNode()->GetId()<<"sendResourcePacket:"<<m_prefix.toUri()<<std::endl;
	prefix.appendNumber(num);
	//std::cout<<"siu:"<<GetNode()->GetId()<<"sendResourcePacket:"<<prefix.toUri()<<std::endl;
	Ptr<Data> data = Create<Data>(Create<Packet>(m_virtualPayloadSize));
	Ptr<Name> dataName = Create<Name>(prefix);
	//dataName->append(m_postfix);

	data->SetName(dataName);
	data->SetFreshness(m_freshness);
	data->SetTimestamp(Simulator::Now());
	data->SetSignature(m_rand.GetValue());//just generate a random number

	if (m_keyLocator.size() > 0)
	{
		data->SetKeyLocator(Create<Name>(m_keyLocator));
	}

	ndn::nrndn::nrndnHeader nrheader;
	nrheader.setSourceId(GetNode()->GetId() );
	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	string lane = m_sensor->getLane();
	nrheader.setCurrentLane(lane);
	nrheader.setPreLane(lane);
	Ptr<Packet> newPayload	= Create<Packet> (m_virtualPayloadSize);

	newPayload->AddHeader(nrheader);

	data->SetPayload(newPayload);

	PacketTypeTag typeTag(RESOURCE_PACKET );
	data->GetPayload()->AddPacketTag(typeTag);

	FwHopCountTag hopCountTag;
	data->GetPayload()->AddPacketTag(hopCountTag);

	std::cout<<"siu:"<<"node("<< GetNode()->GetId() <<")\t send Resource Packet in producer:" << data->GetName ()<<",signature:"<<data->GetSignature()<<std::endl;

	m_face->ReceiveData(data);
	m_transmittedDatas(data, this, m_face);

	Simulator::Schedule (Seconds (10.0), &djProducer::sendResourcePacket, this);//每十秒钟发送一次

	//Simulator::Schedule (Seconds (300.0), &nrProducer::sendResourcePacket, this);
}

//Initialize the callback function after the base class initialize
void djProducer::DoInitialize(void)
{
	if (m_forwardingStrategy == 0)
	{
		Ptr<ForwardingStrategy> forwardingStrategy=m_node->GetObject<ForwardingStrategy>();
		NS_ASSERT_MSG(forwardingStrategy,"djProducer::DoInitialize cannot find ns3::ndn::fw::ForwardingStrategy");
		if(forwardingStrategy)
			m_forwardingStrategy = DynamicCast<fw::nrndn::djNavigationRouteHeuristic>(forwardingStrategy);
	}

	if (m_sensor == 0)
	{
		m_sensor = m_node->GetObject<NodeSensor>();
		NS_ASSERT_MSG(m_sensor,"djProducer::DoInitialize cannot find ns3::ndn::nrndn::NodeSensor");
	}
	super::DoInitialize();
}

void djProducer::NotifyNewAggregate()
{
	super::NotifyNewAggregate();
}

bool djProducer::IsActive()
{
	return m_active;
}

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */


