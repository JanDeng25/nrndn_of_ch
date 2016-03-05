/*
 * ndn-navigation-route-heuristic-forwarding.cc
 *
 *  Created on: Jan 14, 2015
 *      Author: chen
 */

#include "ndn-navigation-route-heuristic-forwarding.h"
#include "NodeSensor.h"
#include "nrHeader.h"
#include "nrndn-Header.h"
#include "nrndn-Neighbors.h"
#include "ndn-pit-entry-nrimpl.h"
#include "ndn-fib-entry-nrimpl.h"
#include "nrUtils.h"
#include "ndn-packet-type-tag.h"

#include "ns3/core-module.h"
#include "ns3/ptr.h"
#include "ns3/ndn-interest.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.h"

#include <algorithm>    // std::find

NS_LOG_COMPONENT_DEFINE ("ndn.fw.NavigationRouteHeuristic");

namespace ns3
{
namespace ndn
{
namespace fw
{
namespace nrndn
{

NS_OBJECT_ENSURE_REGISTERED (NavigationRouteHeuristic);

TypeId NavigationRouteHeuristic::GetTypeId(void)
{
	  static TypeId tid = TypeId ("ns3::ndn::fw::nrndn::NavigationRouteHeuristic")
	    .SetGroupName ("Ndn")
	    .SetParent<GreenYellowRed> ()
	    .AddConstructor<NavigationRouteHeuristic>()
	    .AddAttribute ("HelloInterval", "HELLO messages emission interval.",
	            TimeValue (Seconds (1)),
	            MakeTimeAccessor (&NavigationRouteHeuristic::HelloInterval),
	            MakeTimeChecker ())
	     .AddAttribute ("AllowedHelloLoss", "Number of hello messages which may be loss for valid link.",
	            UintegerValue (2),
	            MakeUintegerAccessor (&NavigationRouteHeuristic::AllowedHelloLoss),
	            MakeUintegerChecker<uint32_t> ())

	   	 .AddAttribute ("gap", "the time gap between interested nodes and disinterested nodes for sending a data packet.",
	   	        UintegerValue (20),
	   	        MakeUintegerAccessor (&NavigationRouteHeuristic::m_gap),
	   	        MakeUintegerChecker<uint32_t> ())
//		.AddAttribute ("CacheSize", "The size of the cache which records the packet sent, use LRU scheme",
//				UintegerValue (6000),
//				MakeUintegerAccessor (&NavigationRouteHeuristic::SetCacheSize,
//									  &NavigationRouteHeuristic::GetCacheSize),
//				MakeUintegerChecker<uint32_t> ())
        .AddAttribute ("UniformRv", "Access to the underlying UniformRandomVariable",
        		 StringValue ("ns3::UniformRandomVariable"),
        		 MakePointerAccessor (&NavigationRouteHeuristic::m_uniformRandomVariable),
        		 MakePointerChecker<UniformRandomVariable> ())
        .AddAttribute ("HelloLogEnable", "the switch which can turn on the log on Functions about hello messages",
        		 BooleanValue (true),
        		 MakeBooleanAccessor (&NavigationRouteHeuristic::m_HelloLogEnable),
        		 MakeBooleanChecker())
        .AddAttribute ("NoFwStop", "When the PIT covers the nodes behind, no broadcast stop message",
        		 BooleanValue (false),
        		 MakeBooleanAccessor (&NavigationRouteHeuristic::NoFwStop),
        		 MakeBooleanChecker())
		.AddAttribute ("TTLMax", "This value indicate that when a data is received by disinterested node, the max hop count it should be forwarded",
		         UintegerValue (3),
		         MakeUintegerAccessor (&NavigationRouteHeuristic::m_TTLMax),
		         MakeUintegerChecker<uint32_t> ())
		 .AddAttribute ("PayloadSize", "Virtual payload size for Content packets",
				 UintegerValue (1024),
				 MakeUintegerAccessor (&NavigationRouteHeuristic::m_virtualPayloadSize),
				  MakeUintegerChecker<uint32_t> ())
		 .AddAttribute ("Freshness", "Freshness of data packets, if 0, then unlimited freshness",
				 TimeValue (Seconds (0)),
				  MakeTimeAccessor (&NavigationRouteHeuristic::m_freshness),
				  MakeTimeChecker ())
//		    .AddAttribute ("Pit","pit of forwarder",
//	    		  PointerValue (),
//		    	  MakePointerAccessor (&NavigationRouteHeuristic::m_pit),
//		    	  MakePointerChecker<ns3::ndn::pit::nrndn::NrPitImpl> ())
//		    .AddAttribute ("Fib","fib of forwarder",
//		    	  PointerValue (),
//		    	  MakePointerAccessor (&NavigationRouteHeuristic::m_fib),
//		    	  MakePointerChecker<ns3::ndn::fib::nrndn::NrFibImpl> ())
//			.AddAttribute ("CS","cs of forwarder",
//	    		  PointerValue (),
//		    	  MakePointerAccessor (&NavigationRouteHeuristic::m_cs),
 //		     MakePointerChecker<ns3::ndn::cs::nrndn::NrPitImpl> ())
	    ;
	  return tid;
}

NavigationRouteHeuristic::NavigationRouteHeuristic():
	HelloInterval (Seconds (1)),
	AllowedHelloLoss (2),
	m_htimer (Timer::CANCEL_ON_DESTROY),
	m_timeSlot(Seconds (0.05)),
	m_CacheSize(5000),// Cache size can not change. Because if you change the size, the m_interestNonceSeen and m_dataNonceSeen also need to change. It is really unnecessary
	m_interestNonceSeen(m_CacheSize),
	m_dataSignatureSeen(m_CacheSize),
	m_nb (HelloInterval),
	m_running(false),
	m_runningCounter(0),
	m_HelloLogEnable(true),
	m_gap(10),
	m_TTLMax(3),
	NoFwStop(false),
	m_virtualPayloadSize(1024)
{
	m_htimer.SetFunction (&NavigationRouteHeuristic::HelloTimerExpire, this);
	m_nb.SetCallback (MakeCallback (&NavigationRouteHeuristic::FindBreaksLinkToNextHop, this));
}

NavigationRouteHeuristic::~NavigationRouteHeuristic ()
{

}

void NavigationRouteHeuristic::Start()
{
	NS_LOG_FUNCTION (this);
	if(!m_runningCounter)
	{
		m_running = true;
		m_offset = MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100));
		m_htimer.Schedule(m_offset);
		m_nb.ScheduleTimer();
		//cout<<"lane:"<<m_sensor->getLane()<<endl;
	}
	m_runningCounter++;

	if(m_node->GetId() < 10)
	{
		uint32_t num =m_node->GetId() % 3 + 1;
		Ptr<Name> dataName = Create<Name>("/");
		dataName->appendNumber(num);
		Ptr<Data> data = Create<Data>(Create<Packet>(m_virtualPayloadSize));
		data->SetName(dataName);
		//cout<<"before add"<<endl;
		m_cs->Add(data);
		//cout<<"after add"<<endl;
	}
}

void NavigationRouteHeuristic::Stop()
{
	NS_LOG_FUNCTION (this);
	if(m_runningCounter)
		m_runningCounter--;
	else
		return;

	if(m_runningCounter)
		return;
	m_running = false;
	m_htimer.Cancel();
	m_nb.CancelTimer();
}

void NavigationRouteHeuristic::WillSatisfyPendingInterest(
		Ptr<Face> inFace, Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

bool NavigationRouteHeuristic::DoPropagateInterest(
		Ptr<Face> inFace, Ptr<const Interest> interest,
		Ptr<pit::Entry> pitEntry)
{
	  NS_LOG_FUNCTION (this);
	  NS_LOG_UNCOND(this <<" is in unused function");
	  NS_ASSERT_MSG (m_pit != 0, "PIT should be aggregated with forwarding strategy");

	  return true;
}

void NavigationRouteHeuristic::WillEraseTimedOutPendingInterest(
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void NavigationRouteHeuristic::AddFace(Ptr<Face> face)
{
	//every time face is added to NDN stack?
	NS_LOG_FUNCTION(this);
	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" add application face "<<face->GetId());
		m_inFaceList.push_back(face);
	}
	else
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" add NOT application face "<<face->GetId());
		m_outFaceList.push_back(face);
	}
}

void NavigationRouteHeuristic::RemoveFace(Ptr<Face> face)
{
	NS_LOG_FUNCTION(this);
	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" remove application face "<<face->GetId());
		m_inFaceList.erase(find(m_inFaceList.begin(),m_inFaceList.end(),face));
	}
	else
	{
		NS_LOG_DEBUG("Node "<<m_node->GetId()<<" remove NOT application face "<<face->GetId());
		m_outFaceList.erase(find(m_outFaceList.begin(),m_outFaceList.end(),face));
	}
}

void NavigationRouteHeuristic::DidReceiveValidNack(
		Ptr<Face> incomingFace, uint32_t nackCode, Ptr<const Interest> nack,
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void NavigationRouteHeuristic::OnInterest(Ptr<Face> face,
		Ptr<Interest> interest)
{
	//NS_LOG_UNCOND("Here is NavigationRouteHeuristic dealing with OnInterest");
	//NS_LOG_FUNCTION (this);
	if(!m_running) return;

	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Get interest packet from APPLICATION");

		cout<<"node: "<<m_node->GetId()<<" receive interest in forwarder"<<endl;
		//cout<<"bad bug"<<endl;
		if(m_fib->Find(interest->GetName())){
			cout<<1<<endl;
			PrepareInterestPacket(interest);
		    cout<<2<<endl;
		}
		else{
			cout<<3<<endl;
			PrepareDetectPacket(interest);
			cout<<4<<endl;
		}

		return;
	}

	if(HELLO_PACKET  == interest->GetScope())
	{
		ProcessHello(interest);
		return;
	}

	//If the interest packet has already been sent, do not proceed the packet
	if(m_interestNonceSeen.Get(interest->GetNonce()))
	{
		NS_LOG_DEBUG("The interest packet has already been sent, do not proceed the packet of "<<interest->GetNonce());
		return;
	}

	Ptr<const Packet> nrPayload	= interest->GetPayload();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->PeekHeader(nrheader);

	double x = nrheader.getX();
	double y = nrheader.getY();
	uint32_t nodeId=nrheader.getSourceId();
	uint32_t seq = interest->GetNonce();
	std::string currentLane = nrheader.getCurrentLane();
	std::string preLane = nrheader.getPreLane();
	std::vector<std::string> laneList = nrheader.getLaneList();

	double disX = ( x - m_sensor->getX()) > 0 ? (x - m_sensor->getX()) : (m_sensor->getX() - x);
	double disY = ( y - m_sensor->getY()) > 0 ? (y - m_sensor->getY()) : (m_sensor->getY() - y);
	double distance = disX + disY;
	double interval = (500 - distance);

	if(DETECT_PACKET == interest->GetScope())
	{
		if(!isDuplicatedInterest(nodeId,seq))
		{
			cout<<"node: "<<m_node->GetId()<<" receive detect packet from "<<nodeId<<endl;
			if(m_fib->Find(interest->GetName()))
			{
				Time sendInterval = MilliSeconds(distance);
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ReplyConfirmPacket, this,interest);//回复的确认包，设置为此探测包的nonce和nodeid
				return;
			}
			else
			{
				Time sendInterval = (MilliSeconds(interval) +  m_gap * m_timeSlot);
				m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ForwardDetectPacket, this,interest);
				return;
			}
		}
		else
		{
			m_interestNonceSeen.Put(interest->GetNonce(),true);
			ExpireInterestPacketTimer(nodeId,seq);
			return;
		}
	}
	else if(INTEREST_PACKET == interest->GetScope())
	{
		if(!isDuplicatedInterest(nodeId,seq))
		{
			cout<<"node: "<<m_node->GetId()<<" receive detect packet from "<<nodeId<<endl;
			if(m_sensor->getLane() != currentLane && m_sensor->getLane() != preLane)
			{
				m_interestNonceSeen.Put(interest->GetNonce(),true);
				return;
			}
			else if(m_cs->Find(interest->GetName()))
			{
				Time sendInterval = MilliSeconds(distance);
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ReplyDataPacket, this,interest);//回复的确认包，设置为此探测包的nonce和nodeid
				return;
			}
			else if(m_pit->Find(interest->GetName()))
			{
				m_pit->UpdatePit(preLane, interest);
				m_interestNonceSeen.Put(interest->GetNonce(),true);
				return;
			}
			else if(m_pit->GetSize () == 0)
			{
				m_interestNonceSeen.Put(interest->GetNonce(),true);
				return;
			}
			else
			{
				if(m_fib->Find(interest->GetName()))
				{
					m_pit->UpdatePit(preLane, interest);
					if(m_sensor->getLane() == currentLane)
					{
						Time sendInterval = (MilliSeconds(interval) +  m_gap * m_timeSlot);
						m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
												&NavigationRouteHeuristic::ForwardInterestPacket, this,interest);
					}
					else
					{
						Time sendInterval = (MilliSeconds(interval) +  (m_gap+ 5) * m_timeSlot);
						m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
												&NavigationRouteHeuristic::ForwardInterestPacket, this,interest);
					}
					return;
				}
			}
		}//非重复包
		else//重复包
		{
			m_interestNonceSeen.Put(interest->GetNonce(),true);
			ExpireInterestPacketTimer(nodeId,seq);
			return;
		}
	}
	else if (MOVE_TO_NEW_LANE == interest->GetScope())
	{
		//更改pit表项?????????????????????????????????
		/*
		std::vector<Ptr<Entry> >::iterator pit=m_pit->m_pitContainer.begin();
		Ptr<Entry> entry = *pit;

			for(;pit!=m_pitContainer.end();++pit)
			{
				Ptr<EntryNrImpl> pitEntry = DynamicCast<EntryNrImpl>(*pit);
				//const name::Component &pitName=(*pit)->GetInterest()->GetName().get(0);
				if(pitEntry->getEntryName() == interest->GetName().toUri())
				{
					std::unordered_set< std::string >::const_iterator it = pitEntry->getIncomingnbs().find(lane);
					if(it==pitEntry->getIncomingnbs().end())
						pitEntry->AddIncomingNeighbors(lane);
					//os<<(*pit)->GetInterest()->GetName().toUri()<<" add Neighbor "<<id<<' ';
				}
			}
			*/
	}
	else if(ASK_FOR_TABLE == interest->GetScope())
	{
		Time sendInterval = MilliSeconds(distance);
		m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ReplyTablePacket, this,interest);//回复的确认包，设置为此探测包的nonce和nodeid
		return;
	}
}

void NavigationRouteHeuristic::OnData(Ptr<Face> face, Ptr<Data> data)
{
	NS_LOG_FUNCTION (this);
	if(!m_running) return;
	if(Face::APPLICATION & face->GetFlags())
	{
		NS_LOG_DEBUG("Get data packet from APPLICATION");
		cout<<"node: "<<m_node->GetId()<<" receive data in forwarder"<<endl;
		// 2. record the Data Packet(only record the forwarded packet)
		m_dataSignatureSeen.Put(data->GetSignature(),true);

		// 3. Then forward the data packet directly
		Simulator::Schedule(
				MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100)),
				&NavigationRouteHeuristic::SendDataPacket, this, data);

		// 4. Although it is from itself, include into the receive record
		///////////NotifyUpperLayer(data);
		return;
	}

	//If the data packet has already been sent, do not proceed the packet
	if(m_dataSignatureSeen.Get(data->GetSignature()))
	{
		NS_LOG_DEBUG("The Data packet has already been sent, do not proceed the packet of "<<data->GetSignature());
		return;
	}

	Ptr<const Packet> nrPayload	= data->GetPayload();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->PeekHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload	->PeekPacketTag(hopCountTag);
	ndn::nrndn::PacketTypeTag packetTypeTag;
	nrPayload	->PeekPacketTag(packetTypeTag);
	double x = nrheader.getX();
	double y = nrheader.getY();
	uint32_t nodeId=nrheader.getSourceId();

	uint32_t signature=data->GetSignature();
	std::string currentLane = nrheader.getCurrentLane();
	std::string preLane = nrheader.getPreLane();
	std::vector<std::string> laneList = nrheader.getLaneList();

	double disX = ( x - m_sensor->getX()) > 0 ? (x - m_sensor->getX()) : (m_sensor->getX() - x);
	double disY = ( y - m_sensor->getY()) > 0 ? (y - m_sensor->getY()) : (m_sensor->getY() - y);
	double distance = disX + disY;
	double interval = (500 - distance) * 10;

	if(RESOURCE_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			cout<<"node: "<<m_node->GetId()<<" receive resource packet from "<<nodeId<<endl;

			if(m_sensor->getLane() == currentLane)
				m_fib-> AddFibEntry(data->GetNamePtr(),preLane, hopCountTag.Get() );
			else
				m_fib-> AddFibEntry(data->GetNamePtr(),currentLane, hopCountTag.Get() );

			Time sendInterval = (MilliSeconds(interval) + m_gap * m_timeSlot);
			m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&NavigationRouteHeuristic::ForwardResourcePacket, this,data);
			return;
		}//end if(!isDuplicatedData(nodeId,signature))
		else   //重复包
		{
			ExpireDataPacketTimer(nodeId,signature);
			m_dataSignatureSeen.Put(data->GetSignature(),true);
			return;
		}//end duplicate packet
	}//end if (RESOURCE_PACKET == packetTypeTag.Get())
	else if (DATA_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			m_cs->Add(data);
			if(isDuplicatedInterest(nodeId,signature))
			{
					ExpireInterestPacketTimer(nodeId,signature);
			}
			if(m_pit->Find(data->GetName()))
			{
				if(OnTheWay(laneList))
				{
					m_pit->RemovePitEntry(data->GetName());
					Time sendInterval = (MilliSeconds(interval) + m_gap * m_timeSlot);
					m_sendingDataEvent[nodeId][signature]=
									Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ForwardDataPacket, this,data);
					return;
				}
				else if(m_sensor->getLane() == preLane)
				{
					m_pit->RemovePitEntry(data->GetName());
					Time sendInterval = (MilliSeconds(interval) + (m_gap+5) * m_timeSlot);
					m_sendingDataEvent[nodeId][signature]=
									Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ForwardDataPacket, this,data);
					return;
				}
			}
			else
			{
				m_dataSignatureSeen.Put(data->GetSignature(),true);
				return;
			}
		}
		else//重复包
		{
			ExpireDataPacketTimer(nodeId,signature);
			m_dataSignatureSeen.Put(data->GetSignature(),true);
			return;
		}
	}
	else if(CONFIRM_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			 if(isDuplicatedInterest(nodeId,signature))
			 {
				 ExpireInterestPacketTimer(nodeId,signature);
			 }
			 if(m_fib->Find(data->GetName()))
			{
				m_dataSignatureSeen.Put(data->GetSignature(),true);
				return;
			}
			//建立FIB表项
			if(m_sensor->getLane() == currentLane)
			{
				Time sendInterval = (MilliSeconds(interval) + m_gap * m_timeSlot);
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&NavigationRouteHeuristic::ForwardConfirmPacket, this,data);
				m_pit->RemovePitEntry(data->GetName());
				return;
			}
			else if(m_sensor->getLane() == preLane)
			{
				Time sendInterval = (MilliSeconds(interval) + (m_gap+5) * m_timeSlot);
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&NavigationRouteHeuristic::ForwardConfirmPacket, this,data);
				m_pit->RemovePitEntry(data->GetName());
				return;
			}
			else
			{
				m_dataSignatureSeen.Put(data->GetSignature(),true);
				return;
			}
		}
		else
		{
			Ptr<ndn::fib::nrndn::EntryNrImpl> nexthop;
			nexthop = DynamicCast<ndn::fib::nrndn::EntryNrImpl>(m_fib->Find(data->GetName()));
			if(nexthop->getIncomingnbs().begin()->second > nrheader.getTTL())
			{
				ExpireDataPacketTimer(nodeId,signature);
				m_dataSignatureSeen.Put(data->GetSignature(),true);
				return;
			}
		}
	}
	else if(TABLE_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			if(m_pit->GetSize() == 0)
			{
					//维护表格
			}
		}
		else
		{
			ExpireDataPacketTimer(nodeId,signature);
			m_dataSignatureSeen.Put(data->GetSignature(),true);
			return;
		}
	}
	return;
}

bool NavigationRouteHeuristic::OnTheWay(std::vector<std::string> laneList)
{
	for(uint32_t i = 0; i < laneList.size(); ++i)
		if(m_sensor->getLane() == laneList[i])
			return true;
	return false;
}

bool NavigationRouteHeuristic::isDuplicatedInterest(
		uint32_t id, uint32_t nonce)
{
	NS_LOG_FUNCTION (this);
	if(!m_sendingInterestEvent.count(id))
		return false;
	else
		return m_sendingInterestEvent[id].count(nonce);
}

bool NavigationRouteHeuristic::isDuplicatedData(uint32_t id, uint32_t signature)
{
	NS_LOG_FUNCTION (this);
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::isDuplicatedData");
	if(!m_sendingDataEvent.count(id))
		return false;
	else
		return m_sendingDataEvent[id].count(signature);
}

void NavigationRouteHeuristic::ExpireInterestPacketTimer(uint32_t nodeId,uint32_t seq)
{
	NS_LOG_FUNCTION (this<< "ExpireInterestPacketTimer id"<<nodeId<<"sequence"<<seq);
	//1. Find the waiting timer
	EventId& eventid = m_sendingInterestEvent[nodeId][seq];
	
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void NavigationRouteHeuristic::ExpireDataPacketTimer(uint32_t nodeId,uint32_t signature)
{
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::ExpireDataPacketTimer");
	NS_LOG_FUNCTION (this<< "ExpireDataPacketTimer id\t"<<nodeId<<"\tsignature:"<<signature);
	//1. Find the waiting timer
	EventId& eventid = m_sendingDataEvent[nodeId][signature];
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void NavigationRouteHeuristic::ForwardResourcePacket(Ptr<Data> src)
{
	if(!m_running) return;

	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	//Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	std::string currentlane = m_sensor->getLane();
	std::string prelane;
	if(currentlane == nrheader.getCurrentLane() )
		prelane = nrheader.getPreLane();
	else
		prelane = nrheader.getCurrentLane();

	// 	2.1 setup nrheader, source id do not change
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setCurrentLane(currentlane);
	nrheader.setPreLane(prelane);
	nrPayload->AddHeader(nrheader);

	// 	2.2 setup FwHopCountTag
	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	hopCountTag. Increment();
	if(hopCountTag.Get() > 15)
	{
		m_dataSignatureSeen.Put(src->GetSignature(),true);
		return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	// 	2.3 copy the data packet, and install new payload to data
	Ptr<Data> data = Create<Data> (*src);
	data->SetPayload(nrPayload);

	m_dataSignatureSeen.Put(src->GetSignature(),true);
	cout<<"node: "<<m_node->GetId()<<" forward resource packet from "<<nrheader.getSourceId()<<endl;
	SendDataPacket(data);

	ndn::nrndn::nrUtils::IncreaseForwardCounter(nrheader.getSourceId(), data->GetSignature());
}

void NavigationRouteHeuristic::ForwardConfirmPacket(Ptr<Data> src)
{
	if(!m_running) return;

	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	//Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	std::vector<std::string> lanelist = nrheader.getLaneList();
	std::string currentlane;
	for(uint32_t i = lanelist.size()-1; i >=0; --i)
	{
		if(i == 0)
			currentlane = lanelist[i];
		else if(m_sensor->getLane() == lanelist[i])
		{
			currentlane = lanelist[i-1];
			break;
		}
	}
	std::string prelane = m_sensor->getLane();

	// 	2.1 setup nrheader, source id do not change
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setCurrentLane(currentlane);
	nrheader.setPreLane(prelane);
	nrPayload->AddHeader(nrheader);

	// 	2.2 setup FwHopCountTag
	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	hopCountTag. Increment();
	if(hopCountTag.Get() > 5)
	{
		m_dataSignatureSeen.Put(src->GetSignature(),true);
		return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	// 	2.3 copy the data packet, and install new payload to data
	Ptr<Data> data = Create<Data> (*src);
	data->SetPayload(nrPayload);

	m_dataSignatureSeen.Put(src->GetSignature(),true);
	cout<<"node: "<<m_node->GetId()<<" forward confirm packet from "<<nrheader.getSourceId()<<endl;
	SendDataPacket(data);

	ndn::nrndn::nrUtils::IncreaseForwardCounter(nrheader.getSourceId(), data->GetSignature());
}

void NavigationRouteHeuristic::ForwardDataPacket(Ptr<Data> src)
{
		if(!m_running) return;

		Ptr<Packet> nrPayload=src->GetPayload()->Copy();
		ndn::nrndn::nrndnHeader nrheader;
		nrPayload->RemoveHeader(nrheader);
		double x= m_sensor->getX();
		double y= m_sensor->getY();
		std::string prelane = m_sensor->getLane();
		std::vector<std::string> lanelist;
		///检查PIT表项，维护lanelist（备选下一跳）
		Ptr<ndn::pit::nrndn::EntryNrImpl> nexthop;
		nexthop = DynamicCast<ndn::pit::nrndn::EntryNrImpl>(m_pit->Find( src->GetName()));
		std::unordered_set< std::string >::const_iterator it;
		for(it =nexthop->getIncomingnbs().begin(); it != nexthop->getIncomingnbs().end(); ++it)
		{
					lanelist.push_back(*it);
		}

		// 	2.1 setup nrheader, source id do not change
		nrheader.setX(x);
		nrheader.setY(y);
		nrheader.setPreLane(prelane);
		nrheader.setLaneList(lanelist);
		nrPayload->AddHeader(nrheader);

		// 	2.2 setup FwHopCountTag
		FwHopCountTag hopCountTag;
		nrPayload->RemovePacketTag( hopCountTag);
		hopCountTag. Increment();
		if(hopCountTag.Get() > 15)
		{
			m_dataSignatureSeen.Put(src->GetSignature(),true);
			return;
		}
		nrPayload->AddPacketTag(hopCountTag);

		// 	2.3 copy the data packet, and install new payload to data
		Ptr<Data> data = Create<Data> (*src);
		data->SetPayload(nrPayload);

		m_dataSignatureSeen.Put(src->GetSignature(),true);
		cout<<"node: "<<m_node->GetId()<<" forward data packet from "<<nrheader.getSourceId()<<endl;

		SendDataPacket(data);

		ndn::nrndn::nrUtils::IncreaseForwardCounter(nrheader.getSourceId(), data->GetSignature());
		//ndn::nrndn::nrUtils::IncreaseDataForwardCounter(nrheader.getSourceId(), data->GetSignature());
}

void NavigationRouteHeuristic::ForwardDetectPacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);

	// 2. prepare the interest
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	std::vector<std::string> lanelist = nrheader.getLaneList();
	if(m_sensor->getLane() != lanelist.back())
	{
		lanelist.push_back(m_sensor->getLane() );
	}
	double x= m_sensor->getX();
	double y= m_sensor->getY();

	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setLaneList(lanelist);
	nrPayload->AddHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	hopCountTag. Increment();
	if(hopCountTag.Get() > 5)
	{
			m_interestNonceSeen.Put(src->GetNonce(),true);
			return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	Ptr<Interest> interest = Create<Interest> (*src);
	interest->SetPayload(nrPayload);

	// 3. Send the interest Packet. Already wait, so no schedule
	m_interestNonceSeen.Put(src->GetNonce(),true);
	cout<<"node: "<<m_node->GetId()<<" forward detect packet from "<<nrheader.getSourceId()<<endl;
	SendInterestPacket(interest);
	ndn::nrndn::nrUtils::IncreaseForwardCounter(nrheader.getSourceId(), interest->GetNonce());
}

void NavigationRouteHeuristic::ForwardInterestPacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);

	// 2. prepare the interest
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	std::vector<std::string> lanelist; //= nrheader.getLaneList();lanelist in pit!!!!!!!!!!!!!!!
	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	nrheader.setPreLane(m_sensor->getLane());
	nrheader.setLaneList(lanelist);
	nrPayload->AddHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	hopCountTag. Increment();
	if(hopCountTag.Get() > 15)
	{
			m_interestNonceSeen.Put(src->GetNonce(),true);
			return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	Ptr<Interest> interest = Create<Interest> (*src);
	interest->SetPayload(nrPayload);

	// 3. Send the interest Packet. Already wait, so no schedule
	m_interestNonceSeen.Put(src->GetNonce(),true);
	cout<<"node: "<<m_node->GetId()<<" forward interest packet from "<<nrheader.getSourceId()<<endl;
	SendInterestPacket(interest);

	ndn::nrndn::nrUtils::IncreaseForwardCounter(nrheader.getSourceId(), interest->GetNonce());
	ndn::nrndn::nrUtils::IncreaseInterestForwardCounter(nrheader.getSourceId(), interest->GetNonce());
}

void NavigationRouteHeuristic::ReplyConfirmPacket(Ptr<Interest> interest)
{
	if (!m_running)  return;

	Ptr<Data> data = Create<Data>(Create<Packet>(m_virtualPayloadSize));
	Ptr<Name> dataName = Create<Name>(interest->GetName());
	data->SetName(dataName);
	data->SetFreshness(m_freshness);
	data->SetTimestamp(Simulator::Now());
	data->SetSignature(interest->GetNonce());//just generate a random number

	ndn::nrndn::nrndnHeader nrheader;
	interest->GetPayload()->PeekHeader(nrheader);
	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	std::string currentlane = nrheader.getLaneList()[0] ;
	for(uint32_t i = nrheader.getLaneList().size()-1; i>=0; --i)
	{
		if( nrheader.getLaneList()[i] != m_sensor->getLane() )
		{
			currentlane = nrheader.getLaneList()[i];
			break;
		}
	}
	nrheader.setCurrentLane(currentlane);
	nrheader.setPreLane(m_sensor->getLane());
	Ptr<ndn::fib::nrndn::EntryNrImpl> nexthop;
	nexthop = DynamicCast<ndn::fib::nrndn::EntryNrImpl>(m_fib->Find(interest->GetName()));
	uint32_t ttl = (nexthop->getIncomingnbs()).begin()->second;
	nrheader.setTTL(ttl);

	Ptr<Packet> newPayload	= Create<Packet> ();
	newPayload->AddHeader(nrheader);

	data->SetPayload(newPayload);

	ndn::nrndn::PacketTypeTag typeTag(CONFIRM_PACKET );
	data->GetPayload()->AddPacketTag(typeTag);

	FwHopCountTag hopCountTag;
	data->GetPayload()->AddPacketTag(hopCountTag);

	m_dataSignatureSeen.Put(data->GetSignature(),true);
	cout<<"node: "<<m_node->GetId()<<" reply confirm packet to "<<nrheader.getSourceId()<<endl;

	SendDataPacket(data);
}

void NavigationRouteHeuristic::ReplyTablePacket(Ptr<Interest> interest)
{
	//将表格打包，signature和nodeid与interest相同
	//不需m_dataSignatureSeen.Put(interest->GetNonce(),true);
	cout<<"node: "<<m_node->GetId()<<" reply table packet to "/*<<nrheader.getSourceId()*/<<endl;
	//SendDataPacket(data);
}

void NavigationRouteHeuristic::ReplyDataPacket(Ptr<Interest> interest)
{
	if (!m_running)  return;

	Ptr<Data> data = Create<Data>(Create<Packet>(m_virtualPayloadSize));
	Ptr<Name> dataName = Create<Name>(interest->GetName());
	data->SetName(dataName);
	data->SetFreshness(m_freshness);
	data->SetTimestamp(Simulator::Now());
	data->SetSignature(interest->GetNonce());

	ndn::nrndn::nrndnHeader nrheader;
	interest->GetPayload()->PeekHeader(nrheader);
	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	nrheader.setCurrentLane(nrheader.getPreLane());
	nrheader.setPreLane(m_sensor->getLane());
	//sourceId not change

	Ptr<Packet> newPayload	= Create<Packet> ();
	newPayload->AddHeader(nrheader);

	data->SetPayload(newPayload);

	ndn::nrndn::PacketTypeTag typeTag(DATA_PACKET );
	data->GetPayload()->AddPacketTag(typeTag);

	FwHopCountTag hopCountTag;
	data->GetPayload()->AddPacketTag(hopCountTag);

	m_dataSignatureSeen.Put(data->GetSignature(),true);
	cout<<"node: "<<m_node->GetId()<<" reply data packet to "<<nrheader.getSourceId()<<endl;
	SendDataPacket(data);
}

void NavigationRouteHeuristic::PrepareInterestPacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);

	// 2. prepare the interest
	Ptr<Packet> nrPayload= interest->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	Ptr<ndn::fib::nrndn::EntryNrImpl> nexthop;
	nexthop = DynamicCast<ndn::fib::nrndn::EntryNrImpl>(m_fib->Find(interest->GetName()));
	std:: string hop;
	hop = (nexthop->getIncomingnbs()).begin()->first	;
	nrheader.setCurrentLane(hop);
	nrPayload->AddHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload->AddPacketTag(hopCountTag);

	ndn::nrndn::PacketTypeTag typeTag(INTEREST_PACKET);
	nrPayload->AddPacketTag (typeTag);

	interest->SetPayload(nrPayload);

	cout<<"node: "<<m_node->GetId()<<"  send interest packet,name: "<<interest->GetName().toUri()<<" in forwarder"<<endl;

	m_interestNonceSeen.Put(interest->GetNonce(),true);

	Simulator::Schedule(MilliSeconds(m_uniformRandomVariable->GetInteger(0,100)),
					&NavigationRouteHeuristic::SendInterestPacket,this,interest);
}

void NavigationRouteHeuristic::PrepareDetectPacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);

	FwHopCountTag hopCountTag;
	interest->GetPayload()->AddPacketTag(hopCountTag);

	ndn::nrndn::PacketTypeTag typeTag(DETECT_PACKET);
	interest->GetPayload()->AddPacketTag (typeTag);

	cout<<"node: "<<m_node->GetId()<<"  send detect packet,name: "<<interest->GetName().toUri()<<" in forwarder"<<endl;

	m_interestNonceSeen.Put(interest->GetNonce(),true);

	Simulator::Schedule(MilliSeconds(m_uniformRandomVariable->GetInteger(0,100)),
					&NavigationRouteHeuristic::SendInterestPacket,this,interest);
}

void NavigationRouteHeuristic::SendInterestPacket(Ptr<Interest> interest)
{
	if(!m_running) return;

	if(HELLO_PACKET !=interest->GetScope()||m_HelloLogEnable)
		NS_LOG_FUNCTION (this);

	//    if the node has multiple out Netdevice face, send the interest package to them all
	//    makde sure this is a NetDeviceFace!!!!!!!!!!!1
	vector<Ptr<Face> >::iterator fit;
	for(fit=m_outFaceList.begin();fit!=m_outFaceList.end();++fit)
	{
		(*fit)->SendInterest(interest);
		//////ndn::nrndn::nrUtils::AggrateInterestPacketSize(interest);
	}
}

void NavigationRouteHeuristic::SendDataPacket(Ptr<Data> data)
{
	if(!m_running) return;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::SendDataPacket");
	vector<Ptr<Face> >::iterator fit;
	for (fit = m_outFaceList.begin(); fit != m_outFaceList.end(); ++fit)
	{
		(*fit)->SendData(data);
		//////////ndn::nrndn::nrUtils::AggrateDataPacketSize(data);
	}
}

void NavigationRouteHeuristic::SendHello()
{
	if(!m_running) return;

	if (m_HelloLogEnable)
		NS_LOG_FUNCTION(this);
	const double& x		= m_sensor->getX();
	const double& y		= m_sensor->getY();
	const string& LaneName=m_sensor->getLane();
	//1.setup name
	Ptr<Name> name = ns3::Create<Name>('/'+LaneName);

	//2. setup payload
	Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrndnHeader nrheader;
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setSourceId(m_node->GetId());
	newPayload->AddHeader(nrheader);

	//3. setup interest packet
	Ptr<Interest> interest	= Create<Interest> (newPayload);
	interest->SetScope(HELLO_PACKET );	// The flag indicate it is hello message
	interest->SetName(name); //interest name is lane;

	//4. send the hello message
	SendInterestPacket(interest);
}

void NavigationRouteHeuristic::DoInitialize(void)
{
	if (m_sensor == 0)
	{
		m_sensor = m_node->GetObject<ndn::nrndn::NodeSensor>();
		NS_ASSERT_MSG(m_sensor,"NavigationRouteHeuristic::DoInitialize cannot find ns3::ndn::nrndn::NodeSensor");
	}
	super::DoInitialize();
}

void NavigationRouteHeuristic::DropPacket()
{
	NS_LOG_DEBUG ("Drop Packet");
}

void NavigationRouteHeuristic::DropDataPacket(Ptr<Data> data)
{
	NS_LOG_DEBUG ("Drop data Packet");
	/*
	 * @Date 2015-03-17 For statistics reason, count the disinterested data
	 * */
	//Choice 1:
	NotifyUpperLayer(data);

	//Choice 2:
	//DropPacket();
}

void NavigationRouteHeuristic::DropInterestePacket(Ptr<Interest> interest)
{
	NS_LOG_DEBUG ("Drop interest Packet");
	DropPacket();
}

void NavigationRouteHeuristic::NotifyNewAggregate()
{

  if (m_sensor == 0)
  {
	  m_sensor = GetObject<ndn::nrndn::NodeSensor> ();
   }

  if ( m_pit == 0)
  {
	  Ptr<Pit> pit=GetObject<Pit>();
	  if(pit)
		  m_pit = DynamicCast<pit::nrndn::NrPitImpl>(pit);
  }

  if (m_fib == 0)
   {
 	  Ptr<Fib> fib=GetObject<Fib>();
 	  if(fib){
 		 //cout<<"fib"<<endl;
 		 m_fib = DynamicCast<fib::nrndn::NrFibImpl>(fib);
 	  }
   }

  if (m_cs == 0)
  {
   	  Ptr<ContentStore> cs=GetObject<ContentStore>();
   	  if(cs!=0){
   		  //cout<<"cs"<<endl;
   		  m_cs = DynamicCast<cs::nrndn::NrCsImpl>(cs);
   		//cout<<"cs"<<endl;
   	  }
   }

  if(m_node==0)
  {
	  m_node=GetObject<Node>();
  }

  super::NotifyNewAggregate ();
}

void
NavigationRouteHeuristic::HelloTimerExpire ()
{
	if(!m_running) return;

	if (m_HelloLogEnable)
		NS_LOG_FUNCTION(this);
	SendHello();

	m_htimer.Cancel();
	Time base(HelloInterval - m_offset);
	m_offset = MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100));
	m_htimer.Schedule(base + m_offset);
}

void
NavigationRouteHeuristic::FindBreaksLinkToNextHop(uint32_t BreakLinkNodeId)
{
	 NS_LOG_FUNCTION (this);
}

void
NavigationRouteHeuristic::ProcessHello(Ptr<Interest> interest)
{
	if(!m_running) return;

	if(m_HelloLogEnable)
		NS_LOG_DEBUG (this << interest << "\tReceived HELLO packet from "<<interest->GetNonce());

	Ptr<const Packet> nrPayload	= interest->GetPayload();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->PeekHeader(nrheader);
	//update neighbor list
	m_nb.Update(nrheader.getSourceId(),nrheader.getX(),nrheader.getY(),Time (AllowedHelloLoss * HelloInterval));
}

void NavigationRouteHeuristic::ToContentStore(Ptr<Data> data)
{
	NS_LOG_DEBUG ("To content store.(Just a trace)");
	return;
}

void NavigationRouteHeuristic::NotifyUpperLayer(Ptr<Data> data)
{
	if(!m_running) return;
	// 1. record the Data Packet received
	m_dataSignatureSeen.Put(data->GetSignature(),true);

	// 2. notify upper layer
	vector<Ptr<Face> >::iterator fit;
	for (fit = m_inFaceList.begin(); fit != m_inFaceList.end(); ++fit)
	{
		//App::OnData() will be executed,
		//including nrProducer::OnData.
		//But none of its business, just ignore
		(*fit)->SendData(data);
	}
}

} /* namespace nrndn */
} /* namespace fw */
} /* namespace ndn */
} /* namespace ns3 */


