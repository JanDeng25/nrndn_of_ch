/*
 * ndn-navigation-route-heuristic-forwarding.cc
 *
 *  Created on: Jan 14, 2015
 *      Author: chen
 */

#include "trndn-navigation-route-heuristic-forwarding.h"
#include "NodeSensor.h"
#include "nrHeader.h"
#include "tableHeader.h"
#include "trndn-Header.h"
#include "nrndn-Neighbors.h"
#include "trndn-pit-entry-nrimpl.h"
#include "trndn-fib-entry-nrimpl.h"
#include "nrUtils.h"
#include "ndn-packet-type-tag.h"

#include "ns3/core-module.h"
#include "ns3/ndn-pit.h"
#include "ns3/ptr.h"
#include "ns3/ndn-interest.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/ndnSIM/utils/ndn-fw-hop-count-tag.h"

#include <algorithm>    // std::find
#include<math.h>
#include<vector>

NS_LOG_COMPONENT_DEFINE ("ndn.fw.TrNavigationRouteHeuristic");

namespace ns3
{
namespace ndn
{
namespace fw
{
namespace nrndn
{

NS_OBJECT_ENSURE_REGISTERED (TrNavigationRouteHeuristic);

TypeId TrNavigationRouteHeuristic::GetTypeId(void)
{
	  static TypeId tid = TypeId ("ns3::ndn::fw::nrndn::TrNavigationRouteHeuristic")
	    .SetGroupName ("Ndn")
	    .SetParent<GreenYellowRed> ()
	    .AddConstructor<TrNavigationRouteHeuristic>()
	    .AddAttribute ("HelloInterval", "HELLO messages emission interval.",
	            TimeValue (Seconds (1)),
	            MakeTimeAccessor (&TrNavigationRouteHeuristic::HelloInterval),
	            MakeTimeChecker ())
	     .AddAttribute ("AllowedHelloLoss", "Number of hello messages which may be loss for valid link.",
	            UintegerValue (2),
	            MakeUintegerAccessor (&TrNavigationRouteHeuristic::AllowedHelloLoss),
	            MakeUintegerChecker<uint32_t> ())

	   	 .AddAttribute ("gap", "the time gap between interested nodes and disinterested nodes for sending a data packet.",
	   	        UintegerValue (20),
	   	        MakeUintegerAccessor (&TrNavigationRouteHeuristic::m_gap),
	   	        MakeUintegerChecker<uint32_t> ())
//		.AddAttribute ("CacheSize", "The size of the cache which records the packet sent, use LRU scheme",
//				UintegerValue (6000),
//				MakeUintegerAccessor (&NavigationRouteHeuristic::SetCacheSize,
//									  &NavigationRouteHeuristic::GetCacheSize),
//				MakeUintegerChecker<uint32_t> ())
        .AddAttribute ("UniformRv", "Access to the underlying UniformRandomVariable",
        		 StringValue ("ns3::UniformRandomVariable"),
        		 MakePointerAccessor (&TrNavigationRouteHeuristic::m_uniformRandomVariable),
        		 MakePointerChecker<UniformRandomVariable> ())
        .AddAttribute ("HelloLogEnable", "the switch which can turn on the log on Functions about hello messages",
        		 BooleanValue (true),
        		 MakeBooleanAccessor (&TrNavigationRouteHeuristic::m_HelloLogEnable),
        		 MakeBooleanChecker())
        .AddAttribute ("NoFwStop", "When the PIT covers the nodes behind, no broadcast stop message",
        		 BooleanValue (false),
        		 MakeBooleanAccessor (&TrNavigationRouteHeuristic::NoFwStop),
        		 MakeBooleanChecker())
		.AddAttribute ("TTLMax", "This value indicate that when a data is received by disinterested node, the max hop count it should be forwarded",
		         UintegerValue (3),
		         MakeUintegerAccessor (&TrNavigationRouteHeuristic::m_TTLMax),
		         MakeUintegerChecker<uint32_t> ())
		 .AddAttribute ("PayloadSize", "Virtual payload size for Content packets",
				 UintegerValue (1024),
				 MakeUintegerAccessor (&TrNavigationRouteHeuristic::m_virtualPayloadSize),
				  MakeUintegerChecker<uint32_t> ())
		 .AddAttribute ("Freshness", "Freshness of data packets, if 0, then unlimited freshness",
				 TimeValue (Seconds (0)),
				  MakeTimeAccessor (&TrNavigationRouteHeuristic::m_freshness),
				  MakeTimeChecker ())
	    .AddAttribute ("Pit","pit of forwarder",
	    		  PointerValue (),
		    	  MakePointerAccessor (&TrNavigationRouteHeuristic::m_pit),
		    	  MakePointerChecker<ns3::ndn::pit::nrndn::TrNrPitImpl> ())
	    .AddAttribute ("Fib","fib of forwarder",
	    	      PointerValue (),
		    	  MakePointerAccessor (&TrNavigationRouteHeuristic::m_fib),
		    	  MakePointerChecker<ns3::ndn::fib::nrndn::TrNrFibImpl> ())
			.AddAttribute ("CS","cs of forwarder",
	    		  PointerValue (),
		    	  MakePointerAccessor (&TrNavigationRouteHeuristic::m_cs),
 		         MakePointerChecker<ns3::ndn::cs::nrndn::NrCsImpl> ())
	    ;
	  return tid;
}

TrNavigationRouteHeuristic::TrNavigationRouteHeuristic():
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
	m_htimer.SetFunction (&TrNavigationRouteHeuristic::HelloTimerExpire, this);
	m_nb.SetCallback (MakeCallback (&TrNavigationRouteHeuristic::FindBreaksLinkToNextHop, this));
}

TrNavigationRouteHeuristic::~TrNavigationRouteHeuristic ()
{

}

void TrNavigationRouteHeuristic::Start()
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

	if(m_node->GetId() <6)
	{
		uint32_t num =m_node->GetId() % 3 + 1;
		Ptr<Name> dataName = Create<Name>("/");
		dataName->appendNumber(num);
		Ptr<Data> data = Create<Data>(Create<Packet>(m_virtualPayloadSize));
		data->SetName(dataName);
		m_cs->Add(data);
	}
	//Simulator::Schedule (Seconds (50), & TrNavigationRouteHeuristic::fibnum, this);

}

void TrNavigationRouteHeuristic::fibnum()
{
	//if(int(Simulator::Now().GetSeconds() )% 10 == 0 )
	{
		if(m_fib->getFIB().size() == 3)
			ndn::nrndn::nrUtils::SetFullFibNum(m_node->GetId(),3);
		else if(m_fib->getFIB().size() == 2)
			ndn::nrndn::nrUtils::SetFullFibNum(m_node->GetId(),2);
		else if(m_fib->getFIB().size() == 1)
			ndn::nrndn::nrUtils::SetFullFibNum(m_node->GetId(),1);
		else if(m_fib->getFIB().size() == 0)
			ndn::nrndn::nrUtils::SetFullFibNum(m_node->GetId(),0);
	}
	Simulator::Schedule (Seconds (2), & TrNavigationRouteHeuristic::fibnum, this);
}

void TrNavigationRouteHeuristic::Stop()
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

void TrNavigationRouteHeuristic::WillSatisfyPendingInterest(
		Ptr<Face> inFace, Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

bool TrNavigationRouteHeuristic::DoPropagateInterest(
		Ptr<Face> inFace, Ptr<const Interest> interest,
		Ptr<pit::Entry> pitEntry)
{
	  NS_LOG_FUNCTION (this);
	  NS_LOG_UNCOND(this <<" is in unused function");
	  NS_ASSERT_MSG (m_pit != 0, "PIT should be aggregated with forwarding strategy");

	  return true;
}

void TrNavigationRouteHeuristic::WillEraseTimedOutPendingInterest(
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void TrNavigationRouteHeuristic::AddFace(Ptr<Face> face)
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

void TrNavigationRouteHeuristic::RemoveFace(Ptr<Face> face)
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

void TrNavigationRouteHeuristic::DidReceiveValidNack(
		Ptr<Face> incomingFace, uint32_t nackCode, Ptr<const Interest> nack,
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void TrNavigationRouteHeuristic::OnInterest(Ptr<Face> face,
		Ptr<Interest> interest)
{
	//NS_LOG_UNCOND("Here is NavigationRouteHeuristic dealing with OnInterest");
	//NS_LOG_FUNCTION (this);
	if(!m_running) return;
	//cout<<"into on interest"<<endl;
	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Get interest packet from APPLICATION");
		PrepareInterestPacket(interest);
		return;
	}

	if(HELLO_PACKET  == interest->GetScope())
	{
		ProcessHello(interest);//处理hello包
		return;
	}

	//If the interest packet has already been sent, do not proceed the packet
	if(m_interestNonceSeen.Get(interest->GetNonce()))//重复包（已发送或者已丢弃），不做处理
	{
		NS_LOG_DEBUG("The interest packet has already been sent, do not proceed the packet of "<<interest->GetNonce());
		return;
	}

	Ptr<const Packet> nrPayload	= interest->GetPayload();
	ndn::nrndn::trndnHeader nrheader;
	nrPayload->PeekHeader(nrheader);

	double x = nrheader.getX();
	double y = nrheader.getY();
	uint32_t nodeId=nrheader.getSourceId();
	uint32_t nextid = nrheader.getNextId();
	uint32_t currentid = nrheader.getCurrentId();
	uint32_t seq = interest->GetNonce();

	double disX =m_sensor->getX() - x;
	double disY =m_sensor->getY() - y;
	double distance = sqrt(disX *disX + disY * disY);
	double interval = (600 - distance) *1.5;

	if(INTEREST_PACKET == interest->GetScope())
	{
		if(!isDuplicatedInterest(nodeId,seq) )
		{
			if(m_cs->Find(interest->GetName()) /*&&( isSameLane(m_sensor->getLane(),currentLane) || isSameLane(m_sensor->getLane(),preLane))*/)
			{
				Time sendInterval = MilliSeconds(distance);
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval*2,
								&TrNavigationRouteHeuristic::ReplyDataPacket, this,interest);//回复的数据包，设置为此探测包的nonce和nodeid
				return;
			}
			else if(m_node->GetId() == nextid)
			{
				 if(m_pit->Find(interest->GetName()) )
				 {
					 m_pit->UpdatePit(currentid, interest);
					 m_interestNonceSeen.Put(interest->GetNonce(),true);
					 return;
				 }
				 else if(m_fib->Find(interest->GetName()))
				 {
					 m_pit->UpdatePit(currentid, interest);
					 Time sendInterval = (MilliSeconds(interval)+  m_gap* m_timeSlot);
					 m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
					 						&TrNavigationRouteHeuristic::ForwardInterestPacket, this,interest);
				 }
				return;
			}
		}//非重复包
		else//重复包
		{
			m_interestNonceSeen.Put(interest->GetNonce(),true);
			ExpireInterestPacketTimer(nodeId,seq);
			return;
		}
	}

}

void TrNavigationRouteHeuristic::OnData(Ptr<Face> face, Ptr<Data> data)
{
	NS_LOG_FUNCTION (this);
	if(!m_running) return;
	//cout<<"into on data"<<endl;
	if(Face::APPLICATION & face->GetFlags())
	{
		NS_LOG_DEBUG("Get data packet from APPLICATION");
		//cout<<"node: "<<m_node->GetId()<<" receive data in forwarder"<<endl;
		// 2. record the Data Packet(only record the forwarded packet)
		m_dataSignatureSeen.Put(data->GetSignature(),true);
		// 3. Then forward the data packet directly
		Simulator::Schedule(
				MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100)),
				&TrNavigationRouteHeuristic::SendDataPacket, this, data);

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

	Ptr<Packet> nrPayload	= data->GetPayload()->Copy();
	ndn::nrndn::trndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload	->PeekPacketTag(hopCountTag);
	ndn::nrndn::PacketTypeTag packetTypeTag;
	nrPayload	->PeekPacketTag(packetTypeTag);
	double x = nrheader.getX();
	double y = nrheader.getY();
	uint32_t nodeId=nrheader.getSourceId();
	uint32_t nextid = nrheader.getNextId();
	uint32_t currentid = nrheader.getCurrentId();
	std::vector<uint32_t> idlist = nrheader.getIdList();

	uint32_t signature=data->GetSignature();

	double disX =m_sensor->getX() - x;
	double disY =m_sensor->getY() - y;
	double distance =sqrt( disX * disX + disY * disY);
	double interval = (600 - distance) * 1.5 ;

	if(RESOURCE_PACKET == packetTypeTag.Get())
	{
		if(resourceReceived.find(signature) !=resourceReceived.end() )
		{//避免一个节点同时收到很多资源包，转发多次
			return;
		}
		resourceReceived.insert(signature);

		Time sendInterval = (MilliSeconds(interval) * 2);
		if( !m_fib->Find(data->GetName()))
		{
				m_fib-> AddFibEntry(data->GetNamePtr(),currentid, hopCountTag.Get() );
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval, &TrNavigationRouteHeuristic::ForwardResourcePacket, this,data);
		}
		return;
	}//end if (RESOURCE_PACKET == packetTypeTag.Get())
	else if (DATA_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			m_cs->Add(data);
			//m_pit->RemovePitEntry(data->GetName());
			NotifyUpperLayer(data);
			if(isDuplicatedInterest(nodeId,signature))
			{
					ExpireInterestPacketTimer(nodeId,signature);
			}
			if(m_pit->Find(data->GetName()) && inIdList(idlist, m_node->GetId()))
			{
				Time sendInterval = MilliSeconds(interval) ;
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&TrNavigationRouteHeuristic::ForwardDataPacket, this,data);
			}
		}
		else//重复包
		{
			ExpireDataPacketTimer(nodeId,signature);
			m_dataSignatureSeen.Put(data->GetSignature(),true);
			m_pit->RemovePitEntry(data->GetName());
			return;
		}
	}

	return;
}

bool TrNavigationRouteHeuristic::inIdList(std::vector<uint32_t> idlist, uint32_t id)
{
	for(uint32_t i = 0; i < idlist.size(); ++i)
		if(id == idlist[i])
			return true;
	return false;
}

bool TrNavigationRouteHeuristic::isDuplicatedInterest(
		uint32_t id, uint32_t nonce)
{
	NS_LOG_FUNCTION (this);
	//cout<<"into is duplicated interest"<<endl;
	if(!m_sendingInterestEvent.count(id))
		return false;
	else
		return m_sendingInterestEvent[id].count(nonce);
}

bool TrNavigationRouteHeuristic::isDuplicatedData(uint32_t id, uint32_t signature)
{
	NS_LOG_FUNCTION (this);
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::isDuplicatedData");
	if(!m_sendingDataEvent.count(id))
		return false;
	else
		return m_sendingDataEvent[id].count(signature);
}

void TrNavigationRouteHeuristic::ExpireInterestPacketTimer(uint32_t nodeId,uint32_t seq)
{
	//cout<<"into expire interest packet timer"<<endl;
	NS_LOG_FUNCTION (this<< "ExpireInterestPacketTimer id"<<nodeId<<"sequence"<<seq);
	//1. Find the waiting timer
	EventId& eventid = m_sendingInterestEvent[nodeId][seq];
	
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void TrNavigationRouteHeuristic::ExpireDataPacketTimer(uint32_t nodeId,uint32_t signature)
{
	//cout<<"into expire data packet timery"<<endl;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::ExpireDataPacketTimer");
	NS_LOG_FUNCTION (this<< "ExpireDataPacketTimer id\t"<<nodeId<<"\tsignature:"<<signature);
	//1. Find the waiting timer
	EventId& eventid = m_sendingDataEvent[nodeId][signature];
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void TrNavigationRouteHeuristic::ForwardResourcePacket(Ptr<Data> src)
{
	if(!m_running) return;
	//cout<<"into forward resource packet"<<endl;
	m_dataSignatureSeen.Put(src->GetSignature(),true);
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	//Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::trndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();

	// 	2.1 setup nrheader, source id do not change
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setCurrentId(m_node->GetId());
	nrPayload->AddHeader(nrheader);

	// 	2.2 setup FwHopCountTag
	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	if(hopCountTag.Get() > 14)
	{
		m_dataSignatureSeen.Put(src->GetSignature(),true);
		return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	// 	2.3 copy the data packet, and install new payload to data
	Ptr<Data> data = Create<Data> (*src);
	data->SetPayload(nrPayload);

	cout<<"node: "<<m_node->GetId()<<" forward resource packet from "<<nrheader.getSourceId()<<" name:"<<data->GetName().toUri ()<<" ttl:"<<(hopCountTag.Get()+1)<<endl;
	SendDataPacket(data);

	ndn::nrndn::nrUtils::IncreaseResourceForwardCounter();
	cout<<"now ResourceForwardSum = "<<ndn::nrndn::nrUtils::ResourceForwardSum<<endl;
}

void TrNavigationRouteHeuristic::ForwardDataPacket(Ptr<Data> src)
{
		if(!m_running) return;
		//cout<<"forward data packet"<<endl;
		Ptr<Packet> nrPayload=src->GetPayload()->Copy();
		ndn::nrndn::trndnHeader nrheader;
		nrPayload->RemoveHeader(nrheader);
		double x= m_sensor->getX();
		double y= m_sensor->getY();
		vector<uint32_t> idlist;
		if(m_pit->Find( src->GetName()) == 0){
			cout<<"pit not find"<<endl;
			return ;
		}
		Ptr<ndn::pit::nrndn::TrEntryNrImpl> nexthop;
		nexthop = DynamicCast<ndn::pit::nrndn::TrEntryNrImpl>(m_pit->Find( src->GetName()));
		std::unordered_set<uint32_t >::const_iterator it;
		for(it =nexthop->getIncomingnbs().begin(); it != nexthop->getIncomingnbs().end(); ++it)
		{
					idlist.push_back(*it);
		}
		// 	2.1 setup nrheader, source id do not change
		nrheader.setX(x);
		nrheader.setY(y);
		nrheader.setIdList(idlist);
		nrheader.setCurrentId(m_node->GetId());
		nrPayload->AddHeader(nrheader);

		// 	2.2 setup FwHopCountTag
		FwHopCountTag hopCountTag;
		nrPayload->RemovePacketTag( hopCountTag);
		if(hopCountTag.Get() > 14)
		{
			m_dataSignatureSeen.Put(src->GetSignature(),true);
			return;
		}
		nrPayload->AddPacketTag(hopCountTag);

		// 	2.3 copy the data packet, and install new payload to data
		Ptr<Data> data = Create<Data> (*src);
		data->SetPayload(nrPayload);

		m_dataSignatureSeen.Put(src->GetSignature(),true);
		cout<<"node: "<<m_node->GetId()<<" forward data packet to "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get()<<" current id:"<<m_node->GetId();

		m_pit->RemovePitEntry(data->GetName());
		SendDataPacket(data);

		ndn::nrndn::nrUtils::IncreaseDataForwardCounter();
}

void TrNavigationRouteHeuristic::ForwardInterestPacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"node: "<<m_node->GetId()<<" into forward interest packet"<<endl;
	m_interestNonceSeen.Put(src->GetNonce(),true);
	// 2. prepare the interest
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	ndn::nrndn::trndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	nrheader.setCurrentId(m_node->GetId());

	if(m_fib->Find( src->GetName()) == 0)
	{
		return;
	}
	Ptr<ndn::fib::nrndn::TrEntryNrImpl> nexthop = DynamicCast<ndn::fib::nrndn::TrEntryNrImpl>(m_fib->Find( src->GetName()));
	nrheader.setNextId(nexthop->getIncomingnbs().begin()->first);
	nrPayload->AddHeader(nrheader);
	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	if(hopCountTag.Get() > 14)
	{
			m_interestNonceSeen.Put(src->GetNonce(),true);
			return;
	}
	nrPayload->AddPacketTag(hopCountTag);
	Ptr<Interest> interest = Create<Interest> (*src);
	interest->SetPayload(nrPayload);
	// 3. Send the interest Packet. Already wait, so no schedule

	cout<<"node: "<<m_node->GetId()<<" forward interest packet from "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get()<<" current id:"<<m_node->GetId()<<endl;
	SendInterestPacket(interest);
	ndn::nrndn::nrUtils:: IncreaseInterestForwardCounter();
	cout<<"Now InterestForwardSum = "<<ndn::nrndn::nrUtils::InterestForwardSum<<endl;
}


void TrNavigationRouteHeuristic::ReplyDataPacket(Ptr<Interest> interest)
{
	if (!m_running)  return;
	//cout<<"into reply data packet"<<endl;
	Ptr<Data> data = Create<Data>(Create<Packet>(m_virtualPayloadSize));
	Ptr<Name> dataName = Create<Name>(interest->GetName());
	data->SetName(dataName);
	data->SetFreshness(m_freshness);
	data->SetTimestamp(Simulator::Now());
	data->SetSignature(interest->GetNonce());

	ndn::nrndn::trndnHeader nrheader;
	interest->GetPayload()->PeekHeader(nrheader);
	uint32_t nextid = nrheader.getNextId();
	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	nrheader.setCurrentId(m_node->GetId());
	nrheader.setNextId(nextid);
	//sourceId not change????????????????????????!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	Ptr<Packet> newPayload	= Create<Packet> (m_virtualPayloadSize);
	newPayload->AddHeader(nrheader);
	//cout<<"header size"<<nrheader.GetSerializedSize()<<endl;
	data->SetPayload(newPayload);

	ndn::nrndn::PacketTypeTag typeTag(DATA_PACKET );
	data->GetPayload()->AddPacketTag(typeTag);

	FwHopCountTag hopCountTag;
	data->GetPayload()->AddPacketTag(hopCountTag);

	m_dataSignatureSeen.Put(data->GetSignature(),true);

	cout<<"node: "<<m_node->GetId()<<" reply data packet to "<<nrheader.getSourceId()<<endl;
	ndn::nrndn::nrUtils::IncreaseDataNum();
	SendDataPacket(data);
}

void TrNavigationRouteHeuristic::PrepareInterestPacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into prepare interest packet"<<endl;
	// 2. prepare the interest
	interest->SetScope(INTEREST_PACKET);
	Ptr<Packet> nrPayload= interest->GetPayload()->Copy();
	ndn::nrndn::trndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	if(!m_fib->Find(interest->GetName()))
		Simulator::Schedule (Seconds (3), &TrNavigationRouteHeuristic::PrepareInterestPacket, this,interest);

	Ptr<ndn::fib::nrndn::TrEntryNrImpl> nexthop;
	nexthop = DynamicCast<ndn::fib::nrndn::TrEntryNrImpl>(m_fib->Find(interest->GetName()));
	uint32_t nextid = (nexthop->getIncomingnbs()).begin()->first	;
	nrheader.setCurrentId(m_node->GetId());
	nrheader.setNextId(nextid);
	nrPayload->AddHeader(nrheader);
	nrPayload->RemoveAllPacketTags();
	FwHopCountTag hopCountTag;
	nrPayload->AddPacketTag(hopCountTag);

	ndn::nrndn::PacketTypeTag typeTag(INTEREST_PACKET);
	nrPayload->AddPacketTag (typeTag);

	interest->SetPayload(nrPayload);
	Simulator::Schedule(
					MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100)),
					&TrNavigationRouteHeuristic::SendInterestPacket, this, interest);
	//interest->SetNonce(m_rand.GetValue());
	//SendInterestPacket(interest);
	cout<<"node: "<<m_node->GetId()<<"  send interest packet,name: "<<interest->GetName().toUri()<<" scope:"<<(int)(interest->GetScope())<<endl;
//getchar();
	m_interestNonceSeen.Put(interest->GetNonce(),true);
	ndn::nrndn::nrUtils::IncreaseInterestNum();

}

void TrNavigationRouteHeuristic::SendInterestPacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	//cout<<"into send interest packet"<<endl;
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

void TrNavigationRouteHeuristic::SendDataPacket(Ptr<Data> data)
{
	if(!m_running) return;
	//cout<<"into send data packet"<<endl;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::SendDataPacket");
	vector<Ptr<Face> >::iterator fit;
	for (fit = m_outFaceList.begin(); fit != m_outFaceList.end(); ++fit)
	{
		(*fit)->SendData(data);
		//////////ndn::nrndn::nrUtils::AggrateDataPacketSize(data);
	}
}

void TrNavigationRouteHeuristic::SendHello()
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
	ndn::nrndn::nrHeader nrheader;
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

void TrNavigationRouteHeuristic::DoInitialize(void)
{
	//cout<<"doinital"<<endl;
	if (m_sensor == 0)
	{
		m_sensor = m_node->GetObject<ndn::nrndn::NodeSensor>();
		NS_ASSERT_MSG(m_sensor,"TrNavigationRouteHeuristic::DoInitialize cannot find ns3::ndn::nrndn::NodeSensor");
	}
	super::DoInitialize();
}

void TrNavigationRouteHeuristic::DropPacket()
{
	NS_LOG_DEBUG ("Drop Packet");
}

void TrNavigationRouteHeuristic::DropDataPacket(Ptr<Data> data)
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

void TrNavigationRouteHeuristic::DropInterestePacket(Ptr<Interest> interest)
{
	NS_LOG_DEBUG ("Drop interest Packet");
	DropPacket();
}

void TrNavigationRouteHeuristic::NotifyNewAggregate()
{

  if (m_sensor == 0)
  {
	  m_sensor = GetObject<ndn::nrndn::NodeSensor> ();
   }

  if ( m_pit == 0)
  {
	  Ptr<Pit> pit=GetObject<Pit>();
	  if(pit)
		  m_pit = DynamicCast<pit::nrndn::TrNrPitImpl>(pit);
  }

  if (m_fib == 0)
   {
 	  Ptr<Fib> fib=GetObject<Fib>();
 	  if(fib)
 		 m_fib = DynamicCast<fib::nrndn::TrNrFibImpl>(fib);
   }

  if (m_cs == 0)
  {
   	  Ptr<ContentStore> cs=GetObject<ContentStore>();
   	  if(cs)
   		  m_cs = DynamicCast<cs::nrndn::NrCsImpl>(cs);
   }

  if(m_node==0)
  {
	  m_node=GetObject<Node>();
  }

  super::NotifyNewAggregate ();
}

void
TrNavigationRouteHeuristic::HelloTimerExpire ()
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
TrNavigationRouteHeuristic::FindBreaksLinkToNextHop(uint32_t BreakLinkNodeId)
{
	 NS_LOG_FUNCTION (this);
}

void
TrNavigationRouteHeuristic::ProcessHello(Ptr<Interest> interest)
{
	if(!m_running) return;

	if(m_HelloLogEnable)
		NS_LOG_DEBUG (this << interest << "\tReceived HELLO packet from "<<interest->GetNonce());

	Ptr<const Packet> nrPayload	= interest->GetPayload();
	ndn::nrndn::nrHeader nrheader;
	nrPayload->PeekHeader(nrheader);
	//update neighbor list
	m_nb.Update(nrheader.getSourceId(),nrheader.getX(),nrheader.getY(),Time (AllowedHelloLoss * HelloInterval));
}

void TrNavigationRouteHeuristic::ToContentStore(Ptr<Data> data)
{
	NS_LOG_DEBUG ("To content store.(Just a trace)");
	return;
}

void TrNavigationRouteHeuristic::NotifyUpperLayer(Ptr<Data> data)
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


