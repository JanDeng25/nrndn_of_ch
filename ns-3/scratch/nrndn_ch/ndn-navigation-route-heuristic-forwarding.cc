/*
 * ndn-navigation-route-heuristic-forwarding.cc
 *
 *  Created on: Jan 14, 2015
 *      Author: chen
 */

#include "ndn-navigation-route-heuristic-forwarding.h"
#include "NodeSensor.h"
#include "nrHeader.h"
#include "tableHeader.h"
#include "nrndn-Header.h"
#include "nrndn-Neighbors.h"
#include "ndn-pit-entry-nrimpl.h"
#include "ndn-fib-entry-nrimpl.h"
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
	    .AddAttribute ("Pit","pit of forwarder",
	    		  PointerValue (),
		    	  MakePointerAccessor (&NavigationRouteHeuristic::m_pit),
		    	  MakePointerChecker<ns3::ndn::pit::nrndn::NrPitImpl> ())
	    .AddAttribute ("Fib","fib of forwarder",
	    	  PointerValue (),
		    	  MakePointerAccessor (&NavigationRouteHeuristic::m_fib),
		    	  MakePointerChecker<ns3::ndn::fib::nrndn::NrFibImpl> ())
			.AddAttribute ("CS","cs of forwarder",
	    		  PointerValue (),
		    	  MakePointerAccessor (&NavigationRouteHeuristic::m_cs),
 		     MakePointerChecker<ns3::ndn::cs::nrndn::NrCsImpl> ())
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
		m_cs->Add(data);
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
	//cout<<"into on interest"<<endl;
	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Get interest packet from APPLICATION");
		//cout<<"node: "<<m_node->GetId()<<" receive interest in forwarder"<<endl;
		//m_fib->Print(cout);
		//cout<<"lane:"<<m_sensor->getLane()<<endl;
		if(interest->GetScope() == MOVE_TO_NEW_LANE)
		{
			PrepareMoveToNewLanePacket(interest);//发送MOVE_TO_NEW_LANE包
			return;
		}
		PreparePacket(interest);//发送兴趣包或者探测包
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
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->PeekHeader(nrheader);

	double x = nrheader.getX();
	double y = nrheader.getY();
	uint32_t nodeId=nrheader.getSourceId();
	uint32_t seq = interest->GetNonce();
	std::string currentLane = nrheader.getCurrentLane();
	std::string preLane = nrheader.getPreLane();
	std::vector<std::string> laneList = nrheader.getLaneList();

	double disX =m_sensor->getX() - x;
	double disY =m_sensor->getY() - y;
	double distance = sqrt(disX *disX + disY * disY);
	double interval = (600 - distance) * 3;

	if(DETECT_PACKET == interest->GetScope())
	{
		if(!isDuplicatedInterest(nodeId,seq) && !isJuction(m_sensor->getLane()))//第一次收到此包
		{
			if(m_fib->Find(interest->GetName()) || m_cs->Find(interest->GetName()))
			{
				Time sendInterval = MilliSeconds(distance);
				//cout<<"detect packet  send interval: "<<sendInterval.GetSeconds()<<endl;
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ReplyConfirmPacket, this,interest);//回复的确认包，设置为此探测包的nonce和nodeid
				return;
			}
			else
			{
				Time sendInterval = (MilliSeconds(interval) +  m_gap * m_timeSlot);
				//cout<<"detect packet   send interval: "<<sendInterval.GetSeconds()<<endl;
				m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ForwardDetectPacket, this,interest);
				return;
			}
		}
		else//重复，准备发送时收到了其他节点转发的同样的包，则取消转发，同时m_interestNonceSeen.Put(interest->GetNonce(),true)，以后不再处理
		{
			m_interestNonceSeen.Put(interest->GetNonce(),true);
			ExpireInterestPacketTimer(nodeId,seq);
			return;
		}
	}
	else if(INTEREST_PACKET == interest->GetScope())
	{
		if(!isDuplicatedInterest(nodeId,seq) )
		{
			cout<<"node: "<<m_node->GetId()<<" receive interest packet from "<<nodeId<<endl;
			if(!isSameLane(m_sensor->getLane(),currentLane)&& !isSameLane(m_sensor->getLane(),preLane))
			{//不在应接受的下一跳路段，也不在与interest包同路段
				//cout<<"not on the section of interest packet, my section:"<<m_sensor->getLane()<<" current lane:"<<currentLane<<" pre lane"<<preLane<<endl;
				m_interestNonceSeen.Put(interest->GetNonce(),true);//不做处理
				return;
			}
			else if(m_cs->Find(interest->GetName()))
			{
				//cout<<"find cs"<<endl;
				Time sendInterval = MilliSeconds(distance);
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ReplyDataPacket, this,interest);//回复的数据包，设置为此探测包的nonce和nodeid
				return;
			}
			else if(m_pit->Find(interest->GetName()) && !isJuction(m_sensor->getLane()))
			{
				cout<<"pit find"<<endl;
				//有pit记录，更新pit，不转发
				m_pit->UpdatePit(preLane, interest);
				m_interestNonceSeen.Put(interest->GetNonce(),true);
				return;
			}
			else if(!isJuction(m_sensor->getLane()))//在路口的车辆不参与转发
			{
				if(m_fib->Find(interest->GetName()))
				{
					m_pit->UpdatePit(preLane, interest);
					if(isSameLane(m_sensor->getLane(),currentLane))//处在兴趣包应走的下一跳路段的车辆优先转发
					{
						Time sendInterval = (MilliSeconds(interval));
						m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
												&NavigationRouteHeuristic::ForwardInterestPacket, this,interest);
					}
					else//若没有下一跳路段车辆转发，则与兴趣包同路段的车辆转发
					{
						Time sendInterval = (MilliSeconds(interval) +  m_gap* m_timeSlot);
						m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
												&NavigationRouteHeuristic::ForwardInterestPacket, this,interest);
					}
					return;
				}
				//cout<<"not find fib"<<endl;
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
		if(!isDuplicatedInterest(nodeId,seq) && isSameLane(m_sensor->getLane(),preLane))
		{
			//cout<<"node: "<<m_node->GetId()<<" receive MOVE_TO_NEW_LANE packet from "<<nodeId<<" new lane: "<<currentLane<<" NONCE:"<<interest->GetNonce()<<endl;
			m_pit->UpdatePit(currentLane, interest);
			Time sendInterval = MilliSeconds(distance*1.5);
			m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval ,
												&NavigationRouteHeuristic::ForwardMoveToNewLanePacket, this,interest);

			return;
		}
		else
		{
			m_interestNonceSeen.Put(interest->GetNonce(),true);
			ExpireInterestPacketTimer(nodeId,seq);
			return;
		}
	}
	else if(ASK_FOR_TABLE == interest->GetScope())
	{
		if(isSameLane(m_sensor->getLane(), currentLane) && !m_fib->getFIB().empty())
		{
			//cout<<"node: "<<m_node->GetId()<<" receive ASK_FOR_TABLE packet from "<<nodeId<<endl;
			Time sendInterval = MilliSeconds(distance *1.5);
			Time delay = MilliSeconds(1000);

			if(m_fib->getFIB().size() == 3)//表格完善的车辆优先回复
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval ,
												&NavigationRouteHeuristic::ReplyTablePacket, this,interest);
			else if(m_fib->getFIB().size() == 2)
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval+delay ,
												&NavigationRouteHeuristic::ReplyTablePacket, this,interest);
			else
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval+delay*2 ,
												&NavigationRouteHeuristic::ReplyTablePacket, this,interest);
			return;
		}
	}
}

void NavigationRouteHeuristic::OnData(Ptr<Face> face, Ptr<Data> data)
{
	NS_LOG_FUNCTION (this);
	if(!m_running) return;
	//cout<<"into on data"<<endl;
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

	Name name;
	name.appendNumber(ASK_FOR_TABLE);
	 if(name == data->GetName())//收到其他节点回复的table packet（非请求table的表格）
	{
		ns3::ndn::nrndn::tableHeader tableheader;
		Ptr<Packet> nrPayload=data->GetPayload()->Copy();
		nrPayload->PeekHeader(tableheader);
		std::vector<Ptr<pit::Entry> > pit = tableheader.getPitContainer();
		std::vector<Ptr<fib::Entry> > fib = tableheader.getFibContainer();
		m_pit->mergePit(pit);
		m_fib->mergeFib(fib);
		if(isDuplicatedData(tableheader.getSourceId(), tableheader.getSignature()))
		{
			ExpireDataPacketTimer(tableheader.getSourceId(), tableheader.getSignature());
			m_dataSignatureSeen.Put(data->GetSignature(),true);
		}
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

	double disX =m_sensor->getX() - x;
	double disY =m_sensor->getY() - y;
	double distance =sqrt( disX * disX + disY * disY);
	double interval = (600 - distance) * 1.5 ;

	if(RESOURCE_PACKET == packetTypeTag.Get())
	{
		if(isJuction(m_sensor->getLane() )|| (isSameLane(m_sensor->getLane(), currentLane) && isDuplicatedData(nodeId,signature)))
		{//若处在路口，或者收到同路段车辆转发的包
			ExpireDataPacketTimer(nodeId,signature);
			m_dataSignatureSeen.Put(data->GetSignature(),true);
			resourceReceived.insert(signature);
			return;
		}
		if(resourceReceived.find(signature) !=resourceReceived.end() )
		{//避免一个节点同时收到很多资源包，转发多次
			return;
		}
		/*
		for(uint32_t i = 0; i<laneList.size(); ++i)
		{
				if(isSameLane(laneList[i], m_sensor->getLane()))
				{
					ExpireDataPacketTimer(nodeId,signature);
					m_dataSignatureSeen.Put(data->GetSignature(),true);
					resourceReceived.insert(signature);
					return;
				}
		}
		*/
		resourceReceived.insert(signature);
		if(isSameLane(m_sensor->getLane(),currentLane) && !m_fib->Find(data->GetName()))
				m_fib-> AddFibEntry(data->GetNamePtr(),preLane, hopCountTag.Get() );
		else  if(!m_fib->Find(data->GetName()))
				m_fib-> AddFibEntry(data->GetNamePtr(),currentLane, hopCountTag.Get() );

		Time sendInterval = (MilliSeconds(interval) );
		m_sendingDataEvent[nodeId][signature]=
							Simulator::Schedule(sendInterval, &NavigationRouteHeuristic::ForwardResourcePacket, this,data);

		return;
	}//end if (RESOURCE_PACKET == packetTypeTag.Get())
	else if (DATA_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			cout<<"node: "<<m_node->GetId()<<" receive data packet from "<<nodeId<<endl;
			m_cs->Add(data);
			m_pit->RemovePitEntry(data->GetName());
			NotifyUpperLayer(data);
			if(isDuplicatedInterest(nodeId,signature))
			{
					ExpireInterestPacketTimer(nodeId,signature);
			}
			if(m_pit->Find(data->GetName()))
			{
				if(OnTheWay(laneList))//在数据包应走的下一跳，可能有多个
				{
					Time sendInterval = MilliSeconds(interval) ;
					m_sendingDataEvent[nodeId][signature]=
									Simulator::Schedule(sendInterval,
									&NavigationRouteHeuristic::ForwardDataPacket, this,data);
					return;
				}
				else if(isSameLane(m_sensor->getLane(),preLane))//与数据包同路段
				{
					m_pit->RemovePitEntry(data->GetName());
					Time sendInterval = (MilliSeconds(interval) + m_gap* m_timeSlot);
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
			//NotifyUpperLayer(data);
			//cout<<"node: "<<m_node->GetId()<<" receive confirm packet from "<<nodeId<<endl;
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
			 m_fib-> AddFibEntry(data->GetNamePtr(),preLane, nrheader.getTTL());
			if(isSameLane(m_sensor->getLane(),currentLane))
			{
				Time sendInterval = (MilliSeconds(interval) + m_gap * m_timeSlot);
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&NavigationRouteHeuristic::ForwardConfirmPacket, this,data);
				return;
			}
			else if(isSameLane(m_sensor->getLane(),preLane))
			{
				Time sendInterval = (MilliSeconds(interval) + (m_gap+5) * m_timeSlot);
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&NavigationRouteHeuristic::ForwardConfirmPacket, this,data);
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
	return;
}

bool NavigationRouteHeuristic::OnTheWay(std::vector<std::string> laneList)
{
	//cout<<"into on the way"<<endl;
	for(uint32_t i = 0; i < laneList.size(); ++i)
		if(isSameLane(m_sensor->getLane(),laneList[i]))
			return true;
	return false;
}

bool NavigationRouteHeuristic::isDuplicatedInterest(
		uint32_t id, uint32_t nonce)
{
	NS_LOG_FUNCTION (this);
	//cout<<"into is duplicated interest"<<endl;
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
	//cout<<"into expire interest packet timer"<<endl;
	NS_LOG_FUNCTION (this<< "ExpireInterestPacketTimer id"<<nodeId<<"sequence"<<seq);
	//1. Find the waiting timer
	EventId& eventid = m_sendingInterestEvent[nodeId][seq];
	
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void NavigationRouteHeuristic::ExpireDataPacketTimer(uint32_t nodeId,uint32_t signature)
{
	//cout<<"into expire data packet timery"<<endl;
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
	//cout<<"into forward resource packet"<<endl;
	m_dataSignatureSeen.Put(src->GetSignature(),true);
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	//Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	std::string currentlane = m_sensor->getLane();
	std::string prelane;
	if(isSameLane(currentlane, nrheader.getCurrentLane() )  )
		prelane = nrheader.getPreLane();
	else
		prelane = nrheader.getCurrentLane();
	vector<string> lanelist = nrheader.getLaneList();
	bool find = false;
	for(uint32_t i = 0; i<lanelist.size(); ++i)
		if(lanelist[i] == currentlane)
		{
			find = true;
			break;
		}
	if(!find)
		lanelist.push_back(currentlane);

	// 	2.1 setup nrheader, source id do not change
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setCurrentLane(currentlane);
	nrheader.setPreLane(prelane);
	nrheader.setLaneList(lanelist);
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

	//cout<<"node: "<<m_node->GetId()<<" forward resource packet from "<<nrheader.getSourceId()<<" name:"<<data->GetName().toUri ()<<" lane:"<<nrheader.getCurrentLane()<<" ttl:"<<(hopCountTag.Get()+1)<<endl;
	SendDataPacket(data);

	ndn::nrndn::nrUtils::IncreaseResourceForwardCounter();
}

void NavigationRouteHeuristic::ForwardConfirmPacket(Ptr<Data> src)
{
	if(!m_running) return;
	//cout<<"into forward confirm packet"<<endl;
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	//Ptr<Packet> newPayload	= Create<Packet> ();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	std::vector<std::string> lanelist = nrheader.getLaneList();
	cout<<lanelist.size()<<endl;
	std::string currentlane = lanelist[0];
	for(uint32_t i = lanelist.size()-1; i >0; --i)
	{
		 if(isSameLane(m_sensor->getLane() ,  lanelist[i]))
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
	if(hopCountTag.Get() > 3)
	{
		m_dataSignatureSeen.Put(src->GetSignature(),true);
		return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	// 	2.3 copy the data packet, and install new payload to data
	Ptr<Data> data = Create<Data> (*src);
	data->SetPayload(nrPayload);

	m_dataSignatureSeen.Put(src->GetSignature(),true);
	cout<<"node: "<<m_node->GetId()<<" forward confirm packet to "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get() <<endl;
	SendDataPacket(data);

	ndn::nrndn::nrUtils::IncreaseConfirmForwardCounter();
}

void NavigationRouteHeuristic::ForwardDataPacket(Ptr<Data> src)
{
		if(!m_running) return;
		//cout<<"into forward data packet"<<endl;
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
		cout<<"node: "<<m_node->GetId()<<" forward data packet from "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get()<<endl;

		SendDataPacket(data);

		ndn::nrndn::nrUtils::IncreaseDataForwardCounter();
}

void NavigationRouteHeuristic::ForwardDetectPacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into forward detect packet"<<endl;
	// 2. prepare the interest
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	std::vector<std::string> lanelist = nrheader.getLaneList();

	for(uint32_t i=0; i<lanelist.size(); ++i)
		if(isSameLane(m_sensor->getLane() ,  lanelist[i]))
			return;
	lanelist.push_back(m_sensor->getLane() );
	double x= m_sensor->getX();
	double y= m_sensor->getY();
	nrheader.setX(x);
	nrheader.setY(y);
	nrheader.setLaneList(lanelist);
	nrPayload->AddHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload->RemovePacketTag( hopCountTag);
	if(hopCountTag.Get() > 3)
	{
			m_interestNonceSeen.Put(src->GetNonce(),true);
			return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	Ptr<Interest> interest = Create<Interest> (*src);
	interest->SetPayload(nrPayload);

	// 3. Send the interest Packet. Already wait, so no schedule
	m_interestNonceSeen.Put(src->GetNonce(),true);
	cout<<"node: "<<m_node->GetId()<<" forward detect packet from "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get()<<endl;
	SendInterestPacket(interest);
	ndn::nrndn::nrUtils::IncreaseDetectForwardCounter();
}

void NavigationRouteHeuristic::ForwardMoveToNewLanePacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into move to new lane packet"<<endl;
	m_interestNonceSeen.Put(src->GetNonce(),true);
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	cout<<"node: "<<m_node->GetId()<<" forward MOVE_TO_NEW_LANE packet from "<<nrheader.getSourceId()<<" new lane: "<<nrheader.getCurrentLane()<<" NONCE: "<<src->GetNonce()<<endl;

	nrPayload->AddHeader(nrheader);
	//Ptr<Interest> interest = Create<Interest> (*src);
	 src->SetPayload(nrPayload);
	 ndn::nrndn::nrUtils::IncreaseTableNum();
	SendInterestPacket( src);
}

void NavigationRouteHeuristic::ForwardInterestPacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into forward interest packet"<<endl;
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
	if(hopCountTag.Get() > 14)
	{
			m_interestNonceSeen.Put(src->GetNonce(),true);
			return;
	}
	nrPayload->AddPacketTag(hopCountTag);

	Ptr<Interest> interest = Create<Interest> (*src);
	interest->SetPayload(nrPayload);

	// 3. Send the interest Packet. Already wait, so no schedule
	m_interestNonceSeen.Put(src->GetNonce(),true);
	cout<<"node: "<<m_node->GetId()<<" forward interest packet from "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get()<<endl;
	SendInterestPacket(interest);

	ndn::nrndn::nrUtils:: IncreaseInterestForwardCounter();
}

void NavigationRouteHeuristic::ReplyConfirmPacket(Ptr<Interest> interest)
{
	if (!m_running)  return;
	//cout<<"into reply confirm packet"<<endl;
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
	for(uint32_t i = nrheader.getLaneList().size()-1; i>0; --i)
	{
		if(isSameLane(m_sensor->getLane() , nrheader.getLaneList()[i]) )
		{
			currentlane = nrheader.getLaneList()[i-1];
			break;
		}
	}
	nrheader.setCurrentLane(currentlane);
	nrheader.setPreLane(m_sensor->getLane());
	uint32_t ttl;
	if(m_cs->Find(interest->GetName()))
	{
		ttl = 0;
	}
	else if(m_fib->Find(interest->GetName()))
	{
		Ptr<ndn::fib::nrndn::EntryNrImpl> nexthop;
		nexthop = DynamicCast<ndn::fib::nrndn::EntryNrImpl>(m_fib->Find(interest->GetName()));
		ttl = (nexthop->getIncomingnbs()).begin()->second;
	}

	nrheader.setTTL(ttl);

	Ptr<Packet> newPayload	= Create<Packet> (m_virtualPayloadSize);
	newPayload->AddHeader(nrheader);

	data->SetPayload(newPayload);

	ndn::nrndn::PacketTypeTag typeTag(CONFIRM_PACKET );
	data->GetPayload()->AddPacketTag(typeTag);

	FwHopCountTag hopCountTag;
	data->GetPayload()->AddPacketTag(hopCountTag);

	m_dataSignatureSeen.Put(data->GetSignature(),true);
	//cout<<"node: "<<m_node->GetId()<<" reply confirm packet to "<<nrheader.getSourceId()<<endl;
	ndn::nrndn::nrUtils:: IncreaseConfirmNum();
	SendDataPacket(data);
}

void NavigationRouteHeuristic::ReplyTablePacket(Ptr<Interest> interest)
{
	//将表格打包，signature和nodeid与interest相同
	if (!m_running)  return;
	//cout<<"into reply table packet"<<endl;
	Ptr<Data> data = Create<Data>(Create<Packet>(m_virtualPayloadSize));
	Ptr<Name> dataName = Create<Name>(interest->GetName());
	data->SetName(dataName);
	data->SetFreshness(m_freshness);
	data->SetTimestamp(Simulator::Now());
	data->SetSignature(interest->GetNonce());

	ndn::nrndn::nrndnHeader nrheader;
	interest->GetPayload()->PeekHeader(nrheader);
	uint32_t sourceid = nrheader.getSourceId();

	ndn::nrndn::tableHeader tableheader;
	tableheader.setSourceId(sourceid);
	tableheader.setSignature(interest->GetNonce());
	if(!m_pit->getPIT().empty())
		tableheader.setPIT(m_pit->getPIT());
	tableheader.setFIB(m_fib->getFIB());

	Ptr<Packet> newPayload	= Create<Packet> (m_virtualPayloadSize);
	newPayload->AddHeader(tableheader);

	ndn::nrndn::PacketTypeTag typeTag(TABLE_PACKET );
	newPayload->AddPacketTag(typeTag);

	data->SetPayload(newPayload);

	m_dataSignatureSeen.Put(data->GetSignature(),true);
	//cout<<"node: "<<m_node->GetId()<<" reply table packet to "<<nrheader.getSourceId()<<endl;
	ndn::nrndn::nrUtils:: IncreaseTableNum();
	SendDataPacket(data);
}

void NavigationRouteHeuristic::ReplyDataPacket(Ptr<Interest> interest)
{
	if (!m_running)  return;
	//cout<<"into reply data packet"<<endl;
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
	//sourceId not change????????????????????????!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	Ptr<Packet> newPayload	= Create<Packet> (m_virtualPayloadSize);
	newPayload->AddHeader(nrheader);

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

void NavigationRouteHeuristic::PrepareInterestPacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into prepare interest packet"<<endl;
	// 2. prepare the interest
	Ptr<Packet> nrPayload= interest->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

	Ptr<ndn::fib::nrndn::EntryNrImpl> nexthop;
	nexthop = DynamicCast<ndn::fib::nrndn::EntryNrImpl>(m_fib->Find(interest->GetName()));
	std:: string hop = "";
	hop = (nexthop->getIncomingnbs()).begin()->first	;
	nrheader.setCurrentLane(hop);
	nrheader.setPreLane(m_sensor->getLane());
	nrPayload->AddHeader(nrheader);
	nrPayload->RemoveAllPacketTags();
	FwHopCountTag hopCountTag;
	nrPayload->AddPacketTag(hopCountTag);

	ndn::nrndn::PacketTypeTag typeTag(INTEREST_PACKET);
	nrPayload->AddPacketTag (typeTag);

	interest->SetPayload(nrPayload);
	interest->SetScope(INTEREST_PACKET);

	cout<<"node: "<<m_node->GetId()<<"  send interest packet,name: "<<interest->GetName().toUri()<<" in forwarder"<<endl;

	m_interestNonceSeen.Put(interest->GetNonce(),true);
	ndn::nrndn::nrUtils::IncreaseInterestNum();
	SendInterestPacket(interest);
}

void NavigationRouteHeuristic::PrepareMoveToNewLanePacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into prepare move to new lane packet"<<endl;
	// 2. prepare the interest
	Ptr<Packet> nrPayload= interest->GetPayload()->Copy();

	FwHopCountTag hopCountTag;
	nrPayload->AddPacketTag(hopCountTag);

	ndn::nrndn::PacketTypeTag typeTag(MOVE_TO_NEW_LANE);
	nrPayload->AddPacketTag (typeTag);

	interest->SetPayload(nrPayload);
	interest->SetScope(MOVE_TO_NEW_LANE);

	cout<<"node: "<<m_node->GetId()<<"  send MOVE_TO_NEW_LANE packet in forwarder"<<endl;

	m_interestNonceSeen.Put(interest->GetNonce(),true);
	ndn::nrndn::nrUtils::IncreaseTableNum();
	SendInterestPacket(interest);
}

void NavigationRouteHeuristic::PrepareDetectPacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into prepare detect packet"<<endl;
	Ptr<Packet> nrPayload= interest->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	vector<string> lanelist;
	lanelist.push_back(m_sensor->getLane());
	nrheader.setLaneList(lanelist);
	nrPayload->AddHeader(nrheader);

	FwHopCountTag hopCountTag;
	nrPayload->AddPacketTag(hopCountTag);

	ndn::nrndn::PacketTypeTag typeTag(DETECT_PACKET);
	nrPayload->AddPacketTag (typeTag);

	interest->SetPayload(nrPayload);
	interest->SetScope(DETECT_PACKET);

	cout<<"node: "<<m_node->GetId()<<"  send detect packet,name: "<<interest->GetName().toUri()<<" in forwarder"<<endl;

	m_interestNonceSeen.Put(interest->GetNonce(),true);
	SendInterestPacket(interest);

	Simulator::Schedule (Seconds (5.0), & NavigationRouteHeuristic::PrepareInterestPacket, this,interest);
	ndn::nrndn::nrUtils::IncreaseDetectNum();
}

void NavigationRouteHeuristic::PreparePacket(Ptr<Interest> interest)
{
	if(m_fib->getFIB().size() == 0)
		Simulator::Schedule (Seconds (5.0), & NavigationRouteHeuristic::PreparePacket, this,interest);
	else if (m_fib->Find(interest->GetName()))
				PrepareInterestPacket(interest);
	else
				PrepareDetectPacket(interest);
}

void NavigationRouteHeuristic::SendInterestPacket(Ptr<Interest> interest)
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

void NavigationRouteHeuristic::SendDataPacket(Ptr<Data> data)
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
	Ptr<Packet> newPayload	= Create<Packet> (m_virtualPayloadSize);
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

void NavigationRouteHeuristic::DoInitialize(void)
{
	//cout<<"doinital"<<endl;
	if (m_sensor == 0)
	{
		m_sensor = m_node->GetObject<ndn::nrndn::NodeSensor>();
		NS_ASSERT_MSG(m_sensor,"NavigationRouteHeuristic::DoInitialize cannot find ns3::ndn::nrndn::NodeSensor");
		if(m_sensor != NULL)
						m_sensor->TraceConnectWithoutContext ("LaneChange", MakeCallback (&NavigationRouteHeuristic::laneChange,this));
	}
	super::DoInitialize();
}

void NavigationRouteHeuristic::laneChange(std::string oldLane, std::string newLane)
{
	if (!m_running)  return;
	if(isJuction(newLane)) return;
	if(Simulator::Now().GetSeconds() <16) return;

	if(oldLane == m_oldLane) return;
	//cout<<m_node->GetId()<<" lane changed from "<<m_oldLane<<" to "<<newLane<<endl;
	//if(Simulator::Now().GetSeconds() > 60)
	//{
		//cout<<"node id:"<<m_node->GetId()<<" lane:"<<newLane<<" ";
		//m_fib->Print(cout);
	//}
	m_oldLane = oldLane;
	m_pit->cleanPIT();/////////////////////////////////////
	m_fib->cleanFIB();/////////////////////////////////////////////
	Name name;
	name.appendNumber(ASK_FOR_TABLE);
	Ptr<Interest> interest = Create<Interest> (Create<Packet>(m_virtualPayloadSize));
	Ptr<Name> interestName = Create<Name>(name);
	interest->SetName(interestName);
	interest->SetNonce(m_uniformRandomVariable->GetValue());//just generate a random number
	interest->SetScope(ASK_FOR_TABLE);

	ndn::nrndn::nrndnHeader nrheader;
	nrheader.setSourceId(m_node->GetId());
	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	std::string lane = m_sensor->getLane();
	nrheader.setPreLane(oldLane);
	nrheader.setCurrentLane(lane);

	Ptr<Packet> newPayload = Create<Packet> ();
	newPayload->AddHeader(nrheader);
	interest->SetPayload(newPayload);

	ndn::nrndn::PacketTypeTag typeTag(ASK_FOR_TABLE );
	interest->GetPayload()->AddPacketTag(typeTag);

	FwHopCountTag hopCountTag;
	interest->GetPayload()->AddPacketTag(hopCountTag);

	m_interestNonceSeen.Put(interest->GetNonce(),true);
	//cout<<"node: "<<m_node->GetId()<<" ask for table on "<<lane<<endl;

	SendInterestPacket(interest);
}

bool NavigationRouteHeuristic::isJuction(std::string lane)
{
	for(uint32_t i = 0; i<lane.length(); ++i)
		if(lane[i] == 't')
			return false;
	return true;
}

bool NavigationRouteHeuristic::isSameLane(string lane1, string lane2)//4_2to4_3与4_3to4_2是同路段
{
	if(lane1.length() != 8 || lane2.length() != 8)
		return false;
	if(lane1[0] == lane2[5] && lane1[2]==lane2[7] && lane1[5]==lane2[0] && lane1[7]==lane2[2])
		return true;
	if(lane1 == lane2)
		return true;
	return false;
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
	  if(m_sensor != NULL)
	  						m_sensor->TraceConnectWithoutContext ("LaneChange", MakeCallback (&NavigationRouteHeuristic::laneChange,this));
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
 	  if(fib)
 		 m_fib = DynamicCast<fib::nrndn::NrFibImpl>(fib);
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
	ndn::nrndn::nrHeader nrheader;
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


