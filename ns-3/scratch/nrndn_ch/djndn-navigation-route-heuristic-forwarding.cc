/*
 * ndn-navigation-route-heuristic-forwarding.cc
 *
 *  Created on: Jan 14, 2015
 *      Author: chen
 */

#include "djndn-navigation-route-heuristic-forwarding.h"
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

NS_LOG_COMPONENT_DEFINE ("ndn.fw.djNavigationRouteHeuristic");

namespace ns3
{
namespace ndn
{
namespace fw
{
namespace nrndn
{

NS_OBJECT_ENSURE_REGISTERED (djNavigationRouteHeuristic);

TypeId djNavigationRouteHeuristic::GetTypeId(void)
{
	  static TypeId tid = TypeId ("ns3::ndn::fw::nrndn::djNavigationRouteHeuristic")
	    .SetGroupName ("Ndn")
	    .SetParent<GreenYellowRed> ()
	    .AddConstructor<djNavigationRouteHeuristic>()
	    .AddAttribute ("HelloInterval", "HELLO messages emission interval.",
	            TimeValue (Seconds (1)),
	            MakeTimeAccessor (&djNavigationRouteHeuristic::HelloInterval),
	            MakeTimeChecker ())
	     .AddAttribute ("AllowedHelloLoss", "Number of hello messages which may be loss for valid link.",
	            UintegerValue (2),
	            MakeUintegerAccessor (&djNavigationRouteHeuristic::AllowedHelloLoss),
	            MakeUintegerChecker<uint32_t> ())

	   	 .AddAttribute ("gap", "the time gap between interested nodes and disinterested nodes for sending a data packet.",
	   	        UintegerValue (20),
	   	        MakeUintegerAccessor (&djNavigationRouteHeuristic::m_gap),
	   	        MakeUintegerChecker<uint32_t> ())
//		.AddAttribute ("CacheSize", "The size of the cache which records the packet sent, use LRU scheme",
//				UintegerValue (6000),
//				MakeUintegerAccessor (&NavigationRouteHeuristic::SetCacheSize,
//									  &NavigationRouteHeuristic::GetCacheSize),
//				MakeUintegerChecker<uint32_t> ())
        .AddAttribute ("UniformRv", "Access to the underlying UniformRandomVariable",
        		 StringValue ("ns3::UniformRandomVariable"),
        		 MakePointerAccessor (&djNavigationRouteHeuristic::m_uniformRandomVariable),
        		 MakePointerChecker<UniformRandomVariable> ())
        .AddAttribute ("HelloLogEnable", "the switch which can turn on the log on Functions about hello messages",
        		 BooleanValue (true),
        		 MakeBooleanAccessor (&djNavigationRouteHeuristic::m_HelloLogEnable),
        		 MakeBooleanChecker())
        .AddAttribute ("NoFwStop", "When the PIT covers the nodes behind, no broadcast stop message",
        		 BooleanValue (false),
        		 MakeBooleanAccessor (&djNavigationRouteHeuristic::NoFwStop),
        		 MakeBooleanChecker())
		.AddAttribute ("TTLMax", "This value indicate that when a data is received by disinterested node, the max hop count it should be forwarded",
		         UintegerValue (3),
		         MakeUintegerAccessor (&djNavigationRouteHeuristic::m_TTLMax),
		         MakeUintegerChecker<uint32_t> ())
		 .AddAttribute ("PayloadSize", "Virtual payload size for Content packets",
				 UintegerValue (1024),
				 MakeUintegerAccessor (&djNavigationRouteHeuristic::m_virtualPayloadSize),
				  MakeUintegerChecker<uint32_t> ())
		 .AddAttribute ("Freshness", "Freshness of data packets, if 0, then unlimited freshness",
				 TimeValue (Seconds (0)),
				  MakeTimeAccessor (&djNavigationRouteHeuristic::m_freshness),
				  MakeTimeChecker ())
	    .AddAttribute ("Pit","pit of forwarder",
	    		  PointerValue (),
		    	  MakePointerAccessor (&djNavigationRouteHeuristic::m_pit),
		    	  MakePointerChecker<ns3::ndn::pit::nrndn::NrPitImpl> ())
	    .AddAttribute ("Fib","fib of forwarder",
	    	  PointerValue (),
		    	  MakePointerAccessor (&djNavigationRouteHeuristic::m_fib),
		    	  MakePointerChecker<ns3::ndn::fib::nrndn::NrFibImpl> ())
			.AddAttribute ("CS","cs of forwarder",
	    		  PointerValue (),
		    	  MakePointerAccessor (&djNavigationRouteHeuristic::m_cs),
		    	  MakePointerChecker<ns3::ndn::cs::nrndn::NrCsImpl> ())
	    ;
	  return tid;
}

djNavigationRouteHeuristic::djNavigationRouteHeuristic():
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
	m_htimer.SetFunction (&djNavigationRouteHeuristic::HelloTimerExpire, this);
	m_nb.SetCallback (MakeCallback (&djNavigationRouteHeuristic::FindBreaksLinkToNextHop, this));
}

djNavigationRouteHeuristic::~djNavigationRouteHeuristic ()
{

}

void djNavigationRouteHeuristic::Start()
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
	Simulator::Schedule (Seconds (50), &djNavigationRouteHeuristic::fibnum, this);
}

void djNavigationRouteHeuristic::fibnum()
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
	Simulator::Schedule (Seconds (2), & djNavigationRouteHeuristic::fibnum, this);
}



void djNavigationRouteHeuristic::Stop()
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

void djNavigationRouteHeuristic::WillSatisfyPendingInterest(
		Ptr<Face> inFace, Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

bool djNavigationRouteHeuristic::DoPropagateInterest(
		Ptr<Face> inFace, Ptr<const Interest> interest,
		Ptr<pit::Entry> pitEntry)
{
	  NS_LOG_FUNCTION (this);
	  NS_LOG_UNCOND(this <<" is in unused function");
	  NS_ASSERT_MSG (m_pit != 0, "PIT should be aggregated with forwarding strategy");

	  return true;
}

void djNavigationRouteHeuristic::WillEraseTimedOutPendingInterest(
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void djNavigationRouteHeuristic::AddFace(Ptr<Face> face)
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

void djNavigationRouteHeuristic::RemoveFace(Ptr<Face> face)
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

void djNavigationRouteHeuristic::DidReceiveValidNack(
		Ptr<Face> incomingFace, uint32_t nackCode, Ptr<const Interest> nack,
		Ptr<pit::Entry> pitEntry)
{
	 NS_LOG_FUNCTION (this);
	 NS_LOG_UNCOND(this <<" is in unused function");
}

void djNavigationRouteHeuristic::OnInterest(Ptr<Face> face,
		Ptr<Interest> interest)
{
	//NS_LOG_UNCOND("Here is NavigationRouteHeuristic dealing with OnInterest");
	//NS_LOG_FUNCTION (this);
	if(!m_running) return;
	//cout<<"into on interest"<<endl;
	if(Face::APPLICATION==face->GetFlags())
	{
		NS_LOG_DEBUG("Get interest packet from APPLICATION");

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
	double interval = (600 - distance) *1.5;

	if(DETECT_PACKET == interest->GetScope())
	{
		if(!isDuplicatedInterest(nodeId,seq) && !isJuction(m_sensor->getLane()))//第一次收到此包
		{
			if(m_fib->Find(interest->GetName()) || m_cs->Find(interest->GetName()))
			{
				Time sendInterval = MilliSeconds(distance);
				//cout<<"detect packet  send interval: "<<sendInterval.GetSeconds()<<endl;
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&djNavigationRouteHeuristic::ReplyConfirmPacket, this,interest);//回复的确认包，设置为此探测包的nonce和nodeid
				return;
			}
			else if(!isSameLane(m_sensor->getLane(), currentLane) && IsConnected(m_sensor->getLane(), currentLane))
			{
				Time sendInterval = (MilliSeconds(interval) +  m_gap * m_timeSlot);
				//cout<<"detect packet   send interval: "<<sendInterval.GetSeconds()<<endl;
				m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&djNavigationRouteHeuristic::ForwardDetectPacket, this,interest);
				return;
			}
			else if(isSameLane(m_sensor->getLane(), currentLane))
			{
				Time sendInterval = (MilliSeconds(interval) +  (m_gap+5) * m_timeSlot);
				//cout<<"detect packet   send interval: "<<sendInterval.GetSeconds()<<endl;
				m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
									&djNavigationRouteHeuristic::ForwardDetectPacket, this,interest);
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
			if(m_cs->Find(interest->GetName()) /*&&( isSameLane(m_sensor->getLane(),currentLane) || isSameLane(m_sensor->getLane(),preLane))*/)
			{
				Time sendInterval = MilliSeconds(distance);
				m_sendingDataEvent[nodeId][seq] = Simulator::Schedule(sendInterval*2,
								&djNavigationRouteHeuristic::ReplyDataPacket, this,interest);//回复的数据包，设置为此探测包的nonce和nodeid
				return;
			}
			else if(!isSameLane(m_sensor->getLane(),currentLane)&& !isSameLane(m_sensor->getLane(),preLane))
			{//不在应接受的下一跳路段，也不在与interest包同路段
				//cout<<"not on the section of interest packet, my section:"<<m_sensor->getLane()<<" current lane:"<<currentLane<<" pre lane"<<preLane<<endl;
				m_interestNonceSeen.Put(interest->GetNonce(),true);//不做处理
				return;
			}
			else if(m_pit->Find(interest->GetName()) && !isJuction(m_sensor->getLane()) && isSameLane(m_sensor->getLane(),currentLane))
			{
				m_pit->UpdatePit(preLane, interest);
				m_interestNonceSeen.Put(interest->GetNonce(),true);
				return;
			}
			else if(m_pit->Find(interest->GetName()) && !isJuction(m_sensor->getLane()) && isSameLane(m_sensor->getLane(),preLane))
			{
				m_pit->UpdatePit(laneList.front(), interest);
				m_interestNonceSeen.Put(interest->GetNonce(),true);
				return;
			}
			else if(!isJuction(m_sensor->getLane()))//在路口的车辆不参与转发
			{
				if(m_fib->Find(interest->GetName()))
				{
					if(isSameLane(m_sensor->getLane(),currentLane))
					{
						m_pit->UpdatePit(preLane, interest);
						Time sendInterval = (MilliSeconds(interval)+  m_gap* m_timeSlot);
						m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
												&djNavigationRouteHeuristic::ForwardInterestPacket, this,interest);
					}
					else if(isSameLane(m_sensor->getLane(),preLane))//处在兴趣包应走的下一跳路段的车辆优先转发
					{
						m_pit->UpdatePit(laneList.front(), interest);
						Time sendInterval = (MilliSeconds(interval) +  (m_gap+5)* m_timeSlot);
						m_sendingInterestEvent[nodeId][seq] = Simulator::Schedule(sendInterval,
												&djNavigationRouteHeuristic::ForwardInterestPacket, this,interest);
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
}

void djNavigationRouteHeuristic::OnData(Ptr<Face> face, Ptr<Data> data)
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
				&djNavigationRouteHeuristic::SendDataPacket, this, data);

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
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);

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
		resourceReceived.insert(signature);

		Time sendInterval = (MilliSeconds(interval) );
		if(isSameLane(m_sensor->getLane(),currentLane) && !m_fib->Find(data->GetName()))
		{
				m_fib-> AddFibEntry(data->GetNamePtr(),preLane, hopCountTag.Get() );
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval, &djNavigationRouteHeuristic::ForwardResourcePacket, this,data);
		}
		else  if(!m_fib->Find(data->GetName()) && IsConnected(m_sensor->getLane(), currentLane) && !isSameLane(m_sensor->getLane(),currentLane)&& !isSameLane(m_sensor->getLane(),preLane)  )
		{
				m_fib-> AddFibEntry(data->GetNamePtr(),currentLane, hopCountTag.Get() );
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval+ m_gap* m_timeSlot, &djNavigationRouteHeuristic::ForwardResourcePacket, this,data);
		}
			return;
	}//end if (RESOURCE_PACKET == packetTypeTag.Get())
	else if (DATA_PACKET == packetTypeTag.Get())
	{
		if(!isDuplicatedData(nodeId,signature))
		{
			//cout<<"header size:"<<nrheader.GetSerializedSize()<<endl;
			//cout<<"node: "<<m_node->GetId()<<" receive data packet from "<<nodeId<<endl;
			m_cs->Add(data);
			//m_pit->RemovePitEntry(data->GetName());
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
									&djNavigationRouteHeuristic::ForwardDataPacket, this,data);
				}
				else if(isSameLane(m_sensor->getLane(),preLane))//与数据包同路段
				{
					Time sendInterval = (MilliSeconds(interval) + m_gap* m_timeSlot);
					m_sendingDataEvent[nodeId][signature]=
									Simulator::Schedule(sendInterval,
									&djNavigationRouteHeuristic::ForwardDataPacket, this,data);
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
			m_pit->RemovePitEntry(data->GetName());
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
				 m_interestNonceSeen.Put(signature,true);
			 }
			 if(m_fib->Find(data->GetName()))
			{
				m_dataSignatureSeen.Put(data->GetSignature(),true);
				//ExpireDataPacketTimer(nodeId,signature);
				return;
			}
			//建立FIB表项
			 m_fib-> AddFibEntry(data->GetNamePtr(),preLane, nrheader.getTTL());
			if(isSameLane(m_sensor->getLane(),currentLane))
			{
				Time sendInterval = (MilliSeconds(interval) );
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&djNavigationRouteHeuristic::ForwardConfirmPacket, this,data);
				return;
			}
			else if(isSameLane(m_sensor->getLane(),preLane))
			{
				Time sendInterval = (MilliSeconds(interval) + (m_gap) * m_timeSlot);
				m_sendingDataEvent[nodeId][signature]=
								Simulator::Schedule(sendInterval,
								&djNavigationRouteHeuristic::ForwardConfirmPacket, this,data);
				return;
			}
			else
			{
				m_dataSignatureSeen.Put(data->GetSignature(),true);
				return;
			}
		}
		else if(m_fib->Find(data->GetName()))
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

bool djNavigationRouteHeuristic::OnTheWay(std::vector<std::string> laneList)
{
	//cout<<"into on the way"<<endl;
	for(uint32_t i = 0; i < laneList.size(); ++i)
		if(isSameLane(m_sensor->getLane(),laneList[i]))
			return true;
	return false;
}

bool djNavigationRouteHeuristic::isDuplicatedInterest(
		uint32_t id, uint32_t nonce)
{
	NS_LOG_FUNCTION (this);
	//cout<<"into is duplicated interest"<<endl;
	if(!m_sendingInterestEvent.count(id))
		return false;
	else
		return m_sendingInterestEvent[id].count(nonce);
}

bool djNavigationRouteHeuristic::isDuplicatedData(uint32_t id, uint32_t signature)
{
	NS_LOG_FUNCTION (this);
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::isDuplicatedData");
	if(!m_sendingDataEvent.count(id))
		return false;
	else
		return m_sendingDataEvent[id].count(signature);
}

void djNavigationRouteHeuristic::ExpireInterestPacketTimer(uint32_t nodeId,uint32_t seq)
{
	//cout<<"into expire interest packet timer"<<endl;
	NS_LOG_FUNCTION (this<< "ExpireInterestPacketTimer id"<<nodeId<<"sequence"<<seq);
	//1. Find the waiting timer
	EventId& eventid = m_sendingInterestEvent[nodeId][seq];
	
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void djNavigationRouteHeuristic::ExpireDataPacketTimer(uint32_t nodeId,uint32_t signature)
{
	//cout<<"into expire data packet timery"<<endl;
	//NS_ASSERT_MSG(false,"NavigationRouteHeuristic::ExpireDataPacketTimer");
	NS_LOG_FUNCTION (this<< "ExpireDataPacketTimer id\t"<<nodeId<<"\tsignature:"<<signature);
	//1. Find the waiting timer
	EventId& eventid = m_sendingDataEvent[nodeId][signature];
	//2. cancel the timer if it is still running
	eventid.Cancel();
}

void djNavigationRouteHeuristic::ForwardResourcePacket(Ptr<Data> src)
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
	if(isSameLane(currentlane, nrheader.getCurrentLane()))
		prelane = nrheader.getPreLane();
	else
		prelane = nrheader.getCurrentLane();
	vector<string> lanelist = nrheader.getLaneList();

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

	cout<<"node: "<<m_node->GetId()<<" forward resource packet from "<<nrheader.getSourceId()<<" name:"<<data->GetName().toUri ()<<" prelane:"<<prelane<<" currentlane:"<<nrheader.getCurrentLane()<<" ttl:"<<(hopCountTag.Get()+1)<<endl;
	SendDataPacket(data);

	ndn::nrndn::nrUtils::IncreaseResourceForwardCounter();
	cout<<"now ResourceForwardSum = "<<ndn::nrndn::nrUtils::ResourceForwardSum<<endl;
}

void djNavigationRouteHeuristic::ForwardConfirmPacket(Ptr<Data> src)
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
	//cout<<lanelist.size()<<endl;
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
	cout<<"NOW ConfirmForwardSum = "<<ndn::nrndn::nrUtils::ConfirmForwardSum<<endl;
}

void djNavigationRouteHeuristic::ForwardDataPacket(Ptr<Data> src)
{
		if(!m_running) return;
		//cout<<"forward data packet"<<endl;
		Ptr<Packet> nrPayload=src->GetPayload()->Copy();
		ndn::nrndn::nrndnHeader nrheader;
		nrPayload->RemoveHeader(nrheader);
		double x= m_sensor->getX();
		double y= m_sensor->getY();
		std::string prelane = m_sensor->getLane();
		std::vector<std::string> lanelist;
		///检查PIT表项，维护lanelist（备选下一跳）
		//NS_ASSERT_MSG(m_pit->Find( src->GetName()) != 0,"pit not find ");

		if(m_pit->Find( src->GetName()) == 0){
			cout<<"pit not find"<<endl;
			return ;
		}
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
		cout<<"node: "<<m_node->GetId()<<" forward data packet to "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get()<<" currentlane:"<<m_sensor->getLane();
		cout<<" next lane: ";
		for(uint32_t i = 0; i<lanelist.size(); ++i)
		{
			cout<<lanelist[i]<<" ";
		}
		cout<<endl;
		m_pit->RemovePitEntry(data->GetName());
		SendDataPacket(data);

		ndn::nrndn::nrUtils::IncreaseDataForwardCounter();
}

void djNavigationRouteHeuristic::ForwardDetectPacket(Ptr<Interest> src)
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
	nrheader.setCurrentLane(m_sensor->getLane());
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

void djNavigationRouteHeuristic::ForwardInterestPacket(Ptr<Interest> src)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"node: "<<m_node->GetId()<<" into forward interest packet"<<endl;
	m_interestNonceSeen.Put(src->GetNonce(),true);
	// 2. prepare the interest
	Ptr<Packet> nrPayload=src->GetPayload()->Copy();
	ndn::nrndn::nrndnHeader nrheader;
	nrPayload->RemoveHeader(nrheader);
	string preprelane = nrheader.getPreLane();
	vector<string> lanelist;
	lanelist.push_back(preprelane);
	nrheader.setX(m_sensor->getX());
	nrheader.setY(m_sensor->getY());
	nrheader.setPreLane(m_sensor->getLane());
	nrheader.setLaneList(lanelist);
	if(m_fib->Find( src->GetName()) == 0)
	{
		return;
	}
	Ptr<ndn::fib::nrndn::EntryNrImpl> nexthop = DynamicCast<ndn::fib::nrndn::EntryNrImpl>(m_fib->Find( src->GetName()));
	nrheader.setCurrentLane(nexthop->getIncomingnbs().begin()->first);
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

	cout<<"node: "<<m_node->GetId()<<" forward interest packet from "<<nrheader.getSourceId()<<" ttl:"<<hopCountTag.Get()<<" currentlane"<<m_sensor->getLane()<<" nextlane"<<nrheader.getCurrentLane()<<endl;
	SendInterestPacket(interest);
	ndn::nrndn::nrUtils:: IncreaseInterestForwardCounter();
	cout<<"Now InterestForwardSum = "<<ndn::nrndn::nrUtils::InterestForwardSum<<endl;
}

void djNavigationRouteHeuristic::ReplyConfirmPacket(Ptr<Interest> interest)
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
	cout<<"node: "<<m_node->GetId()<<" reply confirm packet to "<<nrheader.getSourceId()<<" ttl:"<<ttl<<" current lane"<<m_sensor->getLane()<<" next lane:"<<currentlane<<endl;
	ndn::nrndn::nrUtils:: IncreaseConfirmNum();
	SendDataPacket(data);
}

void djNavigationRouteHeuristic::ReplyDataPacket(Ptr<Interest> interest)
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
	//cout<<"header size"<<nrheader.GetSerializedSize()<<endl;
	data->SetPayload(newPayload);

	ndn::nrndn::PacketTypeTag typeTag(DATA_PACKET );
	data->GetPayload()->AddPacketTag(typeTag);

	FwHopCountTag hopCountTag;
	data->GetPayload()->AddPacketTag(hopCountTag);

	m_dataSignatureSeen.Put(data->GetSignature(),true);

	cout<<"node: "<<m_node->GetId()<<" reply data packet to "<<nrheader.getSourceId()<<" current lane:"<<m_sensor->getLane()<<" next lane:"<<nrheader.getPreLane()<<endl;
	ndn::nrndn::nrUtils::IncreaseDataNum();
	SendDataPacket(data);
}

void djNavigationRouteHeuristic::PrepareInterestPacket(Ptr<Interest> interest)
{
	if(!m_running) return;
	NS_LOG_FUNCTION (this);
	//cout<<"into prepare interest packet"<<endl;
	// 2. prepare the interest
	interest->SetScope(INTEREST_PACKET);
	//interest->SetNonce(m_uniformRandomVariable->GetValue());
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
	Simulator::Schedule(
					MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100)),
					&djNavigationRouteHeuristic::SendInterestPacket, this, interest);
	//interest->SetNonce(m_rand.GetValue());
	//SendInterestPacket(interest);
	cout<<"node: "<<m_node->GetId()<<"  send interest packet,name: "<<interest->GetName().toUri()<<" current lane:"<<m_sensor->getLane()<<" next lane: "<<hop<<" scope:"<<(int)(interest->GetScope())<<endl;
//getchar();
	m_interestNonceSeen.Put(interest->GetNonce(),true);
	ndn::nrndn::nrUtils::IncreaseInterestNum();

}

void djNavigationRouteHeuristic::PrepareDetectPacket(Ptr<Interest> interest)
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
	//SendInterestPacket(interest);
	Simulator::Schedule(
						MilliSeconds(m_uniformRandomVariable->GetInteger(0, 100)),
						&djNavigationRouteHeuristic::SendInterestPacket, this, interest);
	Simulator::Schedule (Seconds (5.0), & djNavigationRouteHeuristic::PreparePacket, this,interest);
	ndn::nrndn::nrUtils::IncreaseDetectNum();
}

void djNavigationRouteHeuristic::PreparePacket(Ptr<Interest> interest)
{
	//cout<<"prepare packet"<<endl;
	if(m_fib->getFIB().size() == 0)
		Simulator::Schedule (Seconds (5.0), & djNavigationRouteHeuristic::PreparePacket, this,interest);
	else if (m_fib->Find(interest->GetName()))
				PrepareInterestPacket(interest);
	else
				PrepareDetectPacket(interest);
}

void djNavigationRouteHeuristic::SendInterestPacket(Ptr<Interest> interest)
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

void djNavigationRouteHeuristic::SendDataPacket(Ptr<Data> data)
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

void djNavigationRouteHeuristic::SendHello()
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

void djNavigationRouteHeuristic::DoInitialize(void)
{
	//cout<<"doinital"<<endl;
	if (m_sensor == 0)
	{
		m_sensor = m_node->GetObject<ndn::nrndn::NodeSensor>();
		NS_ASSERT_MSG(m_sensor,"djNavigationRouteHeuristic::DoInitialize cannot find ns3::ndn::nrndn::NodeSensor");
		if(m_sensor != NULL)
						m_sensor->TraceConnectWithoutContext ("LaneChange", MakeCallback (&djNavigationRouteHeuristic::laneChange,this));
	}
	super::DoInitialize();
}

void djNavigationRouteHeuristic::laneChange(std::string oldLane, std::string newLane)
{
	if (!m_running)  return;
	if(isJuction(newLane)) return;
	if(Simulator::Now().GetSeconds() < 50) return;

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

}

bool djNavigationRouteHeuristic::isJuction(std::string lane)
{
	for(uint32_t i = 0; i<lane.length(); ++i)
		if(lane[i] == 't')
			return false;
	return true;
}

bool djNavigationRouteHeuristic::isSameLane(string lane1, string lane2)//4_2to4_3与4_3to4_2是同路段
{
	//cout<<"is same lane"<<endl;
	if(lane1.length() != 8 || lane2.length() != 8)
		return false;
	if(lane1[0] == lane2[5] && lane1[2]==lane2[7] && lane1[5]==lane2[0] && lane1[7]==lane2[2])
		return true;
	if(lane1 == lane2)
		return true;
	return false;
}

bool djNavigationRouteHeuristic::IsConnected(string lane1, string lane2)//判断两路段是否相连
{
	//cout<<"is connected"<<endl;
	if(isJuction(lane1) || isJuction(lane2))
		return false;
	if(isSameLane(lane1, lane2))
		return false;
	if(lane1[0] == lane2[0] && lane1[2] == lane2[2])
		return true;
	if(lane1[0] == lane2[5] && lane1[2] == lane2[7])
			return true;
	if(lane1[5] == lane2[0] && lane1[7] == lane2[2])
			return true;
	if(lane1[5] == lane2[5] && lane1[7] == lane2[7])
			return true;
	return false;
}

void djNavigationRouteHeuristic::DropPacket()
{
	NS_LOG_DEBUG ("Drop Packet");
}

void djNavigationRouteHeuristic::DropDataPacket(Ptr<Data> data)
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

void djNavigationRouteHeuristic::DropInterestePacket(Ptr<Interest> interest)
{
	NS_LOG_DEBUG ("Drop interest Packet");
	DropPacket();
}

void djNavigationRouteHeuristic::NotifyNewAggregate()
{

  if (m_sensor == 0)
  {
	  m_sensor = GetObject<ndn::nrndn::NodeSensor> ();
	  if(m_sensor != NULL)
	  						m_sensor->TraceConnectWithoutContext ("LaneChange", MakeCallback (&djNavigationRouteHeuristic::laneChange,this));
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
djNavigationRouteHeuristic::HelloTimerExpire ()
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
djNavigationRouteHeuristic::FindBreaksLinkToNextHop(uint32_t BreakLinkNodeId)
{
	 NS_LOG_FUNCTION (this);
}

void
djNavigationRouteHeuristic::ProcessHello(Ptr<Interest> interest)
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

void djNavigationRouteHeuristic::ToContentStore(Ptr<Data> data)
{
	NS_LOG_DEBUG ("To content store.(Just a trace)");
	return;
}

void djNavigationRouteHeuristic::NotifyUpperLayer(Ptr<Data> data)
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


