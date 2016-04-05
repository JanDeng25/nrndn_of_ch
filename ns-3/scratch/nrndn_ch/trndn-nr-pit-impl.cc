/*
 * ndn-nr-pit-impl.cc
 *
 *  Created on: Jan 20, 2015
 *      Author: chen
 */

#include "trndn-nr-pit-impl.h"
#include "trndn-pit-entry-nrimpl.h"
//#include "NodeSensor.h"


#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("ndn.pit.TrNrPitImpl");

#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"


namespace ns3
{
namespace ndn
{
namespace pit
{
namespace nrndn
{


NS_OBJECT_ENSURE_REGISTERED (TrNrPitImpl);

TypeId
TrNrPitImpl::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::ndn::pit::nrndn::TrNrPitImpl")
    .SetGroupName ("Ndn")
    .SetParent<Pit> ()
    .AddConstructor< TrNrPitImpl > ()
   // .AddAttribute ("CleanInterval", "cleaning interval of the timeout incoming faces of PIT entry",
   	//		                    TimeValue (Seconds (10)),
   	//		                    MakeTimeAccessor (&TrNrPitImpl::m_cleanInterval),
   	//		                    MakeTimeChecker ())
    ;

  return tid;
}

TrNrPitImpl::TrNrPitImpl ():
		m_cleanInterval(Seconds(300.0))
{
}

TrNrPitImpl::~TrNrPitImpl ()
{
}


void
TrNrPitImpl::NotifyNewAggregate ()
{
	if (m_fib == 0)
	{
		m_fib = GetObject<Fib>();
	}
	if (m_forwardingStrategy == 0)
	{
		m_forwardingStrategy = GetObject<ForwardingStrategy>();
	}

	if (m_sensor == 0)
	{
		m_sensor = GetObject<ndn::nrndn::NodeSensor>();
		// Setup Lane change action
		/*if (m_sensor != NULL)
		{
			//m_sensor->TraceConnectWithoutContext("LaneChange",
				//	MakeCallback(&TrNrPitImpl::laneChange, this));

			//NrPitEntry needs m_sensor. Initialize immediately after m_sensor is aggregated
			InitializeNrPitEntry();
		}*/
	}

  Pit::NotifyNewAggregate ();
}

//add by DJ on Jan 4,2016:update pit
bool TrNrPitImpl::UpdatePit(uint32_t nexthop,Ptr<const Interest> interest)
{
	if(m_pitContainer.empty())
	{
		Ptr<fib::Entry> fibEntry=ns3::Create<fib::Entry>(Ptr<Fib>(0),Ptr<Name>(0));
		Ptr<TrEntryNrImpl> fentry = ns3::Create<TrEntryNrImpl>(*this,interest,fibEntry);
		fentry->AddIncomingNeighbors(nexthop);
		Ptr<Entry> pitEntry = DynamicCast<Entry>(fentry);
		m_pitContainer.push_back(pitEntry);
		//this->Print(std::cout);
		return true;;
	}
	else
	{
		//std::ostringstream os;
		std::vector<Ptr<pit::Entry> >::iterator pit=m_pitContainer.begin();
		for(;pit!=m_pitContainer.end();++pit)
		{
			Ptr<TrEntryNrImpl> pitEntry = DynamicCast<TrEntryNrImpl>(*pit);
			//const name::Component &pitName=(*pit)->GetInterest()->GetName().get(0);
			if(pitEntry->getEntryName() == interest->GetName().toUri())
			{
				std::unordered_set< uint32_t >::const_iterator it = pitEntry->getIncomingnbs().find(nexthop);
				if(it==pitEntry->getIncomingnbs().end())
				{
					pitEntry->AddIncomingNeighbors(nexthop);
				}
				//this->Print(std::cout);
				return true;
			}
		}
	    Ptr<fib::Entry> fibEntry=ns3::Create<fib::Entry>(Ptr<Fib>(0),Ptr<Name>(0));
	    Ptr<TrEntryNrImpl> fentry = ns3::Create<TrEntryNrImpl>(*this,interest,fibEntry);
	    fentry->AddIncomingNeighbors(nexthop);
		Ptr<Entry> pitEntry = DynamicCast<Entry>(fentry);
		m_pitContainer.push_back(pitEntry);
		//this->Print(std::cout);
	}
	return true;
}

//Mar 17,2016: merge two pit table
 void
 TrNrPitImpl::mergePit(std::vector<Ptr<Entry> >  pitCon)
 {
	 if(pitCon.size()==0)
		 return;
	 std::vector<Ptr<Entry> >::iterator pit_pitCon = pitCon.begin();
	 for(;pit_pitCon!=pitCon.end();++pit_pitCon)
	 {
		 Ptr<TrEntryNrImpl> pit_pitConEntry = DynamicCast<TrEntryNrImpl>(*pit_pitCon);
		 std::unordered_set< uint32_t >::const_iterator incomingnb = pit_pitConEntry->getIncomingnbs().begin();
		 for(;incomingnb != pit_pitConEntry->getIncomingnbs().end(); ++incomingnb){
			 UpdatePit(*incomingnb, pit_pitConEntry->GetInterest());
		 }
	 }

 }

//add by DJ on Jan 4,2016:update pit
bool TrNrPitImpl::RemovePitEntry(const Name& name)
{
	std::vector<Ptr<Entry> >::iterator pit=m_pitContainer.begin();
	//Ptr<Entry> entry = *pit;
	for(;pit!=m_pitContainer.end();++pit)
	{
		Ptr<TrEntryNrImpl> pitEntry = DynamicCast<TrEntryNrImpl>(*pit);
		if(pitEntry->getEntryName() == name.toUri())
		{
			//pitEntry->RemoveIncomingNeighbors(name.toUri());
			m_pitContainer.erase(pit);
			//this->Print(std::cout);
			return true;
		}
	}
	//this->Print(std::cout);
	return false;
}

void
TrNrPitImpl::DoDispose ()
{
	m_forwardingStrategy = 0;
	//m_fib = 0;
  
	Pit::DoDispose ();
 }
  

Ptr<Entry>
TrNrPitImpl::Lookup (const Data &header)
{
	NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::Lookup (const Data &header) should not be invoked");
	return 0;
 }
  
Ptr<Entry>
TrNrPitImpl::Lookup (const Interest &header)
{
	NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::Lookup (const Interest &header) should not be invoked");
	return 0;
 }
  

Ptr<Entry>
TrNrPitImpl::Find (const Name &prefix)
{
	//NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::Find (const Name &prefix) should not be invoked");
	 NS_LOG_INFO ("Finding prefix"<<prefix.toUri());
	 std::vector<Ptr<Entry> >::iterator it;
	 //NS_ASSERT_MSG(m_pitContainer.size()!=0,"Empty pit container. No initialization?");
	 for(it=m_pitContainer.begin();it!=m_pitContainer.end();++it)
	 {
		 if((*it)->GetPrefix()==prefix)
			 return *it;
	 }
	return 0;
}
  

Ptr<Entry>
TrNrPitImpl::Create (Ptr<const Interest> header)
 {

	NS_LOG_DEBUG (header->GetName ());
	NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::Create (Ptr<const Interest> header) "
			"should not be invoked, use "
			"TrNrPitImpl::CreateNrPitEntry instead");
	return 0;
}

// need to modify:how to initialize?
//test pit
/*bool
TrNrPitImpl::InitializeNrPitEntry()
{
	NS_LOG_FUNCTION (this);
	const std::vector<std::string>& route =	m_sensor->getNavigationRoute();
	std::vector<std::string>::const_iterator rit;
	for(rit=route.begin();rit!=route.end();++rit)
	{
		Ptr<Name> name = ns3::Create<Name>('/'+*rit);
		Ptr<Interest> interest=ns3::Create<Interest> ();
		interest->SetName				(name);
		interest->SetInterestLifetime	(Time::Max());//never expire

		//Create a fake FIB entry(if not ,L3Protocol::RemoveFace will have problem when using pitEntry->GetFibEntry)
		Ptr<fib::Entry> fibEntry=ns3::Create<fib::Entry>(Ptr<Fib>(0),Ptr<Name>(0));

		Ptr<Entry> entry = ns3::Create<TrEntryNrImpl>(*this,interest,fibEntry) ;

		m_pitContainer.push_back(entry);
		NS_LOG_DEBUG("Initialize pit:Push_back"<<name->toUri());
	}
	return true;
}
  */

void
TrNrPitImpl::MarkErased (Ptr<Entry> item)
{
	NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::MarkErased (Ptr<Entry> item) should not be invoked");
	return;
}
  
  
void
TrNrPitImpl::Print (std::ostream& os) const
{
	os<<"my lane:"<<m_sensor->getLane()<<std::endl;
	os<<"PIT content "<<std::endl;;
	std::vector<Ptr<Entry> >::const_iterator it;
	for(it=m_pitContainer.begin();it!=m_pitContainer.end();++it)
	{
		Ptr<ndn::pit::nrndn::TrEntryNrImpl>  temp = DynamicCast<ndn::pit::nrndn::TrEntryNrImpl>(*it);
		os<<"name:   "<< temp->getEntryName();
		std::unordered_set< uint32_t >::const_iterator i = temp->getIncomingnbs().begin();
		for(; i !=  temp->getIncomingnbs().end(); ++i)
			os<<" lane:"<<(*i);
		os<<std::endl;
	}
}

uint32_t
TrNrPitImpl::GetSize () const
{
	return m_pitContainer.size ();
}
  
Ptr<Entry>
TrNrPitImpl::Begin ()
{
	//NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::Begin () should not be invoked");

	if(m_pitContainer.begin() == m_pitContainer.end())
		return End();
	else
		return *(m_pitContainer.begin());
}

Ptr<Entry>
TrNrPitImpl::End ()
{
	//NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::End () should not be invoked");
	return 0;
}
  
Ptr<Entry>
TrNrPitImpl::Next (Ptr<Entry> from)
{
	//NS_ASSERT_MSG(false,"In TrNrPitImpl,TrNrPitImpl::Next () should not be invoked");
	if (from == 0) return 0;

	std::vector<Ptr<Entry> >::iterator it;
	it = find(m_pitContainer.begin(),m_pitContainer.end(),from);
	if(it==m_pitContainer.end())
		return End();
	else
	{
		++it;
		if(it==m_pitContainer.end())
			return End();
		else
			return *it;
	}
}

//小锟添加，2015-8-23
std::string TrNrPitImpl::uriConvertToString(std::string str)
{
	//因为获取兴趣时使用toUri，避免出现类似[]的符号，进行编码转换
	std::string ret="";
	for(uint32_t i=0;i<str.size();i++)
	{
		if(i+2<str.size())
		{
			if(str[i]=='%'&&str[i+1]=='5'&&str[i+2]=='B')
			{
				ret+="[";
				i=i+2;
			}
			else if(str[i]=='%'&&str[i+1]=='5'&&str[i+2]=='D')
			{
				ret+="]";
				i=i+2;
			}
			else
				ret+=str[i];
		}
		else
			ret+=str[i];
	}
	return ret;
}


//laneChange means sending packet back to neighbors in last hop whether forward its data packets or not?
/*void TrNrPitImpl::laneChange(std::string oldLane, std::string newLane)
{
	if (oldLane.empty()
			|| (ndn::nrndn::NodeSensor::emptyLane == oldLane
					&& ndn::nrndn::NodeSensor::emptyLane != newLane))
		return;
	NS_LOG_INFO ("Deleting old lane pit entry of "<<oldLane);

	std::vector<Ptr<Entry> >::iterator it;
	it =m_pitContainer.begin();

	bool IsOldLaneAtPitBegin =(  uriConvertToString((*it)->GetInterest()->GetName().get(0).toUri())==(oldLane));

	if(!IsOldLaneAtPitBegin)
	{
		std::cout<<"旧路段不在头部:"<<"oldLane:"<<(oldLane)<<" newLane:"<<uriConvertToString((*it)->GetInterest()->GetName().get(0).toUri())<<std::endl;

		//遍历整个Pit
		std::vector<Ptr<Entry> >::iterator itTraversal;
		itTraversal =m_pitContainer.begin();
		bool findOldLane=false;
		std::cout<<"寻找oldLane中...\n";
		for(;itTraversal!=m_pitContainer.end();itTraversal++)
		{//遍历整个PIT表，寻找oldLane是否在表中
			if( uriConvertToString((*itTraversal)->GetInterest()->GetName().get(0).toUri()) == (oldLane) )
			{//如果找到则直接跳出
				findOldLane=true;
				break;
			}
		}
		if(findOldLane)
		{
			it =m_pitContainer.begin();
			int a=0;
			while(  uriConvertToString((*it)->GetInterest()->GetName().get(0).toUri())!=(oldLane)
					&&it!=m_pitContainer.end())
			{
				std::cout<<a<<"遍历删除中："<<uriConvertToString( (*it)->GetInterest()->GetName().get(0).toUri())<<" OLd:"<<(oldLane)<<std::endl;
				a++;
				DynamicCast<TrEntryNrImpl>(*it)->RemoveAllTimeoutEvent();
				m_pitContainer.erase(it);
				it =m_pitContainer.begin();
			}
			if(it<=m_pitContainer.end())
			{
				std::cout<<"最后遍历删除中："<<uriConvertToString( (*it)->GetInterest()->GetName().get(0).toUri())<<" OLd:"<<(oldLane)<<std::endl;
				//1. Befor erase it, cancel all the counting Timer fore the neighbor to expire
				DynamicCast<TrEntryNrImpl>(*it)->RemoveAllTimeoutEvent();
				//2. erase it
				m_pitContainer.erase(it);
				std::cout<<"删除完毕\n";
			}
			else
				std::cout<<"删除完毕：迭代器为空\n";

		}
		else
		{
			std::cout<<"没找到...\n";
		}
	}
	else
	{//旧路段在pit头部才进行删除

			//报错？
		//NS_ASSERT_MSG(IsOldLaneAtPitBegin,"The old lane should at the beginning of the pitContainer. Please Check~");
		//1. Befor erase it, cancel all the counting Timer fore the neighbor to expire
		DynamicCast<TrEntryNrImpl>(*it)->RemoveAllTimeoutEvent();

		//2. erase it
		m_pitContainer.erase(it);
		//std::cout<<"erase OK!"<<std::endl;
		return;
	}

}*/

void TrNrPitImpl::DoInitialize(void)
{
	Pit::DoInitialize();
}

} /* namespace nrndn */
} /* namespace pit */
} /* namespace ndn */
} /* namespace ns3 */


