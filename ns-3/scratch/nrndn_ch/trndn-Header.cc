/*
 * nrHeader.cc
 *
 *  Created on: Jan 15, 2015
 *      Author: chen
 */

#include "trndn-Header.h"

namespace ns3
{
namespace ndn
{
namespace nrndn
{

trndnHeader::trndnHeader():
		m_sourceId(0),
		m_x(0),
		m_y(0),
		m_type(0),
		m_TTL(0),
		m_nextid(0),
		m_currentid(0)
{
	// TODO Auto-generated constructor stub

}

trndnHeader::~trndnHeader()
{
	// TODO Auto-generated destructor stub
}

TypeId trndnHeader::GetTypeId()
{
	static TypeId tid = TypeId ("ns3::ndn::nrndn::trndnHeader")
	    .SetParent<Header> ()
	    .AddConstructor<trndnHeader> ()
	    ;
	return tid;
}

TypeId trndnHeader::GetInstanceTypeId() const
{
	return GetTypeId ();
}

uint32_t trndnHeader::GetSerializedSize() const
{
	uint32_t size=0;
	size += sizeof(m_sourceId);
	size += sizeof(m_x);
	size += sizeof(m_y);
	size += sizeof(m_type);
	size += sizeof(m_TTL);
	size += sizeof(m_nextid);
	size += sizeof(m_currentid);
	size += sizeof(uint32_t);
	size += (m_idlist.size() * sizeof(uint32_t));
	return size;
}

void trndnHeader::Serialize(Buffer::Iterator start) const
{
	Buffer::Iterator& i = start;
	i.WriteHtonU32(m_sourceId);
	i.Write((uint8_t*)&m_x,sizeof(m_x));
	i.Write((uint8_t*)&m_y,sizeof(m_y));
	i.WriteHtonU32(m_type);
	i.WriteHtonU32(m_TTL);
	i.WriteHtonU32(m_nextid);
	i.WriteHtonU32(m_currentid);
	i.WriteHtonU32(m_idlist.size());
	for(uint32_t p = 0; p <m_idlist.size(); ++p)
	{
		i.WriteHtonU32(m_idlist[p]);
	}
}

uint32_t trndnHeader::Deserialize(Buffer::Iterator start)
{
	Buffer::Iterator i = start;
	m_sourceId	=	i.ReadNtohU32();
	i.Read((uint8_t*)&m_x,sizeof(m_x));
	i.Read((uint8_t*)&m_y,sizeof(m_y));
	m_type = 	i.ReadNtohU32();
	m_TTL = i.ReadNtohU32();
	m_nextid = i.ReadNtohU32();
	m_currentid =  i.ReadNtohU32();
	m_idlist.clear();
	uint32_t size = i.ReadNtohU32();
	for(uint32_t p = 0; p < size; ++p)
	{
		uint32_t temp = i.ReadNtohU32();
		m_idlist.push_back(temp);
	}

	uint32_t dist = i.GetDistanceFrom(start);
	NS_ASSERT(dist == GetSerializedSize());
	//this->Print(std::cout);
	return dist;
}

void trndnHeader::Print(std::ostream& os) const
{
	os<<"trndnHeader content: "
			<<" NodeID="<<m_sourceId
			<<" coordinate = ("<<m_x<<","<<m_y<<")"
			<<" type="<<m_type
			<<" TTL="<<m_TTL;

	os<<std::endl;
}

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */
