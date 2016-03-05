/*
 * nrHeader.cc
 *
 *  Created on: Jan 15, 2015
 *      Author: chen
 */

#include "nrndn-Header.h"

namespace ns3
{
namespace ndn
{
namespace nrndn
{

nrndnHeader::nrndnHeader():
		m_sourceId(0),
		m_x(0),
		m_y(0),
		m_type(0),
		m_TTL(0)
{
	// TODO Auto-generated constructor stub

}

nrndnHeader::~nrndnHeader()
{
	// TODO Auto-generated destructor stub
}

TypeId nrndnHeader::GetTypeId()
{
	static TypeId tid = TypeId ("ns3::ndn::nrndn::nrndnHeader")
	    .SetParent<Header> ()
	    .AddConstructor<nrndnHeader> ()
	    ;
	return tid;
}

TypeId nrndnHeader::GetInstanceTypeId() const
{
	return GetTypeId ();
}

uint32_t nrndnHeader::GetSerializedSize() const
{
	uint32_t size=0;
	size += sizeof(m_sourceId);
	size += sizeof(m_x);
	size += sizeof(m_y);
	size += sizeof(m_type);
	size += sizeof(m_TTL);
	size += sizeof(uint32_t);
	size += m_preLane.size();
	size += sizeof(uint32_t);
	size += m_currentLane.size();
	size += sizeof(uint32_t);
	for(std::vector<std::string>::const_iterator it = m_laneList.begin(); it != m_laneList.end(); ++it)
	{
		size += sizeof(uint32_t);
		size += (*it).size();
	}
	return size;
}

void nrndnHeader::Serialize(Buffer::Iterator start) const
{
	Buffer::Iterator& i = start;
	i.WriteHtonU32(m_sourceId);
	i.Write((uint8_t*)&m_x,sizeof(m_x));
	i.Write((uint8_t*)&m_y,sizeof(m_y));
	i.WriteHtonU32(m_type);
	i.WriteHtonU32(m_TTL);

	i.WriteHtonU32(m_preLane.size());
	for(uint32_t it = 0; it<m_preLane.size(); ++it)
		i.Write((uint8_t*)&(m_preLane[it]),sizeof(char));

	i.WriteHtonU32(m_currentLane.size());
	for(uint32_t it = 0; it<m_currentLane.size(); ++it)
		i.Write((uint8_t*)&(m_currentLane[it]),sizeof(char));

	i.WriteHtonU32(m_laneList.size());
	for(uint32_t it = 0; it<m_laneList.size(); ++it)
	{
		i.WriteHtonU32(m_laneList[it].size());
		for(uint32_t k = 0; k<m_laneList[it].size(); ++k)
			i.Write((uint8_t*)&(m_laneList[it][k]),sizeof(char));
	}
}

uint32_t nrndnHeader::Deserialize(Buffer::Iterator start)
{
	Buffer::Iterator i = start;
	m_sourceId	=	i.ReadNtohU32();
	i.Read((uint8_t*)&m_x,sizeof(m_x));
	i.Read((uint8_t*)&m_y,sizeof(m_y));
	m_type = 	i.ReadNtohU32();
	m_TTL = i.ReadNtohU32();

	uint32_t size;
	char tmp;

	std::string prelane;
	size = i.ReadNtohU32();
	for(uint32_t it = 0; it<size; ++it)
	{
		i.Read((uint8_t*)&(tmp),sizeof(char));
		prelane += tmp;
	}
	m_preLane = prelane;

	std::string currentlane;
	size = i.ReadNtohU32();
	for(uint32_t it = 0; it<size; ++it)
	{
		i.Read((uint8_t*)&(tmp),sizeof(char));
		currentlane+=tmp;
	}
	m_currentLane = currentlane;

	uint32_t size2;
	size = i.ReadNtohU32();
	std::vector<std::string> lanelist;
	for(uint32_t it = 0; it<size; ++it)
	{
		size2 = i.ReadNtohU32();
		std::string lane;
		for(uint32_t k = 0; k<size2; ++k)
		{
			i.Read((uint8_t*)&(tmp),sizeof(char));
			lane += tmp;
		}
		lanelist.push_back(lane);
	}

	std::vector<std::string> lanelist2;
	while(!lanelist.empty())
	{
		lanelist2.push_back(lanelist.back());
		lanelist.pop_back();
	}

	m_laneList.clear();
	while(!lanelist2.empty())
	{
		m_laneList.push_back(lanelist2.back());
		lanelist2.pop_back();
	}

	uint32_t dist = i.GetDistanceFrom(start);
	NS_ASSERT(dist == GetSerializedSize());
	return dist;
}

void nrndnHeader::Print(std::ostream& os) const
{
	os<<"nrnddnHeader content: "
			<<" NodeID="<<m_sourceId
			<<" coordinate = ("<<m_x<<","<<m_y<<")"
			<<" type="<<m_type
			<<" TTL="<<m_TTL
			<<" preLane=" <<m_preLane
			<<" currentLane=" << m_currentLane << std::endl
			<<" laneList=";
	for(std::vector<std::string>::const_iterator it = m_laneList.begin(); it != m_laneList.end(); ++it)
		os<<(*it)<<" ";
	os<<std::endl;
}

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */
