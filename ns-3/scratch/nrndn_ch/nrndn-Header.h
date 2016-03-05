/*
 * nrHeader.h
 *
 *  Created on: Jan 15, 2015
 *      Author: chen
 */

#ifndef NRNDNHEADER_H_
#define NRNDNHEADER_H_

#include "ns3/header.h"
#include "ns3/address-utils.h"
#include "ns3/ndn-name.h"


#include <vector>

namespace ns3
{
namespace ndn
{
namespace nrndn
{

class nrndnHeader: public Header
{
public:
	nrndnHeader();
	virtual ~nrndnHeader();

	///\name Header serialization/deserialization
	//\{
	static TypeId GetTypeId();
	TypeId GetInstanceTypeId() const;
	uint32_t GetSerializedSize() const;
	void Serialize(Buffer::Iterator start) const;
	uint32_t Deserialize(Buffer::Iterator start);
	void Print(std::ostream &os) const;

	//\}

	///\name Fields
	//\{

	uint32_t getSourceId() const
	{
		return m_sourceId;
	}

	void setSourceId(uint32_t sourceId)
	{
		m_sourceId = sourceId;
	}

	double getX() const
	{
		return m_x;
	}

	void setX(double x)
	{
		m_x=x;
	}

	double getY() const
	{
		return m_y;
	}

	void setY(double y)
	{
		m_y = y;
	}

	void setPreLane(std::string lane)
	{
		m_preLane = lane;
	}

	std::string getPreLane ()
	{
		return m_preLane;
	}

	void setCurrentLane(std::string lane)
	{
		m_currentLane = lane;
	}

	std::string getCurrentLane ()
	{
		return m_currentLane;
	}

	void setLaneList(std::vector<std::string> laneList)
	{
		m_laneList = laneList;
	}

	std::vector<std::string> getLaneList ()
	{
		return m_laneList;
	}

	void setType(uint32_t type)
	{
		m_type = type;
	}

	uint32_t getTpye ()
	{
		return m_type;
	}

	void setTTL(uint32_t ttl)
	{
			m_TTL= ttl;
	}

	uint32_t getTTL ()
	{
			return m_TTL;
	}

	//\}

private:
	uint32_t		m_sourceId;	//\ (source)	id of source node (source)
	double		m_x;		//\ (forwarder)	forwarder x coordinate, not source node position!!!!
	double 		m_y;    	//\ (forwarder)	forwarder y coordinate, not source node position!!!!
	uint32_t        m_type;
	uint32_t        m_TTL;
	std::string             m_preLane;
	std::string             m_currentLane;
	std::vector<std::string>   m_laneList;//

};

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* NRNDNHEADER_H_ */
