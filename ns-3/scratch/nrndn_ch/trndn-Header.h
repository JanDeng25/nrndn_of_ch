/*
 * nrHeader.h
 *
 *  Created on: Jan 15, 2015
 *      Author: chen
 */

#ifndef TRNDNHEADER_H_
#define TRNDNHEADER_H_

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

class trndnHeader: public Header
{
public:
	trndnHeader();
	virtual ~trndnHeader();

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

	uint32_t getNextId()
	{
		return m_nextid;
	}

	void setNextId(uint32_t id)
	{
		m_nextid = id;
	}

	void setCurrentId(uint32_t id)
	{
		m_currentid = id;
	}

	uint32_t getCurrentId()
	{
		return m_currentid;
	}

	std::vector<uint32_t> getIdList()
	{
		return m_idlist;
	}

	void setIdList ( std::vector<uint32_t> list)
	{
		m_idlist = list;
	}

	//\}

private:
	uint32_t		m_sourceId;	//\ (source)	id of source node (source)
	uint32_t        m_currentid;
	uint32_t		m_nextid;
	std::vector<uint32_t>       m_idlist;
	double		m_x;		//\ (forwarder)	forwarder x coordinate, not source node position!!!!
	double 		m_y;    	//\ (forwarder)	forwarder y coordinate, not source node position!!!!
	uint32_t        m_type;
	uint32_t        m_TTL;

};

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* NRNDNHEADER_H_ */
