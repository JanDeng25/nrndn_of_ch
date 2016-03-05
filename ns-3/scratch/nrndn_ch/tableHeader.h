/*
 * nrHeader.h
 *
 *  Created on: Jan 15, 2015
 *      Author: chen
 */

#ifndef TABLEHEADER_H_
#define TABLEHEADER_H_

#include "ns3/header.h"
#include "ns3/address-utils.h"
#include "ns3/ndn-name.h"
#include "ns3/ndn-pit-entry.h"
#include "ns3/ndn-fib-entry.h"

#include "ndn-pit-entry-nrimpl.h"
#include "ndn-fib-entry-nrimpl.h"

#include <vector>

namespace ns3
{
namespace ndn
{
namespace nrndn
{

class tableHeader: public Header
{
public:
	tableHeader();
	virtual ~tableHeader();

	///\name Header serialization/deserialization
	//\{
	static TypeId GetTypeId();
	TypeId GetInstanceTypeId() const;
	uint32_t GetSerializedSize() const;
	void Serialize(Buffer::Iterator start) const;
	uint32_t Deserialize(Buffer::Iterator start);
	void Print(std::ostream &os) const;

	uint32_t getSourceId() const
	{
		return m_sourceId;
	}

	void setSourceId(uint32_t sourceId)
	{
		m_sourceId = sourceId;
	}

	std::vector<Ptr<pit::Entry> >	getPitContainer()
	{
		return m_pitContainer;
	}

	std::vector<Ptr<fib::Entry > >	getFibContainer()
	{
		return m_fibContainer;
	}

	void setPIT(std::vector<Ptr<pit::Entry> > pit)
	{
		m_pitContainer = pit;
	}

	void setFIB(std::vector<Ptr<fib::Entry > > fib)
	{
		m_fibContainer = fib;
	}

private:
	uint32_t		m_sourceId;	//\ (source)	id of source node (source)
	 std::vector<Ptr<pit::Entry> >			m_pitContainer;
	std::vector<Ptr<fib::Entry> >		m_fibContainer;

	friend class pit::nrndn::EntryNrImpl;
	friend class fib::nrndn::EntryNrImpl;
};

} /* namespace nrndn */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* TABLEHEADER_H_ */
