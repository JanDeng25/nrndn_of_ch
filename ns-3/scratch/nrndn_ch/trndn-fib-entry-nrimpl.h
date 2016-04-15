/*
 * ndn-fib-entry-nrimpl.h
 *
 *  Created on: Dec 24, 2015
 *      Author: DJ
 */


#ifndef NDN_FIB_ENTRY_TRIMPL_H_
#define NDN_FIB_ENTRY_TRIMPL_H_

#include "ns3/ndn-fib-entry.h"
#include "ns3/ndn-fib.h"
//#include "ns3/ndn-fib-entry-incoming-face.h"

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace ns3 {
namespace ndn {

//class Fib;

namespace fib {
namespace nrndn{
	
	/**
	 * @ingroup ndn-fib
	 * @brief FIB entry implementation with additional pointers to the underlying container
	 *           It is completely unrelated to the class FIB::Entry
	 */

class TrEntryNrImpl : public Entry
{
public:
	typedef Entry super;

	TrEntryNrImpl(Ptr<Fib> fib, const Ptr<const NameComponents> &prefix, Time cleanInterval);
	virtual ~TrEntryNrImpl();
	
	/**
	 * @brief Remove all the timeout event in the timeout event list
	 *
	 */
	//void RemoveEntry();

	/**
	 * @brief Add  to the list of incoming neighbor list(m_incomingnbs)
	 *
	 * @param lane  last hop
	 * @param ttl   ttl of data packet
	 * @returns iterator to the added last lane
	 */
	std::unordered_map<uint32_t,uint32_t  >::iterator
	AddIncomingNeighbors(uint32_t nexthop,uint32_t ttl);

	const std::unordered_map<uint32_t,uint32_t>& getIncomingnbs() const
	{
		return m_incomingnbs;
	}

	std::string getEntryName(){
		return m_data_name;

	}
	//Mar 17,2016: get pointer of prefix
	const Ptr<const Name> & GetPrefixPtr(){
		return m_prefix;
	}
	void Print(std::ostream &os) const;
	void setDataName(std::string name);
	void setNb(std::unordered_map< uint32_t,uint32_t >  nb);
	bool isSameLane(std::string lane1, std::string lane2);
//private:
	//void AddNeighborTimeoutEvent(uint32_t id);

	//when the time expire, the incoming neighbor id will be removed automatically
	//void CleanExpiredIncomingNeighbors(uint32_t id);
private:
	//std::unordered_map< uint32_t,EventId> m_nbTimeoutEvent;///< @brief it is a hashmap that record the timeout event of each neighbor id
	std::unordered_map< uint32_t,uint32_t > 		  m_incomingnbs;///< @brief container for incoming neighbors
	std::string m_data_name;
	Time m_infaceTimeout;

};



} // namespace nrndn
} // namespace fib
} // namespace ndn
} // namespace ns3


#endif /* NDN_FIB_ENTRY_NRIMPL_H_ */
