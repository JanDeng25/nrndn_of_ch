/*
 * ndn-nr-fib-impl.h
 *
 *  Created on: Jan 20, 2015
 *      Author: chen
 */

#ifndef NDN_NR_FIB_IMPL_H_
#define NDN_NR_FIB_IMPL_H_

#include "ns3/ndn-fib.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include "ns3/ndn-interest.h"
#include "ns3/ndn-data.h"
#include "ns3/ndn-forwarding-strategy.h"
#include "ns3/ndn-name.h"


#include "NodeSensor.h"

#include <vector>


namespace ns3
{
namespace ndn
{
namespace fib
{
namespace nrndn
{

/**
 * @ingroup ndn-fib
 * @brief Class implementing Pending Interests Table,
 * 		  with navigation route customize
 */
class NrFibImpl	: public Fib
{
public:
  /**
   * \brief Interface ID
   *
   * \return interface ID
   */
  static TypeId GetTypeId ();

  /**
   * \brief FIB constructor
   */
  NrFibImpl();

  /**
   * \brief Destructor
   */
  virtual ~NrFibImpl();

  // inherited from Fib
  /**
   * \brief Perform longest prefix match
   *
   * \todo Implement exclude filters
   *
   * \param interest Interest packet header
   * \returns If entry found a valid iterator (Ptr<fib::Entry>) will be returned, otherwise End () (==0)
   */
  virtual Ptr<fib::Entry>
  LongestPrefixMatch (const Interest &interest);



  //This prefix is different from the format of data's name
  virtual Ptr<Entry>
  Find (const Name &prefix);

  virtual Ptr<fib::Entry>
  Add (const Name &prefix, Ptr<Face> face, int32_t metric);

  virtual Ptr<fib::Entry>
  Add (const Ptr<const Name> &prefix, Ptr<Face> face, int32_t metric);

  /**
     * @brief Invalidate all FIB entries
     */
  virtual void
  InvalidateAll();

  virtual void
    RemoveFromAll (Ptr<Face> face);



  virtual void
  Remove (const Ptr<const Name> &prefix);

  //abandon
  virtual Ptr<Entry>
  Create (Ptr<const Data> header);

  //replace NrFibImpl::Create
  //bool
  //InitializeNrFibEntry ();


  virtual void
  Print (std::ostream &os) const;

  virtual uint32_t
  GetSize () const;

  virtual Ptr<const fib::Entry>
    Begin () const;

  virtual Ptr<Entry>
  Begin ();

  virtual Ptr<const fib::Entry>
    End () const ;

  virtual Ptr<Entry>
  End ();

  virtual Ptr<const fib::Entry>
    Next (Ptr<const fib::Entry>) const ;

  virtual Ptr<Entry>
  Next (Ptr<Entry>);

  /**
   * This function update the fib using Interest packet
   * not simply add the name into the fib
   * multi entry of fib will be operated
   */
  //bool UpdateFib(std::string lane,Ptr<const Data> data);


  //void laneChange(std::string oldLane, std::string newLane);

  //小锟添加，2015-8-23
  std::string uriConvertToString(std::string str);

  //Jan 10,2016: add a fib entry
   void
   AddFibEntry (const Ptr<const Name> &prefix, std::string lane, uint32_t ttl);

   std::vector<Ptr<Entry> > getFIB(){
	   return m_fibContainer;
   }
protected:
  // inherited from Object class
  virtual void NotifyNewAggregate (); ///< @brief Even when object is aggregated to another Object
  virtual void DoDispose (); ///< @brief Do cleanup
  virtual void DoInitialize(void);



private:
  Time m_cleanInterval;
  Ptr<ForwardingStrategy>		m_forwardingStrategy;
  std::vector<Ptr<Entry> >		m_fibContainer;
  Ptr<ndn::nrndn::NodeSensor>	m_sensor;

  friend class EntryNrImpl;

  //delete by DJ on Dec 27,2015:no need to link to itself
  //Ptr<Fib> m_fib; ///< \brief Link to FIB table(Useless, Just for compatibility)
};


}/* namespace nrndn */
} /* namespace fib */
} /* namespace ndn */
} /* namespace ns3 */

#endif /* NDN_NR_FIB_IMPL_H_ */
