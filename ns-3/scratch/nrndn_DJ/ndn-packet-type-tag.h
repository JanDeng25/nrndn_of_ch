
#ifndef NRNDN_PACKET_TYPE_TAG_H
#define NRNDN_PACKET_TYPE_TAG_H

#include "ns3/tag.h"

namespace ns3 {
namespace ndn {
namespace nrndn {
/**
 * @ingroup ndn-fw
 * @brief Packet tag that is used to track hop count for Interest-Data pairs
 */
class PacketTypeTag : public Tag
{
public:
  static TypeId
  GetTypeId (void);

  /**
   * @brief Default constructor
   */
  PacketTypeTag  () : m_packetTpye (0) { };

  PacketTypeTag  (uint32_t t) : m_packetTpye (t) { };

  /**
   * @brief Destructor
   */
  ~PacketTypeTag  () { }

  /**
   * @brief Increment hop count
   */
  void
  Increment () { m_packetTpye ++; }

  /**
   * @brief Get value of hop count
   */
  uint32_t Get () const { return m_packetTpye; }

  ////////////////////////////////////////////////////////
  // from ObjectBase
  ////////////////////////////////////////////////////////
  virtual TypeId
  GetInstanceTypeId () const;

  ////////////////////////////////////////////////////////
  // from Tag
  ////////////////////////////////////////////////////////

  virtual uint32_t
  GetSerializedSize () const;

  virtual void
  Serialize (TagBuffer i) const;

  virtual void
  Deserialize (TagBuffer i);

  virtual void
  Print (std::ostream &os) const;

private:
  uint32_t m_packetTpye;

};

}
} // namespace ndn
} // namespace ns3

#endif // NRNDN_PACKET_TYPE_TAG_H
