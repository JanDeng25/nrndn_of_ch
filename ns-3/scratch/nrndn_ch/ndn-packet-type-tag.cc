
#include "ndn-packet-type-tag.h"

namespace ns3 {
namespace ndn {
namespace nrndn {

TypeId
PacketTypeTag::GetTypeId ()
{
  static TypeId tid = TypeId("ns3::ndn::nrndn::PacketTypeTag")
    .SetParent<Tag>()
    .AddConstructor<PacketTypeTag>()
    ;
  return tid;
}

TypeId
PacketTypeTag::GetInstanceTypeId () const
{
  return PacketTypeTag::GetTypeId ();
}

uint32_t
PacketTypeTag::GetSerializedSize () const
{
  return sizeof(uint32_t);
}

void
PacketTypeTag::Serialize (TagBuffer i) const
{
  i.WriteU32 (m_packetTpye);
}

void
PacketTypeTag::Deserialize (TagBuffer i)
{
	m_packetTpye = i.ReadU32 ();
	//this->Print(std::cout);
}

void
PacketTypeTag::Print (std::ostream &os) const
{
  os << m_packetTpye;
}

}
} // namespace ndn
} // namespace ns3
