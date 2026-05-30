#include "../gap-protocol.h"

namespace ns3
{

GapHeader::GapHeader()
    : m_sourceId(0),
      m_sequence(0),
      m_co2(0),
      m_priority(0),
      m_hopCount(0),
      m_sendTime(0.0)
{
}

GapHeader::GapHeader(uint32_t sourceId,
                     uint32_t sequence,
                     uint16_t co2,
                     uint8_t priority,
                     uint8_t hopCount,
                     double sendTime)
    : m_sourceId(sourceId),
      m_sequence(sequence),
      m_co2(co2),
      m_priority(priority),
      m_hopCount(hopCount),
      m_sendTime(sendTime)
{
}

TypeId
GapHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::GapHeader")
                          .SetParent<Header>()
                          .SetGroupName("Internet")
                          .AddConstructor<GapHeader>();
  return tid;
}

TypeId
GapHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

void
GapHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU32(m_sourceId);
  start.WriteHtonU32(m_sequence);
  start.WriteHtonU16(m_co2);
  start.WriteU8(m_priority);
  start.WriteU8(m_hopCount);

  uint64_t sendTimeMicro = static_cast<uint64_t>(m_sendTime * 1000000.0);
  start.WriteHtonU64(sendTimeMicro);
}

uint32_t
GapHeader::Deserialize(Buffer::Iterator start)
{
  m_sourceId = start.ReadNtohU32();
  m_sequence = start.ReadNtohU32();
  m_co2 = start.ReadNtohU16();
  m_priority = start.ReadU8();
  m_hopCount = start.ReadU8();

  uint64_t sendTimeMicro = start.ReadNtohU64();
  m_sendTime = static_cast<double>(sendTimeMicro) / 1000000.0;

  return GetSerializedSize();
}

uint32_t
GapHeader::GetSerializedSize() const
{
  return 4 + 4 + 2 + 1 + 1 + 8;
}

void
GapHeader::Print(std::ostream &os) const
{
  os << "src=" << m_sourceId
     << " seq=" << m_sequence
     << " co2=" << m_co2
     << " priority=" << unsigned(m_priority)
     << " hops=" << unsigned(m_hopCount)
     << " sendTime=" << m_sendTime;
}

uint32_t
GapHeader::GetSourceId() const
{
  return m_sourceId;
}

uint32_t
GapHeader::GetSequence() const
{
  return m_sequence;
}

uint16_t
GapHeader::GetCo2() const
{
  return m_co2;
}

uint8_t
GapHeader::GetPriority() const
{
  return m_priority;
}

uint8_t
GapHeader::GetHopCount() const
{
  return m_hopCount;
}

double
GapHeader::GetSendTime() const
{
  return m_sendTime;
}

void
GapHeader::IncreaseHopCount()
{
  m_hopCount++;
}

} // namespace ns3
