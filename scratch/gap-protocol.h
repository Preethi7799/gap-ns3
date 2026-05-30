#ifndef GAP_PROTOCOL_H
#define GAP_PROTOCOL_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"

#include <set>
#include <utility>

namespace ns3
{

static const uint16_t GAP_PORT = 9000;
static const double RADIO_RANGE = 45.0;

// Simulated wireless loss
static const double NORMAL_LOSS_PROBABILITY = 0.08;
static const double HIGH_LOSS_PROBABILITY = 0.05;
static const double CRITICAL_LOSS_PROBABILITY = 0.03;

// --------------------------------------------------
// GAP Packet Header
// --------------------------------------------------
class GapHeader : public Header
{
public:
  GapHeader();

  GapHeader(uint32_t sourceId,
            uint32_t sequence,
            uint16_t co2,
            uint8_t priority,
            uint8_t hopCount,
            double sendTime);

  static TypeId GetTypeId();

  TypeId GetInstanceTypeId() const override;

  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  uint32_t GetSerializedSize() const override;
  void Print(std::ostream &os) const override;

  uint32_t GetSourceId() const;
  uint32_t GetSequence() const;
  uint16_t GetCo2() const;
  uint8_t GetPriority() const;
  uint8_t GetHopCount() const;
  double GetSendTime() const;

  void IncreaseHopCount();

private:
  uint32_t m_sourceId;
  uint32_t m_sequence;
  uint16_t m_co2;
  uint8_t m_priority;
  uint8_t m_hopCount;
  double m_sendTime;
};

// --------------------------------------------------
// GAP Node Application
// --------------------------------------------------
class GapNodeApp : public Application
{
public:
  GapNodeApp();

  void Setup(uint32_t nodeId,
             uint32_t gatewayId,
             bool isGateway,
             NodeContainer *nodes,
             Ipv4InterfaceContainer *interfaces);

  static uint32_t totalSent;
  static uint32_t totalReceived;
  static uint32_t criticalSent;
  static uint32_t criticalReceived;

  static double totalDelay;
  static double criticalDelay;
  static uint32_t totalHopCount;
  static uint32_t criticalHopCount;

private:
  uint32_t m_nodeId;
  uint32_t m_gatewayId;
  bool m_isGateway;
  uint32_t m_sequence;

  Ptr<Socket> m_socket;
  NodeContainer *m_nodes;
  Ipv4InterfaceContainer *m_interfaces;

  std::set<std::pair<uint32_t, uint32_t>> m_seenPackets;

  void StartApplication() override;
  void StopApplication() override;

  double DistanceBetween(uint32_t a, uint32_t b);

  uint8_t ClassifyPriority(uint16_t co2);
  uint16_t GenerateCo2Value();

  void GeneratePacket();
  void ReceivePacket(Ptr<Socket> socket);

  void DelayedSend(Ptr<Packet> packet, Ipv4Address nextHopAddress);
  void ForwardPacket(Ptr<Packet> packet, GapHeader header);
};

class GapHelper
{
public:
  GapHelper();

  void SetGateway(uint32_t gatewayId);

  ApplicationContainer Install(NodeContainer *nodes,
                               Ipv4InterfaceContainer *interfaces) const;

private:
  uint32_t m_gatewayId;
};
} // namespace ns3

#endif // GAP_PROTOCOL_H
