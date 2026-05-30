#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"

#include <vector>
#include <map>
#include <set>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("GapProtocolSimulation");

static const uint16_t GAP_PORT = 9000;
static const double RADIO_RANGE = 45.0;

// Forwarding delay
static const double MIN_FORWARD_DELAY = 0.005;
static const double MAX_FORWARD_DELAY = 0.030;

// Simulated wireless loss
static const double NORMAL_LOSS_PROBABILITY = 0.08;
static const double HIGH_LOSS_PROBABILITY = 0.05;
static const double CRITICAL_LOSS_PROBABILITY = 0.03;

// ------------------------------
// GAP packet header
// ------------------------------
class GapHeader : public Header
{
public:
  GapHeader()
      : m_sourceId(0),
        m_sequence(0),
        m_co2(0),
        m_priority(0),
        m_hopCount(0),
        m_sendTime(0.0)
  {
  }

  GapHeader(uint32_t sourceId,
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

  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("GapHeader")
                            .SetParent<Header>()
                            .AddConstructor<GapHeader>();
    return tid;
  }

  TypeId GetInstanceTypeId() const override
  {
    return GetTypeId();
  }

  void Serialize(Buffer::Iterator start) const override
  {
    start.WriteHtonU32(m_sourceId);
    start.WriteHtonU32(m_sequence);
    start.WriteHtonU16(m_co2);
    start.WriteU8(m_priority);
    start.WriteU8(m_hopCount);

    uint64_t sendTimeMicro = static_cast<uint64_t>(m_sendTime * 1000000.0);
    start.WriteHtonU64(sendTimeMicro);
  }

  uint32_t Deserialize(Buffer::Iterator start) override
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

  uint32_t GetSerializedSize() const override
  {
    return 4 + 4 + 2 + 1 + 1 + 8;
  }

  void Print(std::ostream &os) const override
  {
    os << "src=" << m_sourceId
       << " seq=" << m_sequence
       << " co2=" << m_co2
       << " priority=" << unsigned(m_priority)
       << " hops=" << unsigned(m_hopCount)
       << " sendTime=" << m_sendTime;
  }

  uint32_t GetSourceId() const { return m_sourceId; }
  uint32_t GetSequence() const { return m_sequence; }
  uint16_t GetCo2() const { return m_co2; }
  uint8_t GetPriority() const { return m_priority; }
  uint8_t GetHopCount() const { return m_hopCount; }
  double GetSendTime() const { return m_sendTime; }

  void IncreaseHopCount()
  {
    m_hopCount++;
  }

private:
  uint32_t m_sourceId;
  uint32_t m_sequence;
  uint16_t m_co2;
  uint8_t m_priority;
  uint8_t m_hopCount;
  double m_sendTime;
};

// ------------------------------
// GAP node application
// ------------------------------
class GapNodeApp : public Application
{
public:
  GapNodeApp()
      : m_nodeId(0),
        m_gatewayId(0),
        m_isGateway(false),
        m_sequence(0),
        m_socket(nullptr)
  {
  }

  void Setup(uint32_t nodeId,
             uint32_t gatewayId,
             bool isGateway,
             NodeContainer *nodes,
             Ipv4InterfaceContainer *interfaces)
  {
    m_nodeId = nodeId;
    m_gatewayId = gatewayId;
    m_isGateway = isGateway;
    m_nodes = nodes;
    m_interfaces = interfaces;
  }

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

  void StartApplication() override
  {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), GAP_PORT);
    m_socket->Bind(local);
    m_socket->SetRecvCallback(MakeCallback(&GapNodeApp::ReceivePacket, this));

    if (!m_isGateway)
    {
      Simulator::Schedule(Seconds(1.0 + m_nodeId * 0.2), &GapNodeApp::GeneratePacket, this);
    }
  }

  void StopApplication() override
  {
    if (m_socket)
    {
      m_socket->Close();
    }
  }

  double DistanceBetween(uint32_t a, uint32_t b)
  {
    Ptr<MobilityModel> ma = m_nodes->Get(a)->GetObject<MobilityModel>();
    Ptr<MobilityModel> mb = m_nodes->Get(b)->GetObject<MobilityModel>();

    Vector pa = ma->GetPosition();
    Vector pb = mb->GetPosition();

    double dx = pa.x - pb.x;
    double dy = pa.y - pb.y;

    return std::sqrt(dx * dx + dy * dy);
  }

  uint8_t ClassifyPriority(uint16_t co2)
  {
    if (co2 >= 2000)
    {
      return 3; // critical
    }
    else if (co2 >= 1200)
    {
      return 2; // high
    }
    else
    {
      return 1; // normal
    }
  }

  uint16_t GenerateCo2Value()
  {
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();

    if (m_nodeId % 5 == 0)
    {
      return uv->GetInteger(2000, 2600); // critical
    }
    else if (m_nodeId % 3 == 0)
    {
      return uv->GetInteger(1200, 1800); // high
    }
    else
    {
      return uv->GetInteger(500, 900); // normal
    }
  }

  void GeneratePacket()
  {
    uint16_t co2 = GenerateCo2Value();
    uint8_t priority = ClassifyPriority(co2);

    GapHeader header(m_nodeId,
                     m_sequence,
                     co2,
                     priority,
                     0,
                     Simulator::Now().GetSeconds());

    Ptr<Packet> packet = Create<Packet>(50);
    packet->AddHeader(header);

    totalSent++;

    if (priority == 3)
    {
      criticalSent++;
    }

    NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                          << "s Node " << m_nodeId
                          << " generated packet: CO2=" << co2
                          << " priority=" << unsigned(priority));

    ForwardPacket(packet, header);

    m_sequence++;

    Simulator::Schedule(Seconds(5.0), &GapNodeApp::GeneratePacket, this);
  }

  void ReceivePacket(Ptr<Socket> socket)
  {
    Address from;
    Ptr<Packet> packet = socket->RecvFrom(from);

    GapHeader header;
    packet->RemoveHeader(header);

    std::pair<uint32_t, uint32_t> packetId(header.GetSourceId(), header.GetSequence());

    if (m_seenPackets.find(packetId) != m_seenPackets.end())
    {
      return;
    }

    m_seenPackets.insert(packetId);

    if (m_isGateway)
    {
      totalReceived++;

      double delay = Simulator::Now().GetSeconds() - header.GetSendTime();
      totalDelay += delay;
      totalHopCount += header.GetHopCount();

      if (header.GetPriority() == 3)
      {
        criticalReceived++;
        criticalDelay += delay;
        criticalHopCount += header.GetHopCount();
      }

      NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                            << "s GATEWAY received packet from Node "
                            << header.GetSourceId()
                            << " CO2=" << header.GetCo2()
                            << " priority=" << unsigned(header.GetPriority())
                            << " hops=" << unsigned(header.GetHopCount())
                            << " delay=" << delay << "s");

      return;
    }

    if (header.GetHopCount() >= 15)
    {
      NS_LOG_UNCOND("Packet dropped at Node " << m_nodeId << " due to hop limit");
      return;
    }

    header.IncreaseHopCount();

    Ptr<Packet> newPacket = Create<Packet>(50);
    newPacket->AddHeader(header);

    ForwardPacket(newPacket, header);
  }

  void DelayedSend(Ptr<Packet> packet, Ipv4Address nextHopAddress)
  {
    m_socket->SendTo(packet, 0, InetSocketAddress(nextHopAddress, GAP_PORT));
  }

  void ForwardPacket(Ptr<Packet> packet, GapHeader header)
  {
    Ptr<UniformRandomVariable> lossRv = CreateObject<UniformRandomVariable>();

    double lossProbability = NORMAL_LOSS_PROBABILITY;

    if (header.GetPriority() == 2)
    {
      lossProbability = HIGH_LOSS_PROBABILITY;
    }
    else if (header.GetPriority() == 3)
    {
      lossProbability = CRITICAL_LOSS_PROBABILITY;
    }

    if (lossRv->GetValue(0.0, 1.0) < lossProbability)
    {
      NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                            << "s Node " << m_nodeId
                            << " dropped packet from Node "
                            << header.GetSourceId()
                            << " due to simulated wireless loss"
                            << " priority=" << unsigned(header.GetPriority()));
      return;
    }

    int bestNeighbor = -1;
    double bestScore = -1e9;

    double currentDistanceToGateway = DistanceBetween(m_nodeId, m_gatewayId);

    for (uint32_t i = 0; i < m_nodes->GetN(); i++)
    {
      if (i == m_nodeId)
      {
        continue;
      }

      double distanceToNeighbor = DistanceBetween(m_nodeId, i);

      if (distanceToNeighbor > RADIO_RANGE)
      {
        continue;
      }

      double neighborDistanceToGateway = DistanceBetween(i, m_gatewayId);
      double progress = currentDistanceToGateway - neighborDistanceToGateway;

      if (progress <= 0 && i != m_gatewayId)
      {
        continue;
      }

      double linkQuality = 1.0 - (distanceToNeighbor / RADIO_RANGE);
      if (linkQuality < 0.0)
      {
        linkQuality = 0.0;
      }

      double progressFactor = 0.0;
      if (currentDistanceToGateway > 0.0)
      {
        progressFactor = progress / currentDistanceToGateway;
      }

      if (progressFactor < 0.0)
      {
        progressFactor = 0.0;
      }

      double energyFactor = 1.0 - (0.01 * i);
      double queueLoad = 0.1 * (i % 4);

      double score = 0.0;

      if (header.GetPriority() == 1)
      {
        score = (0.45 * energyFactor) +
                (0.20 * linkQuality) +
                (0.25 * progressFactor) -
                (0.10 * queueLoad);
      }
      else if (header.GetPriority() == 2)
      {
        score = (0.25 * energyFactor) +
                (0.40 * linkQuality) +
                (0.25 * progressFactor) -
                (0.10 * queueLoad);
      }
      else
      {
        // Critical CO2 packet: prioritize strong link quality and faster progress
        // toward the gateway, while giving less weight to energy saving.
        score = (0.05 * energyFactor) +
                (0.65 * linkQuality) +
                (0.35 * progressFactor) -
                (0.05 * queueLoad);

        // Extra bonus if critical packet can go directly to gateway.
        if (i == m_gatewayId)
        {
          score += 0.75;
        }
      }

      if (score > bestScore)
      {
        bestScore = score;
        bestNeighbor = i;
      }
    }

    if (bestNeighbor == -1)
    {
      NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                            << "s Node " << m_nodeId
                            << " could not find next hop. Packet dropped.");
      return;
    }

    Ipv4Address nextHopAddress = m_interfaces->GetAddress(bestNeighbor);

    NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                          << "s Node " << m_nodeId
                          << " forwards packet from Node "
                          << header.GetSourceId()
                          << " to Node " << bestNeighbor
                          << " priority=" << unsigned(header.GetPriority())
                          << " GAP-score=" << bestScore);

    Ptr<UniformRandomVariable> delayRv = CreateObject<UniformRandomVariable>();

    double delay = 0.0;

    if (header.GetPriority() == 1)
    {
      // Normal CO2 packets can tolerate higher delay.
      delay = delayRv->GetValue(0.020, 0.050);
    }
    else if (header.GetPriority() == 2)
    {
      // High CO2 packets receive medium forwarding delay.
      delay = delayRv->GetValue(0.010, 0.025);
    }
    else
    {
      // Critical CO2 packets receive the fastest forwarding delay.
      delay = delayRv->GetValue(0.001, 0.005);
    }

    Simulator::Schedule(Seconds(delay),
                        &GapNodeApp::DelayedSend,
                        this,
                        packet,
                        nextHopAddress);
  }
};

uint32_t GapNodeApp::totalSent = 0;
uint32_t GapNodeApp::totalReceived = 0;
uint32_t GapNodeApp::criticalSent = 0;
uint32_t GapNodeApp::criticalReceived = 0;

double GapNodeApp::totalDelay = 0.0;
double GapNodeApp::criticalDelay = 0.0;
uint32_t GapNodeApp::totalHopCount = 0;
uint32_t GapNodeApp::criticalHopCount = 0;

// ------------------------------
// Main simulation
// ------------------------------
int main(int argc, char *argv[])
{
  uint32_t numNodes = 16;
  uint32_t gatewayId = 0;
  double simulationTime = 40.0;

  CommandLine cmd;
  cmd.AddValue("numNodes", "Number of sensor nodes", numNodes);
  cmd.AddValue("simulationTime", "Simulation time", simulationTime);
  cmd.Parse(argc, argv);

  NodeContainer nodes;
  nodes.Create(numNodes);

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211g);

  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode", StringValue("ErpOfdmRate6Mbps"),
                               "ControlMode", StringValue("ErpOfdmRate6Mbps"));

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

  // Node 0 = Gateway at center
  positionAlloc->Add(Vector(80.0, 80.0, 0.0));

  // Sensor nodes arranged indoors
  positionAlloc->Add(Vector(40.0, 0.0, 0.0));     // Node 1
  positionAlloc->Add(Vector(85.0, 0.0, 0.0));     // Node 2
  positionAlloc->Add(Vector(120.0, 0.0, 0.0));    // Node 3

  positionAlloc->Add(Vector(40.0, 40.0, 0.0));    // Node 4
  positionAlloc->Add(Vector(80.0, 40.0, 0.0));    // Node 5
  positionAlloc->Add(Vector(120.0, 40.0, 0.0));   // Node 6

  positionAlloc->Add(Vector(40.0, 80.0, 0.0));    // Node 7
  positionAlloc->Add(Vector(85.0, 85.0, 0.0));    // Node 8
  positionAlloc->Add(Vector(120.0, 80.0, 0.0));   // Node 9

  positionAlloc->Add(Vector(40.0, 120.0, 0.0));   // Node 10
  positionAlloc->Add(Vector(80.0, 120.0, 0.0));   // Node 11
  positionAlloc->Add(Vector(120.0, 120.0, 0.0));  // Node 12

  positionAlloc->Add(Vector(160.0, 40.0, 0.0));   // Node 13
  positionAlloc->Add(Vector(160.0, 80.0, 0.0));   // Node 14
  positionAlloc->Add(Vector(160.0, 120.0, 0.0));  // Node 15

  mobility.SetPositionAllocator(positionAlloc);
  mobility.Install(nodes);

  InternetStackHelper internet;
  internet.Install(nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  for (uint32_t i = 0; i < numNodes; i++)
  {
    Ptr<GapNodeApp> app = CreateObject<GapNodeApp>();
    bool isGateway = (i == gatewayId);

    app->Setup(i, gatewayId, isGateway, &nodes, &interfaces);
    nodes.Get(i)->AddApplication(app);

    app->SetStartTime(Seconds(0.5));
    app->SetStopTime(Seconds(simulationTime));
  }

  AnimationInterface anim("gap-animation.xml");
  anim.EnablePacketMetadata(true);

  anim.UpdateNodeDescription(nodes.Get(0), "Gateway");
  anim.UpdateNodeColor(nodes.Get(0), 255, 0, 0);

  for (uint32_t i = 1; i < numNodes; i++)
  {
    anim.UpdateNodeDescription(nodes.Get(i), "CO2 Sensor");
    anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255);
  }

  Simulator::Stop(Seconds(simulationTime));
  Simulator::Run();

  std::cout << "\n========== GAP Simulation Results ==========\n";
  std::cout << "Total packets sent: " << GapNodeApp::totalSent << std::endl;
  std::cout << "Total packets received at gateway: " << GapNodeApp::totalReceived << std::endl;

  if (GapNodeApp::totalSent > 0)
  {
    double pdr = (double)GapNodeApp::totalReceived / GapNodeApp::totalSent * 100.0;
    std::cout << "Packet Delivery Ratio: " << pdr << " %" << std::endl;
  }

  std::cout << "Critical packets sent: " << GapNodeApp::criticalSent << std::endl;
  std::cout << "Critical packets received: " << GapNodeApp::criticalReceived << std::endl;

  if (GapNodeApp::criticalSent > 0)
  {
    double criticalPdr = (double)GapNodeApp::criticalReceived / GapNodeApp::criticalSent * 100.0;
    std::cout << "Critical Packet Delivery Ratio: " << criticalPdr << " %" << std::endl;
  }

  if (GapNodeApp::totalReceived > 0)
  {
    double avgDelay = GapNodeApp::totalDelay / GapNodeApp::totalReceived;
    double avgHopCount = (double)GapNodeApp::totalHopCount / GapNodeApp::totalReceived;

    std::cout << "Average End-to-End Delay: " << avgDelay << " s" << std::endl;
    std::cout << "Average Hop Count: " << avgHopCount << std::endl;
  }

  if (GapNodeApp::criticalReceived > 0)
  {
    double avgCriticalDelay = GapNodeApp::criticalDelay / GapNodeApp::criticalReceived;
    double avgCriticalHopCount = (double)GapNodeApp::criticalHopCount / GapNodeApp::criticalReceived;

    std::cout << "Average Critical Packet Delay: " << avgCriticalDelay << " s" << std::endl;
    std::cout << "Average Critical Hop Count: " << avgCriticalHopCount << std::endl;
  }

  std::cout << "============================================\n";

  Simulator::Destroy();
  return 0;
}
