#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/flow-monitor-module.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OlsrBaselineSimulation");

static const uint16_t OLSR_PORT = 9000;

// Same simulated wireless loss idea used in GAP.
// Here, loss depends on packet severity only for fair channel/application simulation.
// But OLSR itself does NOT use CO2 value for routing.
static const double NORMAL_LOSS_PROBABILITY = 0.08;
static const double HIGH_LOSS_PROBABILITY = 0.05;
static const double CRITICAL_LOSS_PROBABILITY = 0.03;

// ------------------------------
// CO2 packet header for baseline
// ------------------------------
class Co2Header : public Header
{
public:
  Co2Header()
      : m_sourceId(0),
        m_sequence(0),
        m_co2(0),
        m_priority(0),
        m_sendTime(0.0)
  {
  }

  Co2Header(uint32_t sourceId,
            uint32_t sequence,
            uint16_t co2,
            uint8_t priority,
            double sendTime)
      : m_sourceId(sourceId),
        m_sequence(sequence),
        m_co2(co2),
        m_priority(priority),
        m_sendTime(sendTime)
  {
  }

  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("Co2Header")
                            .SetParent<Header>()
                            .AddConstructor<Co2Header>();
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

    uint64_t sendTimeMicro = static_cast<uint64_t>(m_sendTime * 1000000.0);
    start.WriteHtonU64(sendTimeMicro);
  }

  uint32_t Deserialize(Buffer::Iterator start) override
  {
    m_sourceId = start.ReadNtohU32();
    m_sequence = start.ReadNtohU32();
    m_co2 = start.ReadNtohU16();
    m_priority = start.ReadU8();

    uint64_t sendTimeMicro = start.ReadNtohU64();
    m_sendTime = static_cast<double>(sendTimeMicro) / 1000000.0;

    return GetSerializedSize();
  }

  uint32_t GetSerializedSize() const override
  {
    return 4 + 4 + 2 + 1 + 8;
  }

  void Print(std::ostream &os) const override
  {
    os << "src=" << m_sourceId
       << " seq=" << m_sequence
       << " co2=" << m_co2
       << " priority=" << unsigned(m_priority)
       << " sendTime=" << m_sendTime;
  }

  uint32_t GetSourceId() const { return m_sourceId; }
  uint32_t GetSequence() const { return m_sequence; }
  uint16_t GetCo2() const { return m_co2; }
  uint8_t GetPriority() const { return m_priority; }
  double GetSendTime() const { return m_sendTime; }

private:
  uint32_t m_sourceId;
  uint32_t m_sequence;
  uint16_t m_co2;
  uint8_t m_priority;
  double m_sendTime;
};

// ------------------------------
// OLSR sensor application
// ------------------------------
class OlsrSensorApp : public Application
{
public:
  OlsrSensorApp()
      : m_nodeId(0),
        m_gatewayId(0),
        m_isGateway(false),
        m_sequence(0),
        m_socket(nullptr),
        m_packetSize(50)
  {
  }

  void Setup(uint32_t nodeId,
             uint32_t gatewayId,
             bool isGateway,
             Ipv4Address gatewayAddress,
             uint32_t packetSize)
  {
    m_nodeId = nodeId;
    m_gatewayId = gatewayId;
    m_isGateway = isGateway;
    m_gatewayAddress = gatewayAddress;
    m_packetSize = packetSize;
  }

  static uint32_t totalSent;
  static uint32_t totalReceived;
  static uint32_t criticalSent;
  static uint32_t criticalReceived;

  static double totalDelay;
  static double criticalDelay;

private:
  uint32_t m_nodeId;
  uint32_t m_gatewayId;
  bool m_isGateway;
  uint32_t m_sequence;

  Ptr<Socket> m_socket;
  Ipv4Address m_gatewayAddress;
  uint32_t m_packetSize;

  EventId m_sendEvent;

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

    // Same CO2 generation logic as the GAP code
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

  void StartApplication() override
  {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());

    if (m_isGateway)
    {
      InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), OLSR_PORT);
      m_socket->Bind(local);
      m_socket->SetRecvCallback(MakeCallback(&OlsrSensorApp::ReceivePacket, this));
    }
    else
    {
      m_socket->Connect(InetSocketAddress(m_gatewayAddress, OLSR_PORT));
      Simulator::Schedule(Seconds(1.0 + m_nodeId * 0.2),
                          &OlsrSensorApp::GeneratePacket,
                          this);
    }
  }

  void StopApplication() override
  {
    if (m_sendEvent.IsPending())
    {
      Simulator::Cancel(m_sendEvent);
    }

    if (m_socket)
    {
      m_socket->Close();
    }
  }

  void GeneratePacket()
  {
    uint16_t co2 = GenerateCo2Value();
    uint8_t priority = ClassifyPriority(co2);

    // Simulated loss before application sends.
    // This keeps the same random loss concept from GAP,
    // but OLSR still does not use CO2 for routing.
    Ptr<UniformRandomVariable> lossRv = CreateObject<UniformRandomVariable>();

    double lossProbability = NORMAL_LOSS_PROBABILITY;

    if (priority == 2)
    {
      lossProbability = HIGH_LOSS_PROBABILITY;
    }
    else if (priority == 3)
    {
      lossProbability = CRITICAL_LOSS_PROBABILITY;
    }

    totalSent++;

    if (priority == 3)
    {
      criticalSent++;
    }

    if (lossRv->GetValue(0.0, 1.0) < lossProbability)
    {
      NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                            << "s Node " << m_nodeId
                            << " generated CO2=" << co2
                            << " priority=" << unsigned(priority)
                            << " but packet dropped due to simulated wireless loss");
    }
    else
    {
      Co2Header header(m_nodeId,
                       m_sequence,
                       co2,
                       priority,
                       Simulator::Now().GetSeconds());

      uint32_t headerSize = header.GetSerializedSize();
      uint32_t payloadSize = 0;

      if (m_packetSize > headerSize)
      {
        payloadSize = m_packetSize - headerSize;
      }

      Ptr<Packet> packet = Create<Packet>(payloadSize);
      packet->AddHeader(header);

      m_socket->Send(packet);

      NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                            << "s Node " << m_nodeId
                            << " sent packet: CO2=" << co2
                            << " priority=" << unsigned(priority));
    }

    m_sequence++;

    m_sendEvent = Simulator::Schedule(Seconds(5.0),
                                      &OlsrSensorApp::GeneratePacket,
                                      this);
  }

  void ReceivePacket(Ptr<Socket> socket)
  {
    Address from;
    Ptr<Packet> packet = socket->RecvFrom(from);

    Co2Header header;
    packet->RemoveHeader(header);

    totalReceived++;

    double delay = Simulator::Now().GetSeconds() - header.GetSendTime();
    totalDelay += delay;

    if (header.GetPriority() == 3)
    {
      criticalReceived++;
      criticalDelay += delay;
    }

    NS_LOG_UNCOND("Time " << Simulator::Now().GetSeconds()
                          << "s GATEWAY received packet from Node "
                          << header.GetSourceId()
                          << " CO2=" << header.GetCo2()
                          << " priority=" << unsigned(header.GetPriority())
                          << " delay=" << delay << "s");
  }
};

uint32_t OlsrSensorApp::totalSent = 0;
uint32_t OlsrSensorApp::totalReceived = 0;
uint32_t OlsrSensorApp::criticalSent = 0;
uint32_t OlsrSensorApp::criticalReceived = 0;

double OlsrSensorApp::totalDelay = 0.0;
double OlsrSensorApp::criticalDelay = 0.0;

// ------------------------------
// Main simulation
// ------------------------------
int main(int argc, char *argv[])
{
  uint32_t numNodes = 16;
  uint32_t gatewayId = 0;
  double simulationTime = 40.0;
  uint32_t packetSize = 50;

  CommandLine cmd;
  cmd.AddValue("numNodes", "Number of sensor nodes", numNodes);
  cmd.AddValue("simulationTime", "Simulation time", simulationTime);
  cmd.AddValue("packetSize", "Packet size in bytes including header", packetSize);
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

  // Same indoor node positions as GAP
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

  // Install OLSR routing
  OlsrHelper olsr;
  InternetStackHelper internet;
  internet.SetRoutingHelper(olsr);
  internet.Install(nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  Ipv4Address gatewayAddress = interfaces.GetAddress(gatewayId);

  for (uint32_t i = 0; i < numNodes; i++)
  {
    Ptr<OlsrSensorApp> app = CreateObject<OlsrSensorApp>();
    bool isGateway = (i == gatewayId);

    app->Setup(i,
               gatewayId,
               isGateway,
               gatewayAddress,
               packetSize);

    nodes.Get(i)->AddApplication(app);

    app->SetStartTime(Seconds(0.5));
    app->SetStopTime(Seconds(simulationTime));
  }

  Simulator::Stop(Seconds(simulationTime));
  Simulator::Run();

  std::cout << "\n========== OLSR Baseline Results ==========\n";
  std::cout << "Total packets sent: " << OlsrSensorApp::totalSent << std::endl;
  std::cout << "Total packets received at gateway: " << OlsrSensorApp::totalReceived << std::endl;

  if (OlsrSensorApp::totalSent > 0)
  {
    double pdr = (double)OlsrSensorApp::totalReceived / OlsrSensorApp::totalSent * 100.0;
    std::cout << "Packet Delivery Ratio: " << pdr << " %" << std::endl;
  }

  std::cout << "Critical packets sent: " << OlsrSensorApp::criticalSent << std::endl;
  std::cout << "Critical packets received: " << OlsrSensorApp::criticalReceived << std::endl;

  if (OlsrSensorApp::criticalSent > 0)
  {
    double criticalPdr = (double)OlsrSensorApp::criticalReceived / OlsrSensorApp::criticalSent * 100.0;
    std::cout << "Critical Packet Delivery Ratio: " << criticalPdr << " %" << std::endl;
  }

  if (OlsrSensorApp::totalReceived > 0)
  {
    double avgDelay = OlsrSensorApp::totalDelay / OlsrSensorApp::totalReceived;
    std::cout << "Average End-to-End Delay: " << avgDelay << " s" << std::endl;
  }

  if (OlsrSensorApp::criticalReceived > 0)
  {
    double avgCriticalDelay = OlsrSensorApp::criticalDelay / OlsrSensorApp::criticalReceived;
    std::cout << "Average Critical Packet Delay: " << avgCriticalDelay << " s" << std::endl;
  }

  std::cout << "===========================================\n";

  Simulator::Destroy();
  return 0;
}
