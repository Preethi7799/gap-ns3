#include "../gap-protocol.h"

namespace ns3
{

/*
 * GAP Helper
 * ----------
 * This helper installs GapNodeApp on all nodes in the given NodeContainer.
 *
 * Note:
 * This helper assumes that the GapHelper class declaration is added
 * to gap-protocol.h.
 */

GapHelper::GapHelper()
    : m_gatewayId(0)
{
}

void
GapHelper::SetGateway(uint32_t gatewayId)
{
  m_gatewayId = gatewayId;
}

ApplicationContainer
GapHelper::Install(NodeContainer *nodes,
                   Ipv4InterfaceContainer *interfaces) const
{
  ApplicationContainer apps;

  for (uint32_t i = 0; i < nodes->GetN(); ++i)
  {
    Ptr<GapNodeApp> app = CreateObject<GapNodeApp>();

    bool isGateway = (i == m_gatewayId);

    app->Setup(i,
               m_gatewayId,
               isGateway,
               nodes,
               interfaces);

    nodes->Get(i)->AddApplication(app);
    apps.Add(app);
  }

  return apps;
}

} // namespace ns3
