#include "spectrum-channel-manager.h"
#include <ns3/log.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SpectrumChannelManager");

NS_OBJECT_ENSURE_REGISTERED(SpectrumChannelManager);

TypeId
SpectrumChannelManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SpectrumChannelManager")
                            .SetParent<Object>()
                            .SetGroupName("Nr")
                            .AddConstructor<SpectrumChannelManager>();
    return tid;
}

SpectrumChannelManager::SpectrumChannelManager()
{
    NS_LOG_FUNCTION(this);
}

SpectrumChannelManager::~SpectrumChannelManager()
{
    NS_LOG_FUNCTION(this);
    Clear();
}

Ptr<SpectrumChannel>
SpectrumChannelManager::GetChannelForCell(uint16_t cellId, ObjectFactory& channelFactory)
{
    NS_LOG_FUNCTION(this << cellId);
    
    auto it = m_cellChannels.find(cellId);
    if (it != m_cellChannels.end())
    {
        NS_LOG_DEBUG("Returning existing spectrum channel for cell " << cellId);
        return it->second;
    }
    
    // Create new channel for this cell
    Ptr<SpectrumChannel> channel = channelFactory.Create<SpectrumChannel>();
    m_cellChannels[cellId] = channel;
    
    NS_LOG_INFO("Created new spectrum channel " << channel << " for cell " << cellId);
    std::cout << "SIVERT: Created separate spectrum channel " << channel << " for cell " << cellId << std::endl;
    
    return channel;
}

void
SpectrumChannelManager::RemoveCell(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);
    
    auto it = m_cellChannels.find(cellId);
    if (it != m_cellChannels.end())
    {
        m_cellChannels.erase(it);
        NS_LOG_INFO("Removed spectrum channel for cell " << cellId);
    }
}

void
SpectrumChannelManager::Clear()
{
    NS_LOG_FUNCTION(this);
    m_cellChannels.clear();
}

} // namespace ns3
