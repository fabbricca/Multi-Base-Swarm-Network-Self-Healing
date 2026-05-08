#include "modules/neighbor/neighbor_manager.h"

#include <algorithm>

NeighborManager::NeighborManager(
    CommunicationManagerInterface* communication_manager
) :
    m_communication_manager(communication_manager)
{ }

void NeighborManager::onPacketReceived(const ::Packet& pkt) {
    if (pkt.type != ::PacketType::NEIGHBOR) {
        return;
    }
    if (pkt.payload.size() < 3) {
        return;
    }

    // v2 payload layout:
    //   [0]   neighbor_id
    //   [1]   flags
    //   [2]   num_bases
    //   [3..] num_bases * (base_id, hops) pairs, then position coords
    const uint8_t neighbor_id = pkt.payload[0];
    if (neighbor_id != pkt.src) {
        // Basic sanity check: outer header src should match payload id.
        return;
    }

    auto info = std::make_unique<NeighborInfo>(neighbor_id,
                                               std::vector<std::pair<uint8_t, uint8_t>>{},
                                               /*returning=*/false,
                                               /*station_keeping=*/false,
                                               std::vector<double>{});
    try {
        info->deserialize(pkt.payload);
    } catch (const std::exception&) {
        return;
    }

    auto& entry = m_neighbors[neighbor_id];
    entry.info = std::move(info);
    entry.last_seen_call = m_call_count;
}

std::vector<NeighborInfoInterface*> NeighborManager::getNeighbors() const {
    // Evict entries not refreshed within STALE_CALLS ticks.
    for (auto it = m_neighbors.begin(); it != m_neighbors.end(); ) {
        const uint32_t age = m_call_count - it->second.last_seen_call;
        if (age > STALE_CALLS) {
            it = m_neighbors.erase(it);
        } else {
            ++it;
        }
    }

    std::vector<NeighborInfoInterface*> neighbors;
    neighbors.reserve(m_neighbors.size());
    for (const auto& kv : m_neighbors) {
        neighbors.push_back(kv.second.info.get());
    }
    return neighbors;
}

void NeighborManager::sendToNeighbors(
    uint8_t id,
    PositionInterface* position,
    const std::vector<std::pair<uint8_t, uint8_t>>& per_base_hops,
    bool returning,
    bool station_keeping
) {
    if (!m_communication_manager || !position) {
        return;
    }

    const uint32_t this_call = m_call_count++;
    const std::vector<double> coords = position->getCoordinates();

    // Gate: honor the min-interval, then skip sends that bring no new info
    // unless the max-interval heartbeat window has elapsed (see header).
    if (m_has_last_sent) {
        const uint32_t calls_since = this_call - m_last_sent_call;
        if (calls_since < MIN_CALLS_BETWEEN_SENDS) {
            return;
        }
        if (calls_since < MAX_CALLS_BETWEEN_SENDS) {
            double dist_sq = 0.0;
            const size_t n = std::min(coords.size(), m_last_sent_coords.size());
            for (size_t i = 0; i < n; ++i) {
                const double d = coords[i] - m_last_sent_coords[i];
                dist_sq += d * d;
            }
            if (dist_sq < (NEIGHBOR_DELTA_THRESHOLD_M * NEIGHBOR_DELTA_THRESHOLD_M)) {
                return;
            }
        }
    }

    NeighborInfo info(id, per_base_hops, returning, station_keeping, coords);

    ::Packet pkt;
    pkt.type = ::PacketType::NEIGHBOR;
    pkt.src = id;
    pkt.dst = BROADCAST_ID;
    info.serialize(pkt.payload);

    m_communication_manager->send(pkt);

    m_last_sent_call = this_call;
    m_last_sent_coords = coords;
    m_has_last_sent = true;
}
