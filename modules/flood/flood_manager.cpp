#include "modules/flood/flood_manager.h"

#include <algorithm>

FloodManager::FloodManager(
    uint8_t self,
    CommunicationManagerInterface& cm,
    std::function<bool(uint8_t)> base_reachable_fn
) :
    self_id(self),
    communication_manager(cm),
    is_base_reachable(std::move(base_reachable_fn))
{ }

void FloodManager::registerBase(uint8_t base_id) {
    // Allocate a PerBase slot if we've never heard of this base before.
    // Subsequent floods from this base_id are now tracked.
    per_base[base_id];  // default-constructs on miss
}

void FloodManager::setSelfBaseId(uint8_t base_id) {
    self_base_id = base_id;
    // A base manager tracks its own floods locally.
    per_base[base_id];
}

void FloodManager::onPacketReceived(const ::Packet& pkt) {
    if (pkt.payload.empty()) {
        return;
    }

    auto type = static_cast<FloodMsgType>(pkt.payload[0]);

    switch(type) {
        case FloodMsgType::START: {
            FloodStartMsg msg;
            if (decodeStart(pkt, msg)) {
                // Base unicasts START to the initiator drone; pkt.src IS
                // the base's id, which we record so this flood is attributed
                // to the right base.
                self_base_id = pkt.src;
                per_base[self_base_id];  // ensure slot exists
                handleStart(msg);
            }
            break;
        }
        case FloodMsgType::DISCOVERY: {
            FloodDiscoveryMsg msg;
            if (decodeDiscovery(pkt, msg)) {
                handleDiscovery(msg);
            }
            break;
        }
        case FloodMsgType::REPORT: {
            FloodReportMsg msg;
            if (decodeReport(pkt, msg)) {
                handleReport(msg);
            }
            break;
        }
        default:
            return;
    }
}

uint8_t FloodManager::getHopsFromBase(uint8_t base_id) const {
    // Fresh ACK from this base is the authoritative proof that we are
    // within one radio hop of it.  A returning drone that just reached
    // the coverage boundary gets hop=1 via this path even if its stored
    // flood hop is still 2+.
    if (is_base_reachable && is_base_reachable(base_id)) {
        return 1;
    }

    auto it_base = per_base.find(base_id);
    if (it_base == per_base.end()) {
        return UINT8_MAX;
    }
    const auto& pb = it_base->second;

    uint16_t latest_flood_id = 0;
    for (auto [flood_id, hops_to_base] : pb.best_hop_per_flood) {
        (void)hops_to_base;
        latest_flood_id = (latest_flood_id < flood_id) ? flood_id : latest_flood_id;
    }
    auto it = pb.best_hop_per_flood.find(latest_flood_id);
    if (it == pb.best_hop_per_flood.end()) {
        return UINT8_MAX;  // no flood seen yet -- unreachable
    }

    const uint8_t flood_hop = it->second;

    // Stale protection: a stored hop=1 means we were once the initiator for
    // that base's flood but we've since lost coverage (is_base_reachable is
    // false above).  Treat it as unreachable -- anything better would be a
    // false direct-coverage claim.
    if (flood_hop == 1) {
        return UINT8_MAX;
    }

    return flood_hop;
}

uint8_t FloodManager::getNearestBaseId() const {
    uint8_t best_id = UINT8_MAX;
    uint8_t best_hops = UINT8_MAX;
    for (const auto& [base_id, pb] : per_base) {
        (void)pb;
        const uint8_t h = getHopsFromBase(base_id);
        if (h < best_hops) {
            best_hops = h;
            best_id = base_id;
        }
    }
    return best_id;
}

uint8_t FloodManager::getNearestBaseHops() const {
    uint8_t best = UINT8_MAX;
    for (const auto& [base_id, pb] : per_base) {
        (void)pb;
        const uint8_t h = getHopsFromBase(base_id);
        if (h < best) {
            best = h;
        }
    }
    return best;
}

std::vector<std::pair<uint8_t, uint8_t>> FloodManager::getPerBaseHops() const {
    std::vector<std::pair<uint8_t, uint8_t>> out;
    out.reserve(per_base.size());
    for (const auto& [base_id, pb] : per_base) {
        (void)pb;
        const uint8_t h = getHopsFromBase(base_id);
        if (h != UINT8_MAX) {
            out.emplace_back(base_id, h);
        }
    }
    return out;
}

void FloodManager::startFlood(uint16_t flood_id) {
    // Initiator seeds the flood; base_id stamped is whichever base's
    // FloodManager this is OR whichever base the upstream START came from.
    FloodDiscoveryMsg msg;
    msg.base_id = self_base_id;
    msg.flood_id = flood_id;
    msg.initiator_id = self_id;
    msg.hop_to_base = 1;

    auto& pb = per_base[self_base_id];
    pb.seen_floods.insert(flood_id);
    pb.best_hop_per_flood[flood_id] = 1;

    ::Packet pkt;
    pkt.type = ::PacketType::FLOOD;
    pkt.src = self_id;
    pkt.dst = BROADCAST_ID;
    pkt.payload.resize(sizeof(msg));
    std::memcpy(pkt.payload.data(), &msg, sizeof(msg));

    communication_manager.send(pkt);
}

void FloodManager::handleStart(const FloodStartMsg& msg) {
    // Base -> initiator drone.  self_base_id is set by the START path
    // above (drones receive START from the base, pkt.src is the base id).
    //
    // Only re-seed the flood if we actually have direct radio reach to
    // this base; otherwise we'd stamp ourselves as hop=1 and then the
    // stale-protection branch in getHopsFromBase() would clamp us to
    // UINT8_MAX as soon as our ACK freshness lapses.  A drone that has
    // truly returned to coverage will pass the reachability check and
    // re-seed; the genuinely-out-of-coverage case correctly bails.
    if (is_base_reachable && !is_base_reachable(self_base_id)) {
        return;
    }
    auto& pb = per_base[self_base_id];
    if (pb.seen_floods.count(msg.flood_id)) {
        return;
    }
    startFlood(msg.flood_id);
}

void FloodManager::handleDiscovery(const FloodDiscoveryMsg& msg) {
    // Ignore floods for bases we don't know about.
    auto it_base = per_base.find(msg.base_id);
    if (it_base == per_base.end()) {
        return;
    }
    auto& pb = it_base->second;

    const uint16_t flood_id = msg.flood_id;
    const uint8_t initiator_id = msg.initiator_id;

    // Compute candidate hop to the originating base.
    const bool base_reachable = (is_base_reachable && is_base_reachable(msg.base_id));
    const uint8_t candidate_hop = base_reachable
        ? static_cast<uint8_t>(1)
        : static_cast<uint8_t>(msg.hop_to_base + 1);

    bool improved = false;
    auto it = pb.best_hop_per_flood.find(flood_id);
    if (it == pb.best_hop_per_flood.end()) {
        improved = true;
        pb.best_hop_per_flood[flood_id] = candidate_hop;
        pb.seen_floods.insert(flood_id);
    } else if (candidate_hop < it->second) {
        improved = true;
        it->second = candidate_hop;
    }

    if (!improved) {
        return;
    }

    pb.best_report_seen[flood_id][self_id] = candidate_hop;
    ::Packet report_pkt = createReportMsg(msg.base_id, flood_id, initiator_id, candidate_hop);
    communication_manager.send(report_pkt);

    ::Packet flood_pkt = createDiscoveryMsg(msg.base_id, flood_id, initiator_id, candidate_hop);
    communication_manager.send(flood_pkt);
}

void FloodManager::handleReport(const FloodReportMsg& msg) {
    auto it_base = per_base.find(msg.base_id);
    if (it_base == per_base.end()) {
        return;
    }
    auto& pb = it_base->second;

    if (!pb.seen_floods.count(msg.flood_id)) {
        return;
    }

    bool improved = false;
    auto& seen = pb.best_report_seen[msg.flood_id];
    auto it_seen = seen.find(msg.reporter_id);
    if (it_seen == seen.end() || msg.hop_to_base < it_seen->second) {
        improved = true;
        seen[msg.reporter_id] = msg.hop_to_base;
    }

    if (!improved) {
        return;
    }

    ::Packet report_pkt = createReportMsg(msg.base_id, msg.flood_id, msg.initiator_id, msg.hop_to_base);
    communication_manager.send(report_pkt);
}

::Packet FloodManager::createReportMsg(uint8_t base_id, uint16_t flood_id, uint8_t initiator_id, uint8_t candidate_hop) {
    FloodReportMsg report;
    report.base_id = base_id;
    report.flood_id = flood_id;
    report.initiator_id = initiator_id;
    report.reporter_id = self_id;
    report.hop_to_base = candidate_hop;

    ::Packet report_pkt;
    report_pkt.type = ::PacketType::FLOOD;
    report_pkt.src = self_id;
    report_pkt.dst = BROADCAST_ID;
    report_pkt.payload.resize(sizeof(report));
    std::memcpy(report_pkt.payload.data(), &report, sizeof(report));
    return report_pkt;
}

::Packet FloodManager::createDiscoveryMsg(uint8_t base_id, uint16_t flood_id, uint8_t initiator_id, uint8_t hop_to_base) {
    FloodDiscoveryMsg msg;
    msg.base_id = base_id;
    msg.flood_id = flood_id;
    msg.initiator_id = initiator_id;
    msg.hop_to_base = hop_to_base;

    ::Packet pkt;
    pkt.type = ::PacketType::FLOOD;
    pkt.src = self_id;
    pkt.dst = BROADCAST_ID;
    pkt.payload.resize(sizeof(msg));
    std::memcpy(pkt.payload.data(), &msg, sizeof(msg));
    return pkt;
}

bool FloodManager::decodeStart(const ::Packet& pkt, FloodStartMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodStartMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodStartMsg));
    return msg.type == FloodMsgType::START;
}

bool FloodManager::decodeDiscovery(const ::Packet& pkt, FloodDiscoveryMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodDiscoveryMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodDiscoveryMsg));
    return msg.type == FloodMsgType::DISCOVERY;
}

bool FloodManager::decodeReport(const ::Packet& pkt, FloodReportMsg& msg) {
    if (pkt.payload.size() < sizeof(FloodReportMsg)) {
        return false;
    }
    std::memcpy(&msg, pkt.payload.data(), sizeof(FloodReportMsg));
    return msg.type == FloodMsgType::REPORT;
}
