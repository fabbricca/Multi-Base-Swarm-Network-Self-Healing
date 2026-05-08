#include "modules/neighbor/neighbor_info.h"

#include <algorithm>

NeighborInfo::NeighborInfo(
    uint8_t id,
    const std::vector<std::pair<uint8_t, uint8_t>>& per_base_hops,
    bool returning,
    bool station_keeping,
    const std::vector<double>& coordinates
) :
    neighbor_id(id),
    is_returning(returning),
    is_station_keeping(station_keeping),
    per_base(per_base_hops),
    position(coordinates)
{ }

std::vector<double> NeighborInfo::getPosition() const {
    return position;
}

bool NeighborInfo::getIsReturning() const {
    return is_returning;
}

bool NeighborInfo::getIsStationKeeping() const {
    return is_station_keeping;
}

uint8_t NeighborInfo::getHopsToBase(uint8_t base_id) const {
    for (const auto& [id, hops] : per_base) {
        if (id == base_id) {
            return hops;
        }
    }
    return UINT8_MAX;
}

std::vector<std::pair<uint8_t, uint8_t>> NeighborInfo::getPerBaseHops() const {
    return per_base;
}

uint8_t NeighborInfo::getMinHopsToAnyBase() const {
    uint8_t best = UINT8_MAX;
    for (const auto& [id, hops] : per_base) {
        (void)id;
        if (hops < best) {
            best = hops;
        }
    }
    return best;
}

// Payload layout:
//   [0]        neighbor_id
//   [1]        flags
//   [2]        num_bases (N)
//   [3..3+2N-1] N * (base_id, hops)
//   [3+2N..]   position coords (doubles, little-endian native)
//
// num_bases is capped at 3 (matches MAX_BASES).  Position is last so its
// size is derived as (payload_size - 3 - 2N) / sizeof(double).
void NeighborInfo::serialize(std::vector<uint8_t>& out_payload) const {
    const size_t n = per_base.size();
    const size_t coord_bytes = position.size() * sizeof(double);
    out_payload.resize(3 + 2 * n + coord_bytes);

    out_payload[0] = neighbor_id;
    uint8_t flags = 0;
    if (is_returning)       flags |= FLAG_RETURNING;
    if (is_station_keeping) flags |= FLAG_STATION_KEEPING;
    out_payload[1] = flags;
    out_payload[2] = static_cast<uint8_t>(n);

    size_t off = 3;
    for (const auto& [id, hops] : per_base) {
        out_payload[off++] = id;
        out_payload[off++] = hops;
    }

    if (coord_bytes > 0) {
        std::memcpy(&out_payload[off], position.data(), coord_bytes);
    }
}

void NeighborInfo::deserialize(const std::vector<uint8_t>& in_payload) {
    if (in_payload.size() < 3) {
        throw std::invalid_argument("Payload too small to deserialize NeighborInfo");
    }

    neighbor_id = in_payload[0];
    is_returning       = (in_payload[1] & FLAG_RETURNING) != 0;
    is_station_keeping = (in_payload[1] & FLAG_STATION_KEEPING) != 0;
    const size_t n = in_payload[2];

    const size_t header = 3 + 2 * n;
    if (in_payload.size() < header) {
        throw std::invalid_argument("NeighborInfo payload truncated");
    }

    per_base.clear();
    per_base.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const uint8_t id   = in_payload[3 + 2 * i];
        const uint8_t hops = in_payload[3 + 2 * i + 1];
        per_base.emplace_back(id, hops);
    }

    const size_t coord_bytes = in_payload.size() - header;
    if (coord_bytes % sizeof(double) != 0) {
        throw std::invalid_argument("NeighborInfo coord section size not a multiple of double");
    }
    position.resize(coord_bytes / sizeof(double));
    if (coord_bytes > 0) {
        std::memcpy(position.data(), &in_payload[header], coord_bytes);
    }
}
