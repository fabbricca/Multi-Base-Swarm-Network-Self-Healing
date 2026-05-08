#include "modules/controller/centroid_controller.h"

#include <unordered_map>

void CentroidController::accumulateAttractive(
    const std::vector<NeighborInfoInterface*>& neighbors,
    uint8_t self_base_id,
    uint8_t self_hops,
    PositionInterface* self_position,
    Vector3D& F_tot
) {
    // Group different-hop neighbors by hop count (relative to self's
    // nearest base) so each hop level contributes a single attraction
    // toward its centroid.  Skip drones currently returning; collision
    // avoidance for them is handled in ControllerBase::step.
    //
    // v2 multi-base: query each neighbor's hop count relative to the base
    // WE (self) are optimizing against.  If we have no nearest base
    // (UINT8_MAX), fall back to getMinHopsToAnyBase so the drone can still
    // form up around whatever chain exists.
    //
    // A neighbor with no path to OUR base needs care: if it has a path to
    // some other base, it belongs to that base's swarm and we skip it
    // (multi-base partition).  If it has no path to ANY base, it is
    // genuinely lost — bucket it under a sentinel hop above self_hops so it
    // acts as an outward attractor (matches pre-multi-base behavior, which
    // is what creates the helper-stays-at-midpoint equilibrium).
    //
    // Station-keeping neighbors report their TRUE hop count (=1 once back
    // in coverage) but in formation terms they are the outward boundary
    // anchor: we want helpers (also at hops=1) to settle midway between
    // the base and the SK ring, not to skip SK as a same-hop peer.  Force
    // SK neighbors into a sentinel bucket above self_hops to recover the
    // pre-fix equilibrium without resurrecting the v1 hops=2 lie.
    std::unordered_map<uint8_t, std::vector<std::vector<double>>> hop_groups;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        uint8_t nh = (self_base_id == UINT8_MAX)
            ? neighbor->getMinHopsToAnyBase()
            : neighbor->getHopsToBase(self_base_id);
        if (nh == UINT8_MAX) {
            if (neighbor->getMinHopsToAnyBase() != UINT8_MAX) continue;
            nh = UINT8_MAX;  // genuinely lost — sentinel "higher than any real hop"
        }
        if (neighbor->getIsStationKeeping()) {
            // SK boundary anchor.  Use a sentinel just above self_hops so
            // it forms its own bucket (and a separate one from genuinely
            // lost drones at UINT8_MAX, so the centroids don't merge).
            const uint8_t sk_sentinel = (self_hops < 254) ? static_cast<uint8_t>(self_hops + 1) : 254;
            hop_groups[sk_sentinel].push_back(neighbor->getPosition());
            continue;
        }
        if (nh != self_hops) {
            hop_groups[nh].push_back(neighbor->getPosition());
        }
    }

    for (const auto& [hop, positions] : hop_groups) {
        (void)hop;
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (const auto& p : positions) {
            cx += p[0];
            cy += (p.size() > 1 ? p[1] : 0.0);
            cz += (p.size() > 2 ? p[2] : 0.0);
        }
        const double n = static_cast<double>(positions.size());
        std::vector<double> centroid = {cx / n, cy / n, cz / n};
        Vector3D diff = self_position->distanceFromCoords(centroid);
        computeAttractiveForces(diff, F_tot);
    }
}
