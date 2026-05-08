#include "modules/controller/weighted_controller.h"

#include <cstdint>

void WeightedController::accumulateAttractive(
    const std::vector<NeighborInfoInterface*>& neighbors,
    uint8_t self_base_id,
    uint8_t self_hops,
    PositionInterface* self_position,
    Vector3D& F_tot
) {
    // Count lower/higher-hop neighbors relative to OUR nearest base so each
    // side can be scaled by the opposing side's cardinality.
    //
    // v2 multi-base: every per-neighbor hop query is against self_base_id so
    // neighbors attached to other bases don't bias the count.  If we have
    // no nearest base (UINT8_MAX), fall back to min-hops-to-any-base so
    // the drone still gets some gradient signal.
    //
    // A neighbor with no path to OUR base: if it has a path to some other
    // base, it belongs to that swarm and we skip it.  If it has no path to
    // ANY base, it is genuinely lost — treat it as higher-hop so it acts
    // as an outward attractor.
    //
    // *** Historical bug (fixed in 76199d4) ***
    // Pre-fix, lost neighbors (UINT8_MAX) were unconditionally skipped.
    // Result: a helper at the coverage boundary whose only outward neighbor
    // was a lost drone would see n_higher = 0; the application loop's
    // cross-product weighting (base contribution scaled by n_higher)
    // produced F_tot ≈ 0; the helper braked at the boundary instead of
    // holding the midpoint -- and the lost drone, with no helper to relay
    // for it, was never saved.  The centroid controller never had this
    // problem because each non-empty hop bucket contributes its own pull
    // regardless of how many other buckets exist.
    // Sentinel-as-higher-hop logic below restores symmetric behavior.
    auto hop_to_our_base = [&](const NeighborInfoInterface* n) -> uint8_t {
        uint8_t h = (self_base_id == UINT8_MAX)
            ? n->getMinHopsToAnyBase()
            : n->getHopsToBase(self_base_id);
        if (h == UINT8_MAX && n->getMinHopsToAnyBase() == UINT8_MAX) {
            // Genuinely lost — sentinel above any real hop count.
            return UINT8_MAX;
        }
        return h;
    };

    uint32_t n_lower = 0, n_higher = 0;
    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t nh = hop_to_our_base(neighbor);
        // UINT8_MAX from a neighbor still attached to another base means
        // "skip"; UINT8_MAX from a fully lost neighbor means "higher hop".
        if (nh == UINT8_MAX && neighbor->getMinHopsToAnyBase() != UINT8_MAX) continue;
        if (nh < self_hops)      ++n_lower;
        else if (nh > self_hops) ++n_higher;
    }

    for (const NeighborInfoInterface* neighbor : neighbors) {
        if (neighbor->getIsReturning()) continue;
        const uint8_t neighbor_hops = hop_to_our_base(neighbor);
        if (neighbor_hops == UINT8_MAX && neighbor->getMinHopsToAnyBase() != UINT8_MAX) continue;
        Vector3D diff = self_position->distanceFromCoords(neighbor->getPosition());
        if (neighbor_hops < self_hops) {
            for (uint32_t i = 0; i < n_higher; ++i) {
                computeAttractiveForces(diff, F_tot);
            }
        } else if (neighbor_hops > self_hops) {
            for (uint32_t i = 0; i < n_lower; ++i) {
                computeAttractiveForces(diff, F_tot);
            }
        }
    }
}
