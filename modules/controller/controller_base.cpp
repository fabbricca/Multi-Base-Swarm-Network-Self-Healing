#include "modules/controller/controller_base.h"

#include <cmath>

ControllerBase::ControllerBase(
    uint8_t self_id,
    float K_att_value,
    float K_rep_value,
    float D_safe,
    float V_max,
    float drone_weight_kg
) :
    self_id(self_id),
    K_att(K_att_value > 0.0 ? K_att_value : DEFAULT_K_ATT),
    K_rep(K_rep_value > 0.0 ? K_rep_value : DEFAULT_K_REP),
    D_safe(D_safe > 0.0 ? D_safe : DEFAULT_D_SAFE),
    V_max(V_max > 0.0 ? V_max : DEFAULT_V_MAX),
    drone_weight_kg(drone_weight_kg > 0.0 ? drone_weight_kg : DEFAULT_DRONE_WEIGHT_KG)
{ }

void ControllerBase::setMissionActive(bool active) {
    mission_active = active;
}

bool ControllerBase::isMissionActive() const {
    return mission_active;
}

void ControllerBase::setReturning(bool returning) {
    m_returning = returning;
}

bool ControllerBase::isReturning() const {
    return m_returning;
}

void ControllerBase::setStationKeeping(bool station_keeping) {
    m_station_keeping = station_keeping;
}

bool ControllerBase::isStationKeeping() const {
    return m_station_keeping;
}

void ControllerBase::setIdleVelocity(const Vector3D& velocity) {
    idle_velocity = velocity;
}

void ControllerBase::computeAttractiveForces(const Vector3D& diff, Vector3D& force) {
    force = force + (K_att * diff);
}

void ControllerBase::computeRepulsiveForces(const Vector3D& diff, Vector3D& force) {
    float distance = diff.module();
    if (distance == 0) {
        return;
    }
    force = force - (K_rep / std::pow(distance, 2)) * diff.unit_vector();
}

void ControllerBase::computeVelocityCommand(const Vector3D& force, Vector3D* new_acceleration) {
    new_acceleration->x = force.x / drone_weight_kg;
    new_acceleration->y = force.y / drone_weight_kg;
    new_acceleration->z = force.z / drone_weight_kg;
}

void ControllerBase::step(
    FloodManagerInterface* flooding_manager,
    VelocityActuatorInterface* velocity_actuator,
    NeighborManagerInterface* neighbor_manager,
    PositionInterface* position
) {
    if (!velocity_actuator) {
        return;
    }
    if (!flooding_manager || !neighbor_manager || !position) {
        return;
    }

    // v2 multi-base: every tick we optimize relative to whichever base is
    // currently nearest.  If no base is reachable, the drone is fully lost
    // -- the per-base hops vector is empty and every branch below degrades
    // gracefully (see below).
    const uint8_t nearest_base = flooding_manager->getNearestBaseId();
    const uint8_t my_hops = flooding_manager->getNearestBaseHops();
    const auto my_per_base_hops = flooding_manager->getPerBaseHops();

    if (m_station_keeping) {
        velocity_actuator->brake();
        position->retrieveCurrentPosition();
        neighbor_manager->sendToNeighbors(self_id, position, my_per_base_hops, /*returning=*/false, /*station_keeping=*/m_station_keeping);
        return;
    }

    if (!mission_active && m_returning) {
        const auto neighbors = neighbor_manager->getNeighbors();
        position->retrieveCurrentPosition();

        Vector3D F_tot{0.0f, 0.0f, 0.0f};
        bool has_attractor = false;
        for (const NeighborInfoInterface* neighbor : neighbors) {
            Vector3D diff = position->distanceFromCoords(neighbor->getPosition());
            // Follow the gradient toward our nearest base using neighbors
            // who claim a lower hop count for THAT base.
            const uint8_t nh = (nearest_base == UINT8_MAX)
                ? neighbor->getMinHopsToAnyBase()
                : neighbor->getHopsToBase(nearest_base);
            const uint8_t mh = (my_hops == UINT8_MAX) ? 0xFE : my_hops;

            if (nh != UINT8_MAX && nh < mh) {
                F_tot = F_tot + (K_att * RETURN_K_ATT_SCALE * diff);
                has_attractor = true;
            }
            if (diff.module() < D_safe) {
                computeRepulsiveForces(diff, F_tot);
            }
        }

        if (!has_attractor) {
            velocity_actuator->brake();
            neighbor_manager->sendToNeighbors(self_id, position, my_per_base_hops, /*returning=*/true, /*station_keeping=*/false);
            return;
        }

        Vector3D new_acceleration(0.0, 0.0, 0.0);
        computeVelocityCommand(F_tot, &new_acceleration);
        velocity_actuator->applyVelocity(new_acceleration, V_max);

        neighbor_manager->sendToNeighbors(self_id, position, my_per_base_hops, /*returning=*/true, /*station_keeping=*/false);
        return;
    }

    if (!mission_active) {
        velocity_actuator->brake();

        // Silence NEIGHBOR only when we're in direct base coverage (hops==1):
        // an in-coverage idle drone plays no role in the formation task.
        const bool is_hop1 = (my_hops == 1);
        if (!is_hop1) {
            position->retrieveCurrentPosition();
            neighbor_manager->sendToNeighbors(self_id, position, my_per_base_hops, /*returning=*/false, /*station_keeping=*/m_station_keeping);
        }
        return;
    }

    const auto neighbors = neighbor_manager->getNeighbors();
    position->retrieveCurrentPosition();

    // Helper auto-deactivation: count outward neighbors (hops > my_hops or
    // genuinely lost) that are NOT yet station-keeping.  Returning drones
    // still count -- they need the relay chain we form during their
    // return.  Once every drone we were helping has reached direct
    // coverage and switched to station-keeping (or aged out of our
    // neighbor list), our relay job is done -- brake and clear
    // mission_active so a future HELP_PROXY can re-arm us.
    //
    // This is the cleanup that lets returned drones truthfully report
    // hops=1: helpers that would otherwise see them as same-hop peers
    // (and skip them, collapsing toward the base) are now off-mission and
    // simply hold position.
    //
    // Skip the check during the startup window where we don't yet have a
    // flood-derived hop count -- otherwise we'd deactivate before the
    // first relay chain even forms.
    if (my_hops != UINT8_MAX) {
        size_t outward_active = 0;
        for (const NeighborInfoInterface* n : neighbors) {
            if (n->getIsStationKeeping()) continue;  // returned and settled
            const uint8_t nh = (nearest_base == UINT8_MAX)
                ? n->getMinHopsToAnyBase()
                : n->getHopsToBase(nearest_base);
            if (nh == UINT8_MAX) {
                // Genuinely lost (no path to any base) -- definitely
                // someone we should still be helping.  A neighbor attached
                // to another base's swarm is excluded.
                if (n->getMinHopsToAnyBase() == UINT8_MAX) ++outward_active;
                continue;
            }
            if (my_hops != UINT8_MAX && nh > my_hops) ++outward_active;
        }
        if (outward_active == 0) {
            if (++m_no_outward_ticks >= HELPER_IDLE_DEACTIVATE_TICKS) {
                mission_active = false;
                m_no_outward_ticks = 0;
                velocity_actuator->brake();
                neighbor_manager->sendToNeighbors(self_id, position, my_per_base_hops, /*returning=*/false, /*station_keeping=*/m_station_keeping);
                return;
            }
        } else {
            m_no_outward_ticks = 0;
        }
    }

    Vector3D F_tot{0.0f, 0.0f, 0.0f};
    accumulateAttractive(neighbors, nearest_base, my_hops, position, F_tot);

    for (const NeighborInfoInterface* neighbor : neighbors) {
        Vector3D diff = position->distanceFromCoords(neighbor->getPosition());
        if (diff.module() < D_safe) {
            computeRepulsiveForces(diff, F_tot);
        }
    }

    if (neighbors.empty() || F_tot.module() < 1e-6f) {
        velocity_actuator->brake();
        neighbor_manager->sendToNeighbors(self_id, position, my_per_base_hops, /*returning=*/false, /*station_keeping=*/m_station_keeping);
        return;
    }

    Vector3D new_acceleration(0.0, 0.0, 0.0);
    computeVelocityCommand(F_tot, &new_acceleration);
    velocity_actuator->applyVelocity(new_acceleration, V_max);

    neighbor_manager->sendToNeighbors(self_id, position, my_per_base_hops, /*returning=*/false, /*station_keeping=*/m_station_keeping);
}
