#pragma once
#include <cstdint>
#include <vector>

#include "common/vector3D.h"
#include "interfaces/controller.h"
#include "interfaces/neighbor_info.h"

const float DEFAULT_K_ATT = 1;
const float DEFAULT_K_REP = 1;
const float DEFAULT_D_SAFE = 1;
const float DEFAULT_V_MAX = 1;
const float DEFAULT_DRONE_WEIGHT_KG = 2.5;

// Shared skeleton for formation-control strategies.  Subclasses supply only
// the per-neighbor attractive aggregation; everything else (mission-active
// gating, idle-broadcast policy, per-neighbor repulsion, velocity command,
// outbound NEIGHBOR broadcast) lives here.
class ControllerBase : public ControllerInterface {
    public:
        ControllerBase(
            uint8_t self_id,
            float K_att_value,
            float K_rep_value,
            float D_safe,
            float V_max,
            float drone_weight_kg = 2.5f
        );

        void setMissionActive(bool active) override;
        bool isMissionActive() const override;
        void setReturning(bool returning) override;
        bool isReturning() const override;
        void setStationKeeping(bool station_keeping) override;
        bool isStationKeeping() const override;
        void setIdleVelocity(const Vector3D& velocity) override;

        void step(
            FloodManagerInterface* flooding_manager,
            VelocityActuatorInterface* velocity_actuator,
            NeighborManagerInterface* neighbor_manager,
            PositionInterface* position
        ) override final;

    protected:
        // Template-method hook: subclass accumulates attractive forces from
        // the neighbor list into F_tot.  Repulsion and velocity integration
        // are handled by the base class.
        //
        // v2 multi-base: the base class picks the nearest base per tick and
        // passes its id through to the subclass so it queries neighbors
        // against the correct base dimension.  Neighbors that have no known
        // path to `self_base_id` should be skipped (getHopsToBase returns
        // UINT8_MAX).
        virtual void accumulateAttractive(
            const std::vector<NeighborInfoInterface*>& neighbors,
            uint8_t self_base_id,
            uint8_t self_hops,
            PositionInterface* self_position,
            Vector3D& F_tot
        ) = 0;

        void computeAttractiveForces(const Vector3D& diff, Vector3D& force);
        void computeRepulsiveForces(const Vector3D& diff, Vector3D& force);

        const uint8_t self_id;
        const float K_att;
        const float K_rep;
        const float D_safe;
        const float V_max;
        const float drone_weight_kg;

    private:
        void computeVelocityCommand(const Vector3D& force, Vector3D* new_acceleration);

        bool mission_active = false;
        bool m_returning = false;
        bool m_station_keeping = false;
        Vector3D idle_velocity{0.5f, 0.0f, 0.0f};

        static constexpr float RETURN_K_ATT_SCALE = 0.3f;

        // Helper auto-deactivation: count consecutive ticks during which
        // every outward neighbor is either station-keeping (returned and
        // settled) or no longer in the neighbor list.  When this counter
        // crosses HELPER_IDLE_DEACTIVATE_TICKS, the helper sets
        // mission_active=false and brakes -- a future HELP_PROXY will
        // re-arm it via Ns3Drone::startMission().
        uint32_t m_no_outward_ticks = 0;
        static constexpr uint32_t HELPER_IDLE_DEACTIVATE_TICKS = 20;  // ~1 s @ 20 Hz
};
