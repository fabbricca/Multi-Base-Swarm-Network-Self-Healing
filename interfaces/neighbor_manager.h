#pragma once
#include <cstdint>
#include <vector>
#include <utility>

#include "interfaces/neighbor_info.h"
#include "interfaces/communication_manager.h"
#include "interfaces/position.h"

class NeighborManagerInterface {
    public:
        virtual ~NeighborManagerInterface() = default;
        virtual void onPacketReceived(const ::Packet& pkt) = 0;
        virtual std::vector<NeighborInfoInterface*> getNeighbors() const = 0;

        // v2 roaming: broadcast our hop count for every base we currently
        // have a path to.  The caller assembles the (base_id, hops) list
        // from the FloodManager and passes it in.
        virtual void sendToNeighbors(
            uint8_t id,
            PositionInterface* position,
            const std::vector<std::pair<uint8_t, uint8_t>>& per_base_hops,
            bool returning,
            bool station_keeping
        ) = 0;
};
