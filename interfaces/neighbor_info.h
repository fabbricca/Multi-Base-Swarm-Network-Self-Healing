#pragma once
#include <cstdint>
#include <vector>

class NeighborInfoInterface {
    public:
        virtual ~NeighborInfoInterface() = default;
        virtual std::vector<double> getPosition() const = 0;
        virtual bool getIsReturning() const = 0;
        // True when the neighbor has completed its return and is parked
        // inside coverage.  Helpers use this to detect "no one left to
        // help" and stand down.
        virtual bool getIsStationKeeping() const = 0;

        // Per-base hop count.  Returns UINT8_MAX when the neighbor has no
        // known path to that base.  Allows a drone to filter/weight its
        // neighbor set relative to whichever base it is currently using.
        virtual uint8_t getHopsToBase(uint8_t base_id) const = 0;

        // All (base_id, hops) pairs the neighbor knows about.  Used by
        // NeighborManager consumers that need to iterate bases.
        virtual std::vector<std::pair<uint8_t, uint8_t>> getPerBaseHops() const = 0;

        // Convenience: smallest hop count across all known bases, or
        // UINT8_MAX if the neighbor has no base info.  Retained for callers
        // that don't care which base is cheapest.
        virtual uint8_t getMinHopsToAnyBase() const = 0;

        virtual void serialize(std::vector<uint8_t>& out_payload) const = 0;
        virtual void deserialize(const std::vector<uint8_t>& in_payload) = 0;
};
