#pragma once
#include <cstdint>
#include <vector>
#include <cstring>
#include <stdexcept>

#include "interfaces/neighbor_info.h"

enum class NeighborMsgType : uint8_t {
    NEIGHBOR_INFO = 10,
};

class NeighborInfo : public NeighborInfoInterface {
    public:
        NeighborInfo(
            uint8_t id,
            const std::vector<std::pair<uint8_t, uint8_t>>& per_base_hops,
            bool returning,
            bool station_keeping,
            const std::vector<double>& coordinates
        );

        std::vector<double> getPosition() const override;
        bool getIsReturning() const override;
        bool getIsStationKeeping() const override;
        uint8_t getHopsToBase(uint8_t base_id) const override;
        std::vector<std::pair<uint8_t, uint8_t>> getPerBaseHops() const override;
        uint8_t getMinHopsToAnyBase() const override;

        void serialize(std::vector<uint8_t>& out_payload) const override;
        void deserialize(const std::vector<uint8_t>& in_payload) override;

        // Payload bit layout for the flags byte.
        static constexpr uint8_t FLAG_RETURNING       = 0x01;
        static constexpr uint8_t FLAG_STATION_KEEPING = 0x02;

    private:
        uint8_t neighbor_id;
        bool is_returning;
        bool is_station_keeping;
        // Per-base hop list: each pair is (base_id, hops).  UINT8_MAX hops
        // means the sender has no known path to that base.
        std::vector<std::pair<uint8_t, uint8_t>> per_base;
        std::vector<double> position;
};
