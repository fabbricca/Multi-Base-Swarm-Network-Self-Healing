#include "platform/ns3/drone/ns3_drone.h"

#include <algorithm>
#include <cmath>

Ns3Drone::Ns3Drone(
  uint8_t id,
  ::ns3::Ptr<::ns3::Node> node,
  ControllerAlgorithm algorithm,
  float k_att,
  float k_rep,
  float d_safe,
  float v_max,
  float drone_weight_kg,
  double uwb_noise_std_dev_m
) :
  m_id(id),
  m_node(node),
  m_comm(std::make_unique<sim::UwbTransport>(node, id), id),
  m_controller(makeController(algorithm, id, k_att, k_rep, d_safe, v_max, drone_weight_kg))
{
  if (!m_node) {
    return;
  }

  auto mobility = m_node->GetObject<::ns3::ConstantPositionMobilityModel>();
  if (!mobility) {
    mobility = ::ns3::CreateObject<::ns3::ConstantPositionMobilityModel>();
    m_node->AggregateObject(mobility);
  }

  m_custom_mobility = std::make_unique<CustomMobility>(mobility);
  m_velocity_actuator = std::make_unique<Ns3VelocityActuator>(m_custom_mobility.get());

  m_comm.setReceiveHandler([this](const ::Packet& pkt) { dispatchPacket(pkt); });

  m_flood_manager = std::make_unique<FloodManager>(
      m_id, m_comm,
      [this](uint8_t base_id) { return isBaseReachableId(base_id); });
  m_neighbor_manager = std::make_unique<NeighborManager>(&m_comm);
  m_uwb_ranging_manager = std::make_unique<UwbRangingManager>(
    []() { return ::ns3::Simulator::Now().GetSeconds(); },
    299792458.0, uwb_noise_std_dev_m);
  m_uwb_position = std::make_unique<UwbPosition>(m_uwb_ranging_manager.get());

  const ::ns3::Vector spawn = mobility->GetPosition();
  m_uwb_ranging_manager->seedEstimatedPosition(spawn.x, spawn.y, 0.0);
  m_uwb_position->retrieveCurrentPosition();

  m_dispatcher.setFloodManager(m_flood_manager.get());
  m_dispatcher.setNeighborManager(m_neighbor_manager.get());
  m_dispatcher.setUwbRangingManager(m_uwb_ranging_manager.get());
  m_dispatcher.setFallbackHandler([this](const ::Packet& pkt) { handleCorePacket(pkt); });

  m_tick_phase_s = 0.01 * static_cast<double>(m_id);
}

void Ns3Drone::registerBase(uint8_t base_id) {
  if (std::find(m_known_bases.begin(), m_known_bases.end(), base_id) != m_known_bases.end()) {
    return;  // already registered
  }
  m_known_bases.push_back(base_id);

  // Do NOT seed last_ack_rx_s with the current time.  Seeding would make
  // isBaseReachable() return true before any real ACK has arrived, which in
  // turn causes FloodManager::handleDiscovery to promote every incoming
  // flood to hop=1 (it assumes "I already have direct coverage").  Start at
  // a very old time so isBaseReachable is false until a genuine ACK lands.
  DroneBaseState st;
  st.last_ack_rx_s = -1e9;
  m_base_state[base_id] = st;

  m_comm.registerPeer(base_id, 0);
  if (m_flood_manager) {
    m_flood_manager->registerBase(base_id);
  }
}

void Ns3Drone::start() {
  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_phase_s), ::ns3::MakeCallback(&Ns3Drone::onTick, this));
}

void Ns3Drone::setRepositionLogger(const std::shared_ptr<std::ofstream>& csv) {
  m_reposition_csv = csv;
}

void Ns3Drone::startMission() {
  if (!m_flood_manager || !m_velocity_actuator || !m_neighbor_manager || !m_uwb_position) {
    return;
  }

  m_controller->setMissionActive(true);

  m_mission_start_s = ::ns3::Simulator::Now().GetSeconds();
  m_last_mission_log_s = -1.0;

  std::cout << "[Mission] t=" << m_mission_start_s << "s drone=" << static_cast<int>(m_id)
            << " mission_active=1" << std::endl;
}

void Ns3Drone::stopMission() {
  m_controller->setMissionActive(false);
}

void Ns3Drone::kill() {
  if (!m_alive) return;
  m_alive = false;
  m_killed_at_s = ::ns3::Simulator::Now().GetSeconds();

  if (auto mob = m_node ? m_node->GetObject<::ns3::ConstantPositionMobilityModel>() : nullptr) {
    const ::ns3::Vector p = mob->GetPosition();
    m_prev_gt_x = p.x;
    m_prev_gt_y = p.y;
    m_prev_gt_z = p.z;
  }

  if (m_velocity_actuator) {
    m_velocity_actuator->brake();
  }
  if (m_controller) {
    m_controller->setMissionActive(false);
    m_controller->setReturning(false);
    m_controller->setStationKeeping(false);
  }

  std::cout << "[KILL] t=" << m_killed_at_s << "s drone=" << static_cast<int>(m_id)
            << " pos=(" << m_prev_gt_x << "," << m_prev_gt_y << "," << m_prev_gt_z
            << ")" << std::endl;
}

void Ns3Drone::onTick() {
  if (!m_alive) {
    return;
  }
  const double now_s = ::ns3::Simulator::Now().GetSeconds();

  if (auto mob = m_node ? m_node->GetObject<::ns3::ConstantPositionMobilityModel>() : nullptr) {
    const ::ns3::Vector p = mob->GetPosition();
    if (m_has_prev_gt_pos) {
      const double dx = p.x - m_prev_gt_x;
      const double dy = p.y - m_prev_gt_y;
      const double dz = p.z - m_prev_gt_z;
      const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
      m_total_distance_m += step;
      if (m_returning) {
        m_return_phase_distance_m += step;
      }
    }
    m_prev_gt_x = p.x;
    m_prev_gt_y = p.y;
    m_prev_gt_z = p.z;
    m_has_prev_gt_pos = true;

    if (m_station_keeping) {
      const double dxr = p.x - m_station_keeping_ref_pos_gt.x;
      const double dyr = p.y - m_station_keeping_ref_pos_gt.y;
      const double dzr = p.z - m_station_keeping_ref_pos_gt.z;
      const double drift = std::sqrt(dxr * dxr + dyr * dyr + dzr * dzr);
      if (drift > m_station_keeping_max_drift_m) {
        m_station_keeping_max_drift_m = drift;
      }
    }
  }

  // Waiting-ack check: if we sent a POS_UPDATE to some base and haven't
  // heard back within m_ack_timeout_s from ANY base, treat it as a loss.
  bool any_waiting_stale = false;
  for (const auto& [base_id, st] : m_base_state) {
    (void)base_id;
    if (st.waiting_ack && (now_s - st.last_ack_rx_s) > m_ack_timeout_s) {
      any_waiting_stale = true;
      break;
    }
  }
  if (any_waiting_stale && !help_proxy_sent && !m_controller->isMissionActive()) {
    sendHelpProxy();
    // Clear waiting_ack across all bases (we've given up this round).
    for (auto& [base_id, st] : m_base_state) {
      (void)base_id;
      st.waiting_ack = false;
    }
  }

  // Platoon return trigger.
  if (help_proxy_sent && m_return_triggered && !m_returning && m_return_start_s < 0.0) {
    const bool timeout_expired = (now_s - m_return_trigger_time_s) >= m_return_timeout_s;
    if (timeout_expired || m_return_flag_received) {
      m_returning = true;
      m_return_start_s = now_s;
      m_return_trigger_was_flag = !timeout_expired;
      m_controller->setReturning(true);

      m_return_start_pos_gt = Vector3D(m_prev_gt_x, m_prev_gt_y, m_prev_gt_z);
      m_return_phase_distance_m = 0.0;

      const uint8_t my_hops = m_flood_manager ? m_flood_manager->getNearestBaseHops() : UINT8_MAX;
      ReturningMsg ret;
      ret.drone_id = m_id;
      ret.hop_count = my_hops;

      ::Packet out;
      out.type = ::PacketType::CORE;
      out.src = m_id;
      out.dst = BROADCAST_ID;
      out.payload.resize(sizeof(ret));
      std::memcpy(out.payload.data(), &ret, sizeof(ret));
      m_comm.send(out);

      std::cout << "[RETURNING] t=" << now_s << "s drone=" << static_cast<int>(m_id)
                << " hops=" << static_cast<int>(my_hops)
                << " trigger=" << (timeout_expired ? "timeout" : "flag") << std::endl;
    }
  }

  // Exit returning mode on direct coverage.
  if (m_returning && hasDirectBaseCoverage()) {
    m_returning = false;
    m_return_complete_s = now_s;
    m_station_keeping = true;
    m_controller->setReturning(false);
    m_controller->setStationKeeping(true);
    if (m_velocity_actuator) {
      m_velocity_actuator->brake();
    }

    m_return_complete_pos_gt = Vector3D(m_prev_gt_x, m_prev_gt_y, m_prev_gt_z);
    m_station_keeping_ref_pos_gt = m_return_complete_pos_gt;
    m_station_keeping_max_drift_m = 0.0;

    std::cout << "[RETURN_COMPLETE] t=" << now_s << "s drone=" << static_cast<int>(m_id) << std::endl;
  }

  // Safety re-arm.
  if (m_station_keeping && !hasDirectBaseCoverage()) {
    double most_recent_direct = -1.0;
    for (const auto& [base_id, st] : m_base_state) {
      (void)base_id;
      if (st.last_direct_ack_rx_s > most_recent_direct) {
        most_recent_direct = st.last_direct_ack_rx_s;
      }
    }
    const double since = (most_recent_direct < 0.0) ? 1e9 : (now_s - most_recent_direct);
    if (since > 2.0 * DIRECT_ACK_TIMEOUT_S) {
      m_station_keeping = false;
      m_returning = true;
      m_rearm_count += 1;
      m_controller->setStationKeeping(false);
      m_controller->setReturning(true);
      std::cout << "[RETURN_REARM] t=" << now_s << "s drone=" << static_cast<int>(m_id)
                << " lost direct coverage after " << since << "s" << std::endl;
    }
  }

  // Post-mission debug logging.
  if (m_mission_start_s >= 0.0) {
    if (m_last_mission_log_s < 0.0 || (now_s - m_last_mission_log_s) >= m_mission_log_dt_s) {
      if (m_uwb_position) {
        m_uwb_position->retrieveCurrentPosition();
        const auto coords = m_uwb_position->getCoordinates();
        const uint8_t hops = m_flood_manager ? m_flood_manager->getNearestBaseHops() : UINT8_MAX;
        const size_t n_neighbors = m_neighbor_manager ? m_neighbor_manager->getNeighbors().size() : 0;
        std::cout << "[Reposition] t=" << now_s << "delta_t" << (now_s - m_last_mission_log_s) << "s drone=" << static_cast<int>(m_id)
                  << " hops=" << static_cast<int>(hops)
                  << " neighbors=" << n_neighbors
                  << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
                  << "," << (coords.size() > 1 ? coords[1] : 0.0)
                  << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")";

        if (m_last_help_proxy_rx_s >= 0.0) {
          std::cout << " after_HELP_PROXY_RX(t=" << m_last_help_proxy_rx_s << "s)";
        }
        if (m_last_help_proxy_tx_s >= 0.0) {
          std::cout << " after_HELP_PROXY_TX(t=" << m_last_help_proxy_tx_s << "s)";
        }
        std::cout << std::endl;

        if (m_reposition_csv && m_reposition_csv->good()) {
          (*m_reposition_csv) << now_s << ","
                              << static_cast<int>(m_id) << ","
                              << static_cast<int>(hops) << ","
                              << n_neighbors << ","
                              << (coords.size() > 0 ? coords[0] : 0.0) << ","
                              << (coords.size() > 1 ? coords[1] : 0.0) << ","
                              << (coords.size() > 2 ? coords[2] : 0.0)
                              << std::endl;
        }
      }
      m_last_mission_log_s = now_s;
    }
  }

  // Controller step.
  if (m_flood_manager && m_velocity_actuator && m_neighbor_manager && m_uwb_position) {
    m_uwb_position->retrieveCurrentPosition();
    m_controller->step(
      m_flood_manager.get(),
      m_velocity_actuator.get(),
      m_neighbor_manager.get(),
      m_uwb_position.get()
    );
  }

  sendPositionUpdate();

  ::ns3::Simulator::Schedule(::ns3::Seconds(m_tick_dt_s), ::ns3::MakeCallback(&Ns3Drone::onTick, this));
}

void Ns3Drone::dispatchPacket(const ::Packet& pkt) {
  if (!m_alive) {
    return;
  }
  if (pkt.payload.empty()) {
    return;
  }

  if (pkt.dst != m_id && pkt.dst != BROADCAST_ID) {
    return;
  }

  switch (pkt.type) {
    case ::PacketType::FLOOD:      ++m_rx_stats.flood;      break;
    case ::PacketType::NEIGHBOR:   ++m_rx_stats.neighbor;   break;
    case ::PacketType::UWB_BEACON: ++m_rx_stats.uwb_beacon; break;
    default: break;
  }

  // Direct FLOOD from a known base (i.e. a FloodStart/FloodDiscovery with
  // pkt.src == base_id) is proof the base is within one radio hop of us.
  // Update both the generic ACK-freshness clock (so the HELP_PROXY-emission
  // timeout sees we're alive) AND the direct-ACK clock (so isBaseReachableId
  // -- which the FloodManager consults to decide "should I claim hop=1?"
  // -- returns true before the first real POS_ACK lands).  This breaks the
  // chicken-and-egg between "pick nearest base" and "send POS_UPDATE to
  // get an ACK".
  if (pkt.type == ::PacketType::FLOOD) {
    auto it = m_base_state.find(pkt.src);
    if (it != m_base_state.end()) {
      const double now_s = ::ns3::Simulator::Now().GetSeconds();
      it->second.last_ack_rx_s = now_s;
      it->second.last_direct_ack_rx_s = now_s;
    }
  }

  m_dispatcher.handlePacket(pkt);
}

void Ns3Drone::handleCorePacket(const ::Packet& pkt) {
  if (pkt.payload.size() < 1) {
    return;
  }

  const auto type = static_cast<SimMsgType>(pkt.payload[0]);
  switch (type) {
    case SimMsgType::POS_ACK: {
      ++m_rx_stats.pos_ack;
      if (pkt.payload.size() < sizeof(PositionAckMsg)) {
        return;
      }
      PositionAckMsg ack;
      std::memcpy(&ack, pkt.payload.data(), sizeof(ack));

      // v2: accept ACKs from any base we know about.
      if (m_base_state.find(ack.base_id) == m_base_state.end()) {
        return;
      }

      if (ack.drone_id != m_id) {
        // Relay for multi-hop, dedup by (drone_id, seq).
        auto& relayed = m_relayed_ack_seqs[ack.drone_id];
        if (relayed.count(ack.seq)) {
          return;
        }
        relayed.insert(ack.seq);
        ::Packet relay_pkt;
        relay_pkt.type = ::PacketType::CORE;
        relay_pkt.src = m_id;
        relay_pkt.dst = BROADCAST_ID;
        relay_pkt.payload.resize(sizeof(ack));
        std::memcpy(relay_pkt.payload.data(), &ack, sizeof(ack));
        m_comm.send(relay_pkt);
        return;
      }

      // Direct vs relayed.
      const bool is_direct = (pkt.src == ack.base_id);
      const double now_s = ::ns3::Simulator::Now().GetSeconds();

      DroneBaseState& st = m_base_state[ack.base_id];

      if (is_direct) {
        st.last_direct_ack_rx_s = now_s;
      }

      if (help_proxy_sent && m_first_ack_after_help_s < 0.0) {
        m_first_ack_after_help_s = now_s;
      }

      // A direct ACK is unambiguous proof the base is within one radio hop;
      // a relayed ACK still proves the chain reached us.  In both cases
      // refresh last_ack_rx_s so a returning drone that re-enters base
      // coverage truthfully reports hops=1 (via getHopsFromBase's direct
      // short-circuit) instead of being stuck at the last flood-derived
      // value.  Helpers no longer need the old "freeze at hops=2" hack:
      // ControllerBase::step now auto-deactivates them when their lost
      // drones are all station-keeping, so a returned drone showing up at
      // hops=1 doesn't cause the formation to collapse.
      st.last_ack_rx_s = now_s;
      if (help_proxy_sent) {
        std::cout << "[RELAYED_ACK_RX] t=" << now_s
                  << "s drone=" << static_cast<int>(m_id)
                  << " seq=" << ack.seq << std::endl;

        const uint8_t hops = m_flood_manager
            ? m_flood_manager->getHopsFromBase(ack.base_id)
            : UINT8_MAX;
        if (!m_return_triggered && hops != UINT8_MAX) {
          m_return_triggered = true;
          m_return_trigger_time_s = now_s;
          m_return_timeout_s = static_cast<double>(MAX_CHAIN_HOPS - std::min(hops, MAX_CHAIN_HOPS)) * RETURN_DELTA_S;

          std::cout << "[RETURN_ARMED] t=" << m_return_trigger_time_s
                    << "s drone=" << static_cast<int>(m_id)
                    << " hops=" << static_cast<int>(hops)
                    << " timeout=" << m_return_timeout_s << "s" << std::endl;
        }
      }
      st.last_acked_seq = ack.seq;
      st.waiting_ack = false;

      // Synthesize base-as-neighbor entry ONLY on direct ACK.  Payload is
      // the v2 NeighborInfo layout: [id][flags][num_bases=1][base_id,hops=0][coords].
      if (is_direct && m_neighbor_manager) {
        ::Packet base_as_neighbor;
        base_as_neighbor.type = ::PacketType::NEIGHBOR;
        base_as_neighbor.src = ack.base_id;
        base_as_neighbor.dst = m_id;

        base_as_neighbor.payload.resize(5 + 3 * sizeof(double));
        base_as_neighbor.payload[0] = ack.base_id;                     // neighbor_id
        base_as_neighbor.payload[1] = 0;                                // flags
        base_as_neighbor.payload[2] = 1;                                // num_bases
        base_as_neighbor.payload[3] = ack.base_id;                      // (base_id, hops)
        base_as_neighbor.payload[4] = ack.base_hops_to_base_station;    //   hops (== 0)

        const double base_coords[3] = {ack.x, ack.y, ack.z};
        std::memcpy(base_as_neighbor.payload.data() + 5, base_coords, sizeof(base_coords));

        m_neighbor_manager->onPacketReceived(base_as_neighbor);
      }
      return;
    }

    case SimMsgType::HELP_PROXY: {
      ++m_rx_stats.help_proxy;
      if (pkt.payload.size() < sizeof(HelpProxyMsg)) {
        return;
      }
      HelpProxyMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));

      // v2: only react if the target base is one we know about.
      if (m_base_state.find(msg.base_id) == m_base_state.end()) {
        return;
      }

      if (msg.requester_id == m_id) {
        return;
      }

      m_last_help_proxy_rx_s = ::ns3::Simulator::Now().GetSeconds();
      if (m_uwb_position) {
        m_uwb_position->retrieveCurrentPosition();
        const auto coords = m_uwb_position->getCoordinates();
        std::cout << "[HELP_PROXY RX] t=" << m_last_help_proxy_rx_s << "s drone=" << static_cast<int>(m_id)
                  << " requester=" << static_cast<int>(msg.requester_id)
                  << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
                  << "," << (coords.size() > 1 ? coords[1] : 0.0)
                  << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")" << std::endl;
      }

      if (!help_proxy_sent && isBaseReachable()) {
        startMission();
      } else if (!isBaseReachable()) {
        if (!m_relayed_help_proxy.count(msg.requester_id)) {
          m_relayed_help_proxy.insert(msg.requester_id);
          ::Packet relay;
          relay.type = ::PacketType::CORE;
          relay.src = m_id;
          relay.dst = BROADCAST_ID;
          relay.payload.resize(sizeof(msg));
          std::memcpy(relay.payload.data(), &msg, sizeof(msg));
          m_comm.send(relay);
        }
      }
      return;
    }

    case SimMsgType::RETURNING: {
      if (pkt.payload.size() < sizeof(ReturningMsg)) return;
      ReturningMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));

      if (msg.drone_id == m_id) return;

      const uint8_t my_hops = m_flood_manager ? m_flood_manager->getNearestBaseHops() : UINT8_MAX;
      if (msg.hop_count == my_hops + 1 && m_return_triggered && !m_returning && !m_return_flag_received) {
        m_return_flag_received = true;
        std::cout << "[RETURN_FLAG_RX] t=" << ::ns3::Simulator::Now().GetSeconds()
                  << "s drone=" << static_cast<int>(m_id)
                  << " from=" << static_cast<int>(msg.drone_id)
                  << " (hop " << static_cast<int>(msg.hop_count) << ")" << std::endl;
      }

      if (!m_relayed_returning.count(msg.drone_id)) {
        m_relayed_returning.insert(msg.drone_id);
        ::Packet relay;
        relay.type = ::PacketType::CORE;
        relay.src = m_id;
        relay.dst = BROADCAST_ID;
        relay.payload.resize(sizeof(msg));
        std::memcpy(relay.payload.data(), &msg, sizeof(msg));
        m_comm.send(relay);
      }
      return;
    }

    case SimMsgType::POS_UPDATE: {
      ++m_rx_stats.pos_update;
      if (pkt.payload.size() < sizeof(PositionUpdateMsg)) {
        return;
      }
      PositionUpdateMsg msg;
      std::memcpy(&msg, pkt.payload.data(), sizeof(msg));

      if (m_base_state.find(msg.base_id) == m_base_state.end()) {
        return;
      }

      if (msg.drone_id == m_id) {
        return;
      }

      if (pkt.dst != BROADCAST_ID) {
        return;
      }

      auto& relayed = m_relayed_pos_update_seqs[msg.drone_id];
      if (relayed.count(msg.seq)) {
        return;
      }
      relayed.insert(msg.seq);

      ::Packet relay_pkt;
      relay_pkt.type = ::PacketType::CORE;
      relay_pkt.src = m_id;
      relay_pkt.dst = help_proxy_sent ? BROADCAST_ID : msg.base_id;
      relay_pkt.payload.resize(sizeof(msg));
      std::memcpy(relay_pkt.payload.data(), &msg, sizeof(msg));

      m_comm.send(relay_pkt);
      return;
    }

    default:
      return;
  }
}

void Ns3Drone::sendPositionUpdate() {
  if (!m_uwb_position || m_known_bases.empty()) {
    return;
  }

  // Pick target base: nearest reachable.  If none have flood convergence,
  // fall back to the round-robin HELP_PROXY cursor.
  uint8_t target_base = UINT8_MAX;
  if (m_flood_manager) {
    target_base = m_flood_manager->getNearestBaseId();
  }
  if (target_base == UINT8_MAX) {
    // No flood convergence yet (startup window or all bases dead).  Use
    // round-robin over known bases so we still send periodic heartbeats.
    target_base = m_known_bases[m_help_proxy_target_idx % m_known_bases.size()];
  }

  DroneBaseState& st = m_base_state[target_base];

  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  if ((now_s - st.last_pos_send_s) < m_pos_update_interval_s) {
    return;
  }

  m_uwb_position->retrieveCurrentPosition();
  const auto coords = m_uwb_position->getCoordinates();

  const double cur_x = coords.size() > 0 ? coords[0] : 0.0;
  const double cur_y = coords.size() > 1 ? coords[1] : 0.0;
  const double cur_z = coords.size() > 2 ? coords[2] : 0.0;
  if (st.has_last_sent_pos
      && (now_s - st.last_pos_send_s) < m_pos_update_max_interval_s) {
    const double dx = cur_x - st.last_sent_x;
    const double dy = cur_y - st.last_sent_y;
    const double dz = cur_z - st.last_sent_z;
    const double dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq < (m_pos_delta_threshold_m * m_pos_delta_threshold_m)) {
      return;
    }
  }

  PositionUpdateMsg pos;
  pos.drone_id = m_id;
  pos.base_id = target_base;
  pos.seq = ++m_pos_seq;
  pos.x = static_cast<float>(cur_x);
  pos.y = static_cast<float>(cur_y);
  pos.z = static_cast<float>(cur_z);

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = help_proxy_sent ? BROADCAST_ID : target_base;
  out.payload.resize(sizeof(pos));
  std::memcpy(out.payload.data(), &pos, sizeof(pos));

  m_comm.send(out);
  st.last_pos_send_s = now_s;
  st.last_sent_x = cur_x;
  st.last_sent_y = cur_y;
  st.last_sent_z = cur_z;
  st.has_last_sent_pos = true;
  st.waiting_ack = true;
}

void Ns3Drone::sendHelpProxy() {
  if (m_known_bases.empty()) {
    return;
  }

  m_last_help_proxy_tx_s = ::ns3::Simulator::Now().GetSeconds();

  // Round-robin across known bases — each HELP_PROXY targets a different
  // base so if some are dead we still eventually hit a live one.  Advancing
  // every tick means we retry through all bases within a few seconds.
  const uint8_t target_base = m_known_bases[m_help_proxy_target_idx % m_known_bases.size()];
  m_help_proxy_target_idx = (m_help_proxy_target_idx + 1) % m_known_bases.size();

  if (m_uwb_position) {
    m_uwb_position->retrieveCurrentPosition();
    const auto coords = m_uwb_position->getCoordinates();
    std::cout << "[HELP_PROXY TX] t=" << m_last_help_proxy_tx_s << "s drone=" << static_cast<int>(m_id)
              << " target_base=" << static_cast<int>(target_base)
              << " reason=ACK_TIMEOUT"
              << " pos=(" << (coords.size() > 0 ? coords[0] : 0.0)
              << "," << (coords.size() > 1 ? coords[1] : 0.0)
              << "," << (coords.size() > 2 ? coords[2] : 0.0) << ")" << std::endl;
  } else {
    std::cout << "[HELP_PROXY TX] t=" << m_last_help_proxy_tx_s << "s drone=" << static_cast<int>(m_id)
              << " target_base=" << static_cast<int>(target_base)
              << " reason=ACK_TIMEOUT" << std::endl;
  }

  HelpProxyMsg help;
  help.requester_id = m_id;
  help.base_id = target_base;

  ::Packet out;
  out.type = ::PacketType::CORE;
  out.src = m_id;
  out.dst = BROADCAST_ID;
  out.payload.resize(sizeof(help));
  std::memcpy(out.payload.data(), &help, sizeof(help));

  m_comm.send(out);

  stopMission();
  help_proxy_sent = true;
}

bool Ns3Drone::isBaseReachable() const {
  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  for (const auto& [base_id, st] : m_base_state) {
    (void)base_id;
    if ((now_s - st.last_ack_rx_s) <= m_ack_timeout_s) {
      return true;
    }
  }
  return false;
}

bool Ns3Drone::hasDirectBaseCoverage() const {
  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  for (const auto& [base_id, st] : m_base_state) {
    (void)base_id;
    if (st.last_direct_ack_rx_s < 0.0) continue;
    if ((now_s - st.last_direct_ack_rx_s) <= DIRECT_ACK_TIMEOUT_S) {
      return true;
    }
  }
  return false;
}

bool Ns3Drone::isBaseReachableId(uint8_t base_id) const {
  // FloodManager asks this to decide whether to short-circuit hops to 1
  // and whether to re-seed floods.  Both questions are about *direct*
  // radio reach -- a relayed ACK proves a chain exists but says nothing
  // about being inside the base's coverage circle.  Use the direct-ACK
  // clock (refreshed on real direct ACKs and on direct FLOOD bootstrap),
  // not the generic last_ack_rx_s which also counts relayed ACKs.
  auto it = m_base_state.find(base_id);
  if (it == m_base_state.end()) return false;
  if (it->second.last_direct_ack_rx_s < 0.0) return false;
  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  return (now_s - it->second.last_direct_ack_rx_s) <= DIRECT_ACK_TIMEOUT_S;
}

bool Ns3Drone::hasDirectCoverageId(uint8_t base_id) const {
  auto it = m_base_state.find(base_id);
  if (it == m_base_state.end()) return false;
  if (it->second.last_direct_ack_rx_s < 0.0) return false;
  const double now_s = ::ns3::Simulator::Now().GetSeconds();
  return (now_s - it->second.last_direct_ack_rx_s) <= DIRECT_ACK_TIMEOUT_S;
}
