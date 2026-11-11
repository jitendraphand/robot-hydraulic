// Sanity tests over the GENERATED robot configuration + CAN protocol.
#include "hx/can_protocol.h"
#include "hx/control.h"
#include "hx/robot_config.h"
#include "test.h"

using namespace hx;

TEST(generated_config_is_consistent) {
    int hyd = 0;
    for (int n = 0; n < cfg::kNodeCount; ++n) {
        const auto& nd = cfg::kNodes[n];
        CHECK(nd.joint_count <= cfg::kMaxJointsPerNode);
        for (int j = 0; j < nd.joint_count; ++j) {
            const auto& p = nd.joints[j];
            CHECK(p.q_min < p.q_max);
            CHECK(p.soft_margin >= 0 && p.soft_margin < 0.25f * (p.q_max - p.q_min));
            CHECK(p.tau_max > 0);
            if (p.actuator == HX_ACT_HYDRAULIC) {
                ++hyd;
                CHECK(p.a_bore > p.a_annulus && p.a_annulus > 0);
                CHECK(p.p_trip > p.p_supply);
                CHECK(p.kp_force > 0 && p.kp_pos_valve > 0);
                Linkage l;
                l.axis = p.axis;
                for (int i = 0; i < 3; ++i) { l.p[i] = p.anchor_parent[i]; l.c[i] = p.anchor_child[i]; }
                // the moment arm never vanishes or changes sign inside the range
                const float m0 = l.moment_arm(p.q_min);
                for (float q = p.q_min; q <= p.q_max; q += 0.01f) {
                    CHECK(fabsf(l.moment_arm(q)) >= p.min_moment_arm);
                    CHECK(signf(l.moment_arm(q)) == signf(m0));
                }
            }
        }
    }
    CHECK(hyd == 14);
}

TEST(can_frames_fit_and_ids_unique) {
    CHECK(sizeof(can::SetpointRaw) == 8);
    CHECK(sizeof(can::StateRaw) == 8);
    CHECK(sizeof(can::StatusRaw) <= 8);
    // setpoint/state id ranges for all nodes must not collide with other messages
    for (int n = 0; n < cfg::kNodeCount; ++n) {
        const auto& nd = cfg::kNodes[n];
        for (int j = 0; j < nd.joint_count; ++j) {
            CHECK(can::setpoint_id(nd.id, (uint8_t)j) < can::kIdState);
            CHECK(can::state_id(nd.id, (uint8_t)j) < can::kIdStatus);
        }
    }
}

TEST(c_struct_sizes_match_sil_contract) {
    // the Python ctypes mirror (hx/sim/sil.py) asserts the same numbers
    CHECK(sizeof(hx_joint_params_t) == 37 * 4);
    CHECK(sizeof(hx_joint_in_t) == 14 * 4);
    CHECK(sizeof(hx_joint_out_t) == 9 * 4);
}
