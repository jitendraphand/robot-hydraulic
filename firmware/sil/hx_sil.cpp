// C API over the firmware core for the Python software-in-the-loop harness.
// Host-only (uses the heap); the embedded build uses static instances.
#include "hx/control.h"

// ---------------------------------------------------------------- C API
extern "C" {
void* hx_joint_new(const hx_joint_params_t* p) {
    auto* j = new hx::JointController();
    j->configure(*p);
    return j;
}
void hx_joint_free(void* j) { delete static_cast<hx::JointController*>(j); }
void hx_joint_reset(void* j) {
    auto* c = static_cast<hx::JointController*>(j);
    c->reset();
    c->clear_faults();
}
void hx_joint_step(void* j, const hx_joint_in_t* in, hx_joint_out_t* out) {
    static_cast<hx::JointController*>(j)->step(*in, *out);
}
uint32_t hx_sizeof_params(void) { return sizeof(hx_joint_params_t); }
uint32_t hx_sizeof_in(void) { return sizeof(hx_joint_in_t); }
uint32_t hx_sizeof_out(void) { return sizeof(hx_joint_out_t); }
}
