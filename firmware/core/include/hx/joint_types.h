/* HX-1 joint controller - plain C interface types.
 *
 * Shared by the C++ firmware core, the ESP32 application and the Python
 * software-in-the-loop harness (ctypes).  POD only, SI units throughout.
 * Keep the field order in sync with hx/sim/sil.py (checked by a unit test
 * that compares sizeof() with the Python ctypes structure).
 */
#ifndef HX_JOINT_TYPES_H
#define HX_JOINT_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum hx_actuator_type { HX_ACT_HYDRAULIC = 0, HX_ACT_SERVO = 1 };

enum hx_joint_mode {
    HX_MODE_OFF = 0,        /* valve centred (hydraulic lock), no control      */
    HX_MODE_HOLD = 1,       /* hold the position captured on entry             */
    HX_MODE_POSITION = 2,   /* position PID -> valve opening (stiff)           */
    HX_MODE_IMPEDANCE = 3,  /* PD + tau_ff -> force loop on chamber pressures  */
    HX_MODE_FORCE = 4       /* pure torque tracking (tau_ff only)              */
};

enum hx_joint_fault {
    HX_JF_NONE = 0,
    HX_JF_ENCODER = 1u << 0,
    HX_JF_PRESSURE_SENSOR = 1u << 1,
    HX_JF_OVERPRESSURE = 1u << 2,
    HX_JF_FOLLOWING = 1u << 3,
    HX_JF_LIMIT = 1u << 4,
    HX_JF_SUPPLY_LOW = 1u << 5
};

typedef struct {
    /* kinematics / limits */
    float q_min, q_max;          /* rad (hard limits)                         */
    float soft_margin;           /* rad: control refuses to push beyond       */
    int32_t actuator;            /* hx_actuator_type                          */
    /* cylinder + linkage (joint frame, metres) */
    int32_t axis;                /* 0=x 1=y 2=z                               */
    float anchor_parent[3];
    float anchor_child[3];
    float a_bore, a_annulus;     /* m^2                                       */
    float min_moment_arm;        /* m: below this, force control is unsafe   */
    /* hydraulics */
    float p_supply, p_return;    /* Pa                                        */
    float p_trip;                /* Pa: chamber over-pressure trip            */
    float p_supply_min;          /* Pa: below this -> SUPPLY_LOW fault        */
    float valve_q_rated;         /* m^3/s at full spool and rated dp          */
    float valve_dp_rated;        /* Pa                                        */
    float valve_overlap;         /* fraction of spool half travel             */
    float valve_deadband;        /* command deadband                          */
    float valve_rate;            /* full-travel per second (spool slew)       */
    /* gains */
    float kp, ki, kd;            /* N m / rad, N m / (rad s), N m s / rad     */
    float kp_pos_valve;          /* POSITION mode: valve per rad              */
    float kd_pos_valve;          /* POSITION mode: valve per rad/s            */
    float ki_pos_valve;          /* POSITION mode: valve per (rad s), clamped */
    float kp_force, ki_force;    /* valve per N, valve per (N s)              */
    float k_vel_ff;              /* scale on flow feed-forward (0..1.5)       */
    float tau_max;               /* N m                                       */
    float max_following_err;     /* rad                                       */
    float following_time;        /* s above max_following_err before fault    */
    /* servo actuator (servo joints) */
    float servo_center_us, servo_us_per_rad;
} hx_joint_params_t;

typedef struct {
    float dt;                    /* s                                         */
    int32_t mode;                /* hx_joint_mode                             */
    float q, qd;                 /* measured                                  */
    float p_a, p_b;              /* measured chamber pressures, Pa (A = bore) */
    float p_supply;              /* measured supply pressure, Pa              */
    int32_t q_valid, p_valid;
    float q_ref, qd_ref, tau_ff; /* setpoints                                 */
    float kp_scale, kd_scale;    /* 0..2 gain scheduling from the brain       */
} hx_joint_in_t;

typedef struct {
    float valve_cmd;             /* -1..1 (+ = extend: P->A, B->T)           */
    float spool;                 /* -1..1 after overlap comp. & slew limit    */
    float pulse_us;              /* valve servo pulse width                   */
    float tau_cmd;               /* N m                                       */
    float force_cmd, force_meas; /* N (+ = extension)                         */
    float tau_meas;              /* N m estimated from pressures              */
    float moment_arm;            /* m                                         */
    uint32_t faults;             /* hx_joint_fault bits (latched)             */
} hx_joint_out_t;

/* C API used by the SIL harness */
void* hx_joint_new(const hx_joint_params_t* p);
void hx_joint_free(void* j);
void hx_joint_reset(void* j);
void hx_joint_step(void* j, const hx_joint_in_t* in, hx_joint_out_t* out);
uint32_t hx_sizeof_params(void);
uint32_t hx_sizeof_in(void);
uint32_t hx_sizeof_out(void);

#ifdef __cplusplus
}
#endif
#endif
