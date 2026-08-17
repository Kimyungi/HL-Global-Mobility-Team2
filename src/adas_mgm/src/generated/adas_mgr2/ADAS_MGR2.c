/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ADAS_MGR2.c
 *
 * Code generated for Simulink model 'ADAS_MGR2'.
 *
 * Model version                  : 1.55
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Mon Aug 17 22:48:12 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "ADAS_MGR2.h"
#include "rtwtypes.h"
#include <string.h>
#include "ADAS_MGR2_private.h"
#include <emmintrin.h>
#include <xmmintrin.h>
#include <math.h>
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Named constants for Chart: '<S1>/ADAS_MGR_Decision' */
#define ADAS_MGR2_IN_Avoid             ((uint8_T)1U)
#define ADAS_MGR2_IN_Lane              ((uint8_T)2U)
#define ADAS_MGR2_IN_Parking           ((uint8_T)3U)
#define ADAS_MGR2_IN_Waypoint          ((uint8_T)4U)

/* Exported block parameters */
int32_T MGM_avoid_max_cycles = 1200;   /* Variable: avoid_max_cycles
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * Maximum AVOID duration in ticks
                                        */
int32_T MGM_avoid_return_hold_cycles = 300;/* Variable: avoid_return_hold_cycles
                                            * Referenced by: '<S1>/ADAS_MGR_Decision'
                                            * WAYPOINT hold time after leaving AVOID
                                            */
int32_T MGM_blend_cycles = 10;         /* Variable: blend_cycles
                                        * Referenced by: '<S6>/blend_cycles'
                                        * Reference path blend duration in ticks
                                        */
int32_T MGM_n_cycles = 50;             /* Variable: n_cycles
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * Required consecutive lane confidence cycles
                                        */
int32_T MGM_wrongway_cycles = 50;      /* Variable: wrongway_cycles
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * Consecutive cycles for wrong-way latch/set-clear
                                        */
real32_T MGM_a_down = 1.5F;            /* Variable: a_down
                                        * Referenced by: '<S7>/a_down'
                                        * Normal deceleration rate limit
                                        */
real32_T MGM_a_up = 0.5F;              /* Variable: a_up
                                        * Referenced by: '<S7>/a_up'
                                        * Acceleration rate limit
                                        */
real32_T MGM_lane_conf_exit = 0.35F;   /* Variable: lane_conf_exit
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * LANE -> WAYPOINT confidence threshold
                                        */
real32_T MGM_lane_conf_return = 0.7F;  /* Variable: lane_conf_return
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * WAYPOINT -> LANE confidence threshold
                                        */
real32_T MGM_lane_entry_max_cross = 0.5F;/* Variable: lane_entry_max_cross
                                          * Referenced by: '<S1>/ADAS_MGR_Decision'
                                          * Maximum GPS cross-track error for WAYPOINT -> LANE
                                          */
real32_T MGM_ttc_stop = 0.8F;          /* Variable: ttc_stop
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * Immediate stop TTC threshold
                                        */
real32_T MGM_v_accel_zone = 1.0F;      /* Variable: v_accel_zone
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * Target velocity in GPS acceleration zone
                                        */
real32_T MGM_v_base = 0.6F;            /* Variable: v_base
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * Base target velocity
                                        */
real32_T MGM_v_narrow = 0.2F;          /* Variable: v_narrow
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * AVOID velocity upper limit in narrow gap
                                        */
real32_T MGM_wrongway_yaw = 2.1F;      /* Variable: wrongway_yaw
                                        * Referenced by: '<S1>/ADAS_MGR_Decision'
                                        * Wrong-way latch yaw threshold
                                        */

/* Block signals (default storage) */
B_ADAS_MGR2_T ADAS_MGR2_B;

/* Block states (default storage) */
DW_ADAS_MGR2_T ADAS_MGR2_DW;

/* External inputs (root inport signals with default storage) */
ExtU_ADAS_MGR2_T ADAS_MGR2_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_ADAS_MGR2_T ADAS_MGR2_Y;

/* Real-time model */
static RT_MODEL_ADAS_MGR2_T ADAS_MGR2_M_;
RT_MODEL_ADAS_MGR2_T *const ADAS_MGR2_M = &ADAS_MGR2_M_;

/* Forward declaration for local functions */
static void ADAS_MGR2_enter_atomic_Lane(uint8_T *path_source, boolean_T
  *immediate_stop, real32_T *v_ref_req);
static void ADAS_MGR2_enter_atomic_Avoid(uint8_T *path_source, boolean_T
  *immediate_stop, real32_T *v_ref_req);
static void ADAS_MGR2_enter_atomic_Waypoint(uint8_T *path_source, boolean_T
  *immediate_stop, real32_T *v_ref_req);
static void ADAS_MGR2_MGM(uint8_T *path_source, boolean_T *immediate_stop,
  real32_T *v_ref_req);

/* Function for Chart: '<S1>/ADAS_MGR_Decision' */
static void ADAS_MGR2_enter_atomic_Lane(uint8_T *path_source, boolean_T
  *immediate_stop, real32_T *v_ref_req)
{
  ADAS_MGR2_Y.core_output.state = 0U;
  *path_source = 0U;

  /* Inport: '<Root>/core_snapshot' */
  if (ADAS_MGR2_U.core_snapshot.estop) {
    *v_ref_req = 0.0F;
    *immediate_stop = true;
  } else if (ADAS_MGR2_U.core_snapshot.traffic_stop_required) {
    *v_ref_req = 0.0F;
    *immediate_stop = false;
  } else if (ADAS_MGR2_DW.at_end_latched) {
    *v_ref_req = 0.0F;
    *immediate_stop = false;
  } else if (ADAS_MGR2_U.core_snapshot.gps_accel_zone) {
    *v_ref_req = MGM_v_accel_zone;
    *immediate_stop = false;
  } else {
    *v_ref_req = MGM_v_base;
    *immediate_stop = false;
  }

  /* End of Inport: '<Root>/core_snapshot' */
}

/* Function for Chart: '<S1>/ADAS_MGR_Decision' */
static void ADAS_MGR2_enter_atomic_Avoid(uint8_T *path_source, boolean_T
  *immediate_stop, real32_T *v_ref_req)
{
  if (ADAS_MGR2_DW.avoid_ticks <= 2147483646) {
    ADAS_MGR2_DW.avoid_ticks++;
  }

  ADAS_MGR2_Y.core_output.state = 2U;
  *path_source = 2U;

  /* Inport: '<Root>/core_snapshot' */
  if (ADAS_MGR2_U.core_snapshot.estop || (ADAS_MGR2_U.core_snapshot.avoid_ttc <
       MGM_ttc_stop)) {
    *v_ref_req = 0.0F;
    *immediate_stop = true;
  } else if (ADAS_MGR2_U.core_snapshot.avoid_narrow_gap) {
    if (ADAS_MGR2_U.core_snapshot.avoid_v_suggest < MGM_v_narrow) {
      *v_ref_req = ADAS_MGR2_U.core_snapshot.avoid_v_suggest;
    } else {
      *v_ref_req = MGM_v_narrow;
    }

    *immediate_stop = false;
  } else {
    *v_ref_req = ADAS_MGR2_U.core_snapshot.avoid_v_suggest;
    *immediate_stop = false;
  }

  /* End of Inport: '<Root>/core_snapshot' */
}

/* Function for Chart: '<S1>/ADAS_MGR_Decision' */
static void ADAS_MGR2_enter_atomic_Waypoint(uint8_T *path_source, boolean_T
  *immediate_stop, real32_T *v_ref_req)
{
  ADAS_MGR2_Y.core_output.state = 1U;
  *path_source = 1U;

  /* Inport: '<Root>/core_snapshot' */
  if (ADAS_MGR2_U.core_snapshot.estop) {
    *v_ref_req = 0.0F;
    *immediate_stop = true;
  } else if (ADAS_MGR2_U.core_snapshot.traffic_stop_required) {
    *v_ref_req = 0.0F;
    *immediate_stop = false;
  } else if (ADAS_MGR2_DW.at_end_latched) {
    *v_ref_req = 0.0F;
    *immediate_stop = false;
  } else if (ADAS_MGR2_DW.wrongway_latched) {
    *v_ref_req = 0.0F;
    *immediate_stop = false;
  } else if (ADAS_MGR2_U.core_snapshot.gps_accel_zone) {
    *v_ref_req = MGM_v_accel_zone;
    *immediate_stop = false;
  } else {
    *v_ref_req = MGM_v_base;
    *immediate_stop = false;
  }

  /* End of Inport: '<Root>/core_snapshot' */
}

/* Function for Chart: '<S1>/ADAS_MGR_Decision' */
static void ADAS_MGR2_MGM(uint8_T *path_source, boolean_T *immediate_stop,
  real32_T *v_ref_req)
{
  boolean_T wrongway_latched_tmp;

  /* Inport: '<Root>/core_snapshot' */
  if (ADAS_MGR2_U.core_snapshot.lane_confidence < MGM_lane_conf_exit) {
    if (ADAS_MGR2_DW.lane_low_cnt <= 2147483646) {
      ADAS_MGR2_DW.lane_low_cnt++;
    }
  } else {
    ADAS_MGR2_DW.lane_low_cnt = 0;
  }

  if (ADAS_MGR2_U.core_snapshot.lane_confidence > MGM_lane_conf_return) {
    if (ADAS_MGR2_DW.lane_high_cnt <= 2147483646) {
      ADAS_MGR2_DW.lane_high_cnt++;
    }
  } else {
    ADAS_MGR2_DW.lane_high_cnt = 0;
  }

  if (ADAS_MGR2_U.core_snapshot.gps_heading_valid &&
      (ADAS_MGR2_U.core_snapshot.gps_path.n > 0)) {
    if ((ADAS_MGR2_U.core_snapshot.gps_path.pts[0].yaw > MGM_wrongway_yaw) ||
        (ADAS_MGR2_U.core_snapshot.gps_path.pts[0].yaw < -MGM_wrongway_yaw)) {
      if (ADAS_MGR2_DW.wrongway_cnt <= 2147483646) {
        ADAS_MGR2_DW.wrongway_cnt++;
      }

      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
    } else {
      ADAS_MGR2_DW.wrongway_cnt = 0;
      if (ADAS_MGR2_DW.wrongway_ok_cnt <= 2147483646) {
        ADAS_MGR2_DW.wrongway_ok_cnt++;
      }
    }
  }

  wrongway_latched_tmp = !ADAS_MGR2_U.core_snapshot.estop_latch_release;
  ADAS_MGR2_DW.wrongway_latched = (wrongway_latched_tmp &&
    ((ADAS_MGR2_DW.wrongway_cnt >= MGM_wrongway_cycles) ||
     ((!ADAS_MGR2_U.core_snapshot.gps_heading_valid ||
       (ADAS_MGR2_U.core_snapshot.gps_path.n <= 0) ||
       (ADAS_MGR2_DW.wrongway_ok_cnt < MGM_wrongway_cycles)) &&
      ADAS_MGR2_DW.wrongway_latched)));
  ADAS_MGR2_DW.at_end_latched = (wrongway_latched_tmp &&
    (((ADAS_MGR2_Y.core_output.state != 3) &&
      ADAS_MGR2_U.core_snapshot.gps_at_end &&
      (ADAS_MGR2_U.core_snapshot.gps_path.n > 0)) || ADAS_MGR2_DW.at_end_latched));
  if (ADAS_MGR2_DW.return_hold_left > 0) {
    ADAS_MGR2_DW.return_hold_left--;
  }

  switch (ADAS_MGR2_DW.is_MGM) {
   case ADAS_MGR2_IN_Avoid:
    /* Inport: '<Root>/core_snapshot' */
    if (ADAS_MGR2_U.core_snapshot.avoid_maneuver_done || ((MGM_avoid_max_cycles >
          0) && (ADAS_MGR2_DW.avoid_ticks >= MGM_avoid_max_cycles))) {
      ADAS_MGR2_DW.avoid_ticks = 0;
      ADAS_MGR2_DW.return_hold_left = MGM_avoid_return_hold_cycles;
      ADAS_MGR2_DW.lane_low_cnt = 0;
      ADAS_MGR2_DW.lane_high_cnt = 0;
      ADAS_MGR2_DW.wrongway_cnt = 0;
      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
      ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Waypoint;
      ADAS_MGR2_enter_atomic_Waypoint(path_source, immediate_stop, v_ref_req);
    } else {
      if (ADAS_MGR2_DW.avoid_ticks <= 2147483646) {
        ADAS_MGR2_DW.avoid_ticks++;
      }

      ADAS_MGR2_Y.core_output.state = 2U;
      *path_source = 2U;
      if (ADAS_MGR2_U.core_snapshot.estop ||
          (ADAS_MGR2_U.core_snapshot.avoid_ttc < MGM_ttc_stop)) {
        *v_ref_req = 0.0F;
        *immediate_stop = true;
      } else if (ADAS_MGR2_U.core_snapshot.avoid_narrow_gap) {
        if (ADAS_MGR2_U.core_snapshot.avoid_v_suggest < MGM_v_narrow) {
          *v_ref_req = ADAS_MGR2_U.core_snapshot.avoid_v_suggest;
        } else {
          *v_ref_req = MGM_v_narrow;
        }

        *immediate_stop = false;
      } else {
        *v_ref_req = ADAS_MGR2_U.core_snapshot.avoid_v_suggest;
        *immediate_stop = false;
      }
    }
    break;

   case ADAS_MGR2_IN_Lane:
    /* Inport: '<Root>/core_snapshot' */
    if (ADAS_MGR2_U.core_snapshot.gps_parking_zone &&
        ADAS_MGR2_U.core_snapshot.parking_space_found) {
      ADAS_MGR2_DW.lane_low_cnt = 0;
      ADAS_MGR2_DW.lane_high_cnt = 0;
      ADAS_MGR2_DW.wrongway_cnt = 0;
      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
      ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Parking;
      ADAS_MGR2_Y.core_output.state = 3U;
      *path_source = 3U;
      if (ADAS_MGR2_U.core_snapshot.estop) {
        *v_ref_req = 0.0F;
        *immediate_stop = true;
      } else if (ADAS_MGR2_U.core_snapshot.parking_path_blocked) {
        *v_ref_req = 0.0F;
        *immediate_stop = false;
      } else {
        *v_ref_req = ADAS_MGR2_U.core_snapshot.parking_v_suggest;
        *immediate_stop = false;
      }
    } else if (ADAS_MGR2_U.core_snapshot.avoid_obstacle_detected &&
               ADAS_MGR2_U.core_snapshot.avoid_avoidable) {
      ADAS_MGR2_DW.lane_low_cnt = 0;
      ADAS_MGR2_DW.lane_high_cnt = 0;
      ADAS_MGR2_DW.wrongway_cnt = 0;
      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
      ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Avoid;
      ADAS_MGR2_enter_atomic_Avoid(path_source, immediate_stop, v_ref_req);
    } else if (ADAS_MGR2_DW.lane_low_cnt >= MGM_n_cycles) {
      ADAS_MGR2_DW.lane_low_cnt = 0;
      ADAS_MGR2_DW.lane_high_cnt = 0;
      ADAS_MGR2_DW.wrongway_cnt = 0;
      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
      ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Waypoint;
      ADAS_MGR2_enter_atomic_Waypoint(path_source, immediate_stop, v_ref_req);
    } else {
      ADAS_MGR2_Y.core_output.state = 0U;
      *path_source = 0U;
      if (ADAS_MGR2_U.core_snapshot.estop) {
        *v_ref_req = 0.0F;
        *immediate_stop = true;
      } else if (ADAS_MGR2_U.core_snapshot.traffic_stop_required) {
        *v_ref_req = 0.0F;
        *immediate_stop = false;
      } else if (ADAS_MGR2_DW.at_end_latched) {
        *v_ref_req = 0.0F;
        *immediate_stop = false;
      } else if (ADAS_MGR2_U.core_snapshot.gps_accel_zone) {
        *v_ref_req = MGM_v_accel_zone;
        *immediate_stop = false;
      } else {
        *v_ref_req = MGM_v_base;
        *immediate_stop = false;
      }
    }
    break;

   case ADAS_MGR2_IN_Parking:
    /* Inport: '<Root>/core_snapshot' */
    if (ADAS_MGR2_U.core_snapshot.parking_done) {
      ADAS_MGR2_DW.lane_low_cnt = 0;
      ADAS_MGR2_DW.lane_high_cnt = 0;
      ADAS_MGR2_DW.wrongway_cnt = 0;
      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
      ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Lane;
      ADAS_MGR2_enter_atomic_Lane(path_source, immediate_stop, v_ref_req);
    } else {
      ADAS_MGR2_Y.core_output.state = 3U;
      *path_source = 3U;
      if (ADAS_MGR2_U.core_snapshot.estop) {
        *v_ref_req = 0.0F;
        *immediate_stop = true;
      } else if (ADAS_MGR2_U.core_snapshot.parking_path_blocked) {
        *v_ref_req = 0.0F;
        *immediate_stop = false;
      } else {
        *v_ref_req = ADAS_MGR2_U.core_snapshot.parking_v_suggest;
        *immediate_stop = false;
      }
    }
    break;

   default:
    /* Inport: '<Root>/core_snapshot' */
    /* case IN_Waypoint: */
    if (ADAS_MGR2_U.core_snapshot.avoid_obstacle_detected &&
        ADAS_MGR2_U.core_snapshot.avoid_avoidable) {
      ADAS_MGR2_DW.lane_low_cnt = 0;
      ADAS_MGR2_DW.lane_high_cnt = 0;
      ADAS_MGR2_DW.wrongway_cnt = 0;
      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
      ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Avoid;
      ADAS_MGR2_enter_atomic_Avoid(path_source, immediate_stop, v_ref_req);
    } else if ((ADAS_MGR2_DW.return_hold_left == 0) &&
               (ADAS_MGR2_DW.lane_high_cnt >= MGM_n_cycles) &&
               ((MGM_lane_entry_max_cross <= 0.0F) ||
                ((ADAS_MGR2_U.core_snapshot.gps_path.n > 0) &&
                 (ADAS_MGR2_U.core_snapshot.gps_cross_track <=
                  MGM_lane_entry_max_cross)))) {
      ADAS_MGR2_DW.lane_low_cnt = 0;
      ADAS_MGR2_DW.lane_high_cnt = 0;
      ADAS_MGR2_DW.wrongway_cnt = 0;
      ADAS_MGR2_DW.wrongway_ok_cnt = 0;
      ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Lane;
      ADAS_MGR2_enter_atomic_Lane(path_source, immediate_stop, v_ref_req);
    } else {
      ADAS_MGR2_Y.core_output.state = 1U;
      *path_source = 1U;
      if (ADAS_MGR2_U.core_snapshot.estop) {
        *v_ref_req = 0.0F;
        *immediate_stop = true;
      } else if (ADAS_MGR2_U.core_snapshot.traffic_stop_required) {
        *v_ref_req = 0.0F;
        *immediate_stop = false;
      } else if (ADAS_MGR2_DW.at_end_latched) {
        *v_ref_req = 0.0F;
        *immediate_stop = false;
      } else if (ADAS_MGR2_DW.wrongway_latched) {
        *v_ref_req = 0.0F;
        *immediate_stop = false;
      } else if (ADAS_MGR2_U.core_snapshot.gps_accel_zone) {
        *v_ref_req = MGM_v_accel_zone;
        *immediate_stop = false;
      } else {
        *v_ref_req = MGM_v_base;
        *immediate_stop = false;
      }
    }
    break;
  }
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  int32_T tmp;
  int32_T tmp_0;
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0F) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = (real32_T)atan2((real32_T)tmp, (real32_T)tmp_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void mgm_step(void)
{
  real32_T rtb_selected_path_pts;
  uint8_T path_source;
  boolean_T immediate_stop;
  boolean_T is_stale_repeat;
  boolean_T rtb_path_valid;
  static const int32_T offsets[4] = { 0, 1, 2, 3 };

  __m128 tmp;
  static const int32_T offsets_0[4] = { 0, 1, 2, 3 };

  __m128 tmp_0;
  __m128 tmp_1;

  /* Chart: '<S1>/ADAS_MGR_Decision' */
  if (ADAS_MGR2_DW.is_active_c3_ADAS_MGR2 == 0) {
    ADAS_MGR2_DW.is_active_c3_ADAS_MGR2 = 1U;
    ADAS_MGR2_DW.lane_low_cnt = 0;
    ADAS_MGR2_DW.lane_high_cnt = 0;
    ADAS_MGR2_DW.wrongway_cnt = 0;
    ADAS_MGR2_DW.wrongway_ok_cnt = 0;
    ADAS_MGR2_DW.wrongway_latched = false;
    ADAS_MGR2_DW.return_hold_left = 0;
    ADAS_MGR2_DW.avoid_ticks = 0;
    ADAS_MGR2_DW.at_end_latched = false;
    ADAS_MGR2_Y.core_output.state = 0U;
    ADAS_MGR2_DW.is_MGM = ADAS_MGR2_IN_Lane;
    ADAS_MGR2_enter_atomic_Lane(&path_source, &immediate_stop,
      &ADAS_MGR2_B.v_ref_req);
  } else {
    ADAS_MGR2_MGM(&path_source, &immediate_stop, &ADAS_MGR2_B.v_ref_req);
  }

  /* End of Chart: '<S1>/ADAS_MGR_Decision' */

  /* UnitDelay: '<S7>/Unit Delay' */
  ADAS_MGR2_B.UnitDelay = ADAS_MGR2_DW.UnitDelay_DSTATE;

  /* MATLAB Function: '<S7>/MATLAB Function' incorporates:
   *  Constant: '<S7>/MGM_PERIOD_S'
   *  Constant: '<S7>/a_down'
   *  Constant: '<S7>/a_up'
   *  UnitDelay: '<S7>/Unit Delay'
   */
  if (immediate_stop) {
    ADAS_MGR2_DW.UnitDelay_DSTATE = 0.0F;
  } else {
    ADAS_MGR2_B.v_min = ADAS_MGR2_DW.UnitDelay_DSTATE - MGM_a_down * 0.01F;
    ADAS_MGR2_DW.UnitDelay_DSTATE += MGM_a_up * 0.01F;
    if (ADAS_MGR2_B.v_ref_req < ADAS_MGR2_B.v_min) {
      ADAS_MGR2_DW.UnitDelay_DSTATE = ADAS_MGR2_B.v_min;
    } else if (!(ADAS_MGR2_B.v_ref_req > ADAS_MGR2_DW.UnitDelay_DSTATE)) {
      ADAS_MGR2_DW.UnitDelay_DSTATE = ADAS_MGR2_B.v_ref_req;
    }
  }

  /* End of MATLAB Function: '<S7>/MATLAB Function' */

  /* MATLAB Function: '<S5>/Select_Path' incorporates:
   *  Inport: '<Root>/core_snapshot'
   */
  switch (path_source) {
   case 0U:
    ADAS_MGR2_B.n_valid = ADAS_MGR2_U.core_snapshot.lane_path.n;
    memcpy(&ADAS_MGR2_B.rtb_raw_path_pts[0],
           &ADAS_MGR2_U.core_snapshot.lane_path.pts[0], 20U * sizeof
           (CorePointBus));
    break;

   case 1U:
    ADAS_MGR2_B.n_valid = ADAS_MGR2_U.core_snapshot.gps_path.n;
    memcpy(&ADAS_MGR2_B.rtb_raw_path_pts[0],
           &ADAS_MGR2_U.core_snapshot.gps_path.pts[0], 20U * sizeof(CorePointBus));
    break;

   case 2U:
    ADAS_MGR2_B.n_valid = ADAS_MGR2_U.core_snapshot.avoid_path.n;
    memcpy(&ADAS_MGR2_B.rtb_raw_path_pts[0],
           &ADAS_MGR2_U.core_snapshot.avoid_path.pts[0], 20U * sizeof
           (CorePointBus));
    break;

   case 3U:
    ADAS_MGR2_B.n_valid = ADAS_MGR2_U.core_snapshot.parking_path.n;
    memcpy(&ADAS_MGR2_B.rtb_raw_path_pts[0],
           &ADAS_MGR2_U.core_snapshot.parking_path.pts[0], 20U * sizeof
           (CorePointBus));
    break;

   default:
    ADAS_MGR2_B.n_valid = ADAS_MGR2_U.core_snapshot.lane_path.n;
    memcpy(&ADAS_MGR2_B.rtb_raw_path_pts[0],
           &ADAS_MGR2_U.core_snapshot.lane_path.pts[0], 20U * sizeof
           (CorePointBus));
    break;
  }

  /* End of MATLAB Function: '<S5>/Select_Path' */

  /* MATLAB Function: '<S5>/Normalize_Path' */
  memcpy(&ADAS_MGR2_B.rtb_selected_path_pts[0], &ADAS_MGR2_B.rtb_raw_path_pts[0],
         20U * sizeof(CorePointBus));
  if (ADAS_MGR2_B.n_valid <= 0) {
    ADAS_MGR2_B.n_valid = 0;
    rtb_path_valid = false;
  } else {
    if (ADAS_MGR2_B.n_valid > 20) {
      ADAS_MGR2_B.n_valid = 20;
    }

    rtb_path_valid = true;
    for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i < 20; ADAS_MGR2_B.i++) {
      if (ADAS_MGR2_B.i + 1 <= ADAS_MGR2_B.n_valid) {
        ADAS_MGR2_B.rtb_selected_path_pts[ADAS_MGR2_B.i] =
          ADAS_MGR2_B.rtb_raw_path_pts[ADAS_MGR2_B.i];
      } else {
        ADAS_MGR2_B.rtb_selected_path_pts[ADAS_MGR2_B.i] =
          ADAS_MGR2_B.rtb_raw_path_pts[ADAS_MGR2_B.n_valid - 1];
      }
    }
  }

  /* End of MATLAB Function: '<S5>/Normalize_Path' */

  /* MATLAB Function: '<S6>/Ref_Hold_Blend_Core' incorporates:
   *  BusCreator: '<S4>/Bus Creator'
   *  Constant: '<S6>/blend_cycles'
   *  Inport: '<Root>/core_snapshot'
   *  Outport: '<Root>/core_output'
   */
  if (rtb_path_valid) {
    if (ADAS_MGR2_B.n_valid > 20) {
      ADAS_MGR2_B.n_valid = 20;
    }

    for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i < 20; ADAS_MGR2_B.i++) {
      ADAS_MGR2_B.target_x[ADAS_MGR2_B.i] =
        ADAS_MGR2_B.rtb_selected_path_pts[ADAS_MGR2_B.i].x;
      ADAS_MGR2_B.target_y[ADAS_MGR2_B.i] =
        ADAS_MGR2_B.rtb_selected_path_pts[ADAS_MGR2_B.i].y;
      ADAS_MGR2_B.target_yaw[ADAS_MGR2_B.i] =
        ADAS_MGR2_B.rtb_selected_path_pts[ADAS_MGR2_B.i].yaw;
      ADAS_MGR2_B.target_curvature[ADAS_MGR2_B.i] =
        ADAS_MGR2_B.rtb_selected_path_pts[ADAS_MGR2_B.i].curvature;
    }

    ADAS_MGR2_DW.n_out = ADAS_MGR2_B.n_valid;
    if (ADAS_MGR2_B.n_valid == 1) {
      ADAS_MGR2_B.v_ref_req = rt_atan2f_snf(ADAS_MGR2_B.rtb_selected_path_pts[0]
        .y, ADAS_MGR2_B.rtb_selected_path_pts[0].x);
      ADAS_MGR2_B.v_min = ADAS_MGR2_B.rtb_selected_path_pts[0].x;
      rtb_selected_path_pts = ADAS_MGR2_B.rtb_selected_path_pts[0].y;
      for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i <= 16; ADAS_MGR2_B.i += 4) {
        tmp_0 = _mm_div_ps(_mm_add_ps(_mm_cvtepi32_ps(_mm_add_epi32
          (_mm_set1_epi32(ADAS_MGR2_B.i), _mm_loadu_si128((const __m128i *)
          &offsets_0[0]))), _mm_set1_ps(1.0F)), _mm_set1_ps(20.0F));
        _mm_storeu_ps(&ADAS_MGR2_B.target_x[ADAS_MGR2_B.i], _mm_mul_ps
                      (_mm_set1_ps(ADAS_MGR2_B.v_min), tmp_0));
        _mm_storeu_ps(&ADAS_MGR2_B.target_y[ADAS_MGR2_B.i], _mm_mul_ps
                      (_mm_set1_ps(rtb_selected_path_pts), tmp_0));
        _mm_storeu_ps(&ADAS_MGR2_B.target_yaw[ADAS_MGR2_B.i], _mm_set1_ps
                      (ADAS_MGR2_B.v_ref_req));
        _mm_storeu_ps(&ADAS_MGR2_B.target_curvature[ADAS_MGR2_B.i], _mm_set1_ps
                      (0.0F));
      }

      ADAS_MGR2_DW.n_out = 20;
    }

    switch (path_source) {
     case 0U:
      rtb_path_valid = ADAS_MGR2_U.core_snapshot.lane_updated;
      break;

     case 1U:
      rtb_path_valid = ADAS_MGR2_U.core_snapshot.gps_updated;
      break;

     case 2U:
      rtb_path_valid = ADAS_MGR2_U.core_snapshot.avoid_updated;
      break;

     default:
      rtb_path_valid = false;
      break;
    }

    is_stale_repeat = false;
    if (!rtb_path_valid && ADAS_MGR2_DW.has_raw_target && (ADAS_MGR2_DW.raw_n ==
         ADAS_MGR2_B.n_valid)) {
      is_stale_repeat = true;
      for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i < 20; ADAS_MGR2_B.i++) {
        if (ADAS_MGR2_B.i + 1 <= ADAS_MGR2_B.n_valid) {
          if ((real32_T)fabs(ADAS_MGR2_B.target_x[ADAS_MGR2_B.i] -
                             ADAS_MGR2_DW.last_raw_x[ADAS_MGR2_B.i]) > 1.0E-6F)
          {
            is_stale_repeat = false;
          } else if ((real32_T)fabs(ADAS_MGR2_B.target_y[ADAS_MGR2_B.i] -
                      ADAS_MGR2_DW.last_raw_y[ADAS_MGR2_B.i]) > 1.0E-6F) {
            is_stale_repeat = false;
          } else if ((real32_T)fabs(ADAS_MGR2_B.target_yaw[ADAS_MGR2_B.i] -
                      ADAS_MGR2_DW.last_raw_yaw[ADAS_MGR2_B.i]) > 1.0E-6F) {
            is_stale_repeat = false;
          } else {
            is_stale_repeat = (!((real32_T)fabs
                                 (ADAS_MGR2_B.target_curvature[ADAS_MGR2_B.i] -
                                  ADAS_MGR2_DW.last_raw_curvature[ADAS_MGR2_B.i])
                                 > 1.0E-6F) && is_stale_repeat);
          }
        }
      }
    }

    if (path_source != ADAS_MGR2_DW.last_src) {
      memcpy(&ADAS_MGR2_DW.from_x[0], &ADAS_MGR2_DW.ref_x[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.from_y[0], &ADAS_MGR2_DW.ref_y[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.from_yaw[0], &ADAS_MGR2_DW.ref_yaw[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.from_curvature[0], &ADAS_MGR2_DW.ref_curvature[0],
             20U * sizeof(real32_T));
      ADAS_MGR2_DW.blend_left = MGM_blend_cycles;
      ADAS_MGR2_DW.last_src = path_source;
    }

    if (ADAS_MGR2_DW.blend_left > 0) {
      if (MGM_blend_cycles > 2147483646) {
        ADAS_MGR2_B.i = MAX_int32_T;
      } else {
        ADAS_MGR2_B.i = MGM_blend_cycles + 1;
      }

      ADAS_MGR2_B.UnitDelay = 1.0F - (real32_T)ADAS_MGR2_DW.blend_left /
        (real32_T)ADAS_MGR2_B.i;
      for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i <= 16; ADAS_MGR2_B.i += 4) {
        tmp_0 = _mm_loadu_ps(&ADAS_MGR2_B.target_x[ADAS_MGR2_B.i]);
        tmp = _mm_loadu_ps(&ADAS_MGR2_DW.from_x[ADAS_MGR2_B.i]);
        tmp_1 = _mm_set1_ps(ADAS_MGR2_B.UnitDelay);
        _mm_storeu_ps(&ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.i], _mm_add_ps(_mm_mul_ps
          (_mm_sub_ps(tmp_0, tmp), tmp_1), tmp));
        tmp_0 = _mm_loadu_ps(&ADAS_MGR2_B.target_y[ADAS_MGR2_B.i]);
        tmp = _mm_loadu_ps(&ADAS_MGR2_DW.from_y[ADAS_MGR2_B.i]);
        _mm_storeu_ps(&ADAS_MGR2_DW.ref_y[ADAS_MGR2_B.i], _mm_add_ps(_mm_mul_ps
          (_mm_sub_ps(tmp_0, tmp), tmp_1), tmp));
        tmp_0 = _mm_loadu_ps(&ADAS_MGR2_B.target_yaw[ADAS_MGR2_B.i]);
        tmp = _mm_loadu_ps(&ADAS_MGR2_DW.from_yaw[ADAS_MGR2_B.i]);
        _mm_storeu_ps(&ADAS_MGR2_DW.ref_yaw[ADAS_MGR2_B.i], _mm_add_ps
                      (_mm_mul_ps(_mm_sub_ps(tmp_0, tmp), tmp_1), tmp));
        tmp_0 = _mm_loadu_ps(&ADAS_MGR2_B.target_curvature[ADAS_MGR2_B.i]);
        tmp = _mm_loadu_ps(&ADAS_MGR2_DW.from_curvature[ADAS_MGR2_B.i]);
        _mm_storeu_ps(&ADAS_MGR2_DW.ref_curvature[ADAS_MGR2_B.i], _mm_add_ps
                      (_mm_mul_ps(_mm_sub_ps(tmp_0, tmp), tmp_1), tmp));
      }

      ADAS_MGR2_DW.blend_left--;
    } else if (is_stale_repeat) {
      ADAS_MGR2_B.UnitDelay *= 0.01F;
      if (ADAS_MGR2_B.n_valid == 1) {
        ADAS_MGR2_B.v_ref_req = ADAS_MGR2_DW.ref_y[19];
        ADAS_MGR2_B.UnitDelay = ADAS_MGR2_DW.ref_x[19] - ADAS_MGR2_B.UnitDelay;
        if (!(ADAS_MGR2_B.UnitDelay > 0.19999999F)) {
          ADAS_MGR2_B.UnitDelay = 0.19999999F;
        }

        for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i <= 16; ADAS_MGR2_B.i += 4) {
          tmp_0 = _mm_div_ps(_mm_add_ps(_mm_cvtepi32_ps(_mm_add_epi32
            (_mm_set1_epi32(ADAS_MGR2_B.i), _mm_loadu_si128((const __m128i *)
            &offsets[0]))), _mm_set1_ps(1.0F)), _mm_set1_ps(20.0F));
          _mm_storeu_ps(&ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.i], _mm_mul_ps
                        (_mm_set1_ps(ADAS_MGR2_B.UnitDelay), tmp_0));
          _mm_storeu_ps(&ADAS_MGR2_DW.ref_y[ADAS_MGR2_B.i], _mm_mul_ps
                        (_mm_set1_ps(ADAS_MGR2_B.v_ref_req), tmp_0));
        }
      } else {
        for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i < 20; ADAS_MGR2_B.i++) {
          ADAS_MGR2_B.v_ref_req = ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.i] -
            ADAS_MGR2_B.UnitDelay;
          if (ADAS_MGR2_B.v_ref_req > 0.01F) {
            ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.i] = ADAS_MGR2_B.v_ref_req;
          } else {
            ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.i] = 0.01F;
          }
        }
      }
    } else {
      memcpy(&ADAS_MGR2_DW.ref_x[0], &ADAS_MGR2_B.target_x[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.ref_y[0], &ADAS_MGR2_B.target_y[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.ref_yaw[0], &ADAS_MGR2_B.target_yaw[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.ref_curvature[0], &ADAS_MGR2_B.target_curvature[0],
             20U * sizeof(real32_T));
    }

    if (!is_stale_repeat) {
      memcpy(&ADAS_MGR2_DW.last_raw_x[0], &ADAS_MGR2_B.target_x[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.last_raw_y[0], &ADAS_MGR2_B.target_y[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.last_raw_yaw[0], &ADAS_MGR2_B.target_yaw[0], 20U *
             sizeof(real32_T));
      memcpy(&ADAS_MGR2_DW.last_raw_curvature[0], &ADAS_MGR2_B.target_curvature
             [0], 20U * sizeof(real32_T));
      ADAS_MGR2_DW.raw_n = ADAS_MGR2_B.n_valid;
      ADAS_MGR2_DW.has_raw_target = true;
    }
  }

  for (ADAS_MGR2_B.n_valid = 0; ADAS_MGR2_B.n_valid < 20; ADAS_MGR2_B.n_valid++)
  {
    ADAS_MGR2_Y.core_output.ref_points[ADAS_MGR2_B.n_valid].x =
      ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.n_valid];
    ADAS_MGR2_Y.core_output.ref_points[ADAS_MGR2_B.n_valid].y =
      ADAS_MGR2_DW.ref_y[ADAS_MGR2_B.n_valid];
    ADAS_MGR2_Y.core_output.ref_points[ADAS_MGR2_B.n_valid].yaw =
      ADAS_MGR2_DW.ref_yaw[ADAS_MGR2_B.n_valid];
    ADAS_MGR2_Y.core_output.ref_points[ADAS_MGR2_B.n_valid].curvature =
      ADAS_MGR2_DW.ref_curvature[ADAS_MGR2_B.n_valid];
  }

  /* BusCreator: '<S4>/Bus Creator' incorporates:
   *  MATLAB Function: '<S6>/Ref_Hold_Blend_Core'
   *  Outport: '<Root>/core_output'
   *  UnitDelay: '<S7>/Unit Delay'
   */
  ADAS_MGR2_Y.core_output.path_source = path_source;
  ADAS_MGR2_Y.core_output.immediate_stop = immediate_stop;
  ADAS_MGR2_Y.core_output.v_ref = ADAS_MGR2_DW.UnitDelay_DSTATE;
  ADAS_MGR2_Y.core_output.n_points = ADAS_MGR2_DW.n_out;
}

/* Model initialize function */
void ADAS_MGR2_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* SystemInitialize for MATLAB Function: '<S6>/Ref_Hold_Blend_Core' */
  ADAS_MGR2_DW.n_out = 1;
}

/* Model terminate function */
void ADAS_MGR2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
