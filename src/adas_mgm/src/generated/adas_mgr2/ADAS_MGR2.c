/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ADAS_MGR2.c
 *
 * Code generated for Simulink model 'ADAS_MGR2'.
 *
 * Model version                  : 1.68
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Tue Aug 18 12:41:30 2026
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

/* Named constants for Chart: '<S1>/Chart' */
#define ADAS_MGR2_Source_GPS           ((uint8_T)1U)
#define ADAS_MGR2_Source_Lane          ((uint8_T)0U)
#define ADAS_MGR2_State_Lane           ((uint8_T)0U)
#define ADAS_MGR2_State_Waypoint       ((uint8_T)1U)

/* Exported block parameters */
int32_T MGM_blend_cycles = 10;         /* Variable: blend_cycles
                                        * Referenced by: '<S7>/blend_cycles'
                                        * Reference path blend duration in ticks
                                        */
int32_T MGM_n_cycles = 50;             /* Variable: n_cycles
                                        * Referenced by: '<S1>/Chart'
                                        * Required consecutive lane confidence cycles
                                        */
real32_T MGM_a_down = 1.5F;            /* Variable: a_down
                                        * Referenced by: '<S8>/a_down'
                                        * Normal deceleration rate limit
                                        */
real32_T MGM_a_up = 0.5F;              /* Variable: a_up
                                        * Referenced by: '<S8>/a_up'
                                        * Acceleration rate limit
                                        */
real32_T MGM_lane_conf_exit = 0.35F;   /* Variable: lane_conf_exit
                                        * Referenced by: '<S1>/Chart'
                                        * LANE -> WAYPOINT confidence threshold
                                        */
real32_T MGM_lane_conf_return = 0.7F;  /* Variable: lane_conf_return
                                        * Referenced by: '<S1>/Chart'
                                        * WAYPOINT -> LANE confidence threshold
                                        */
real32_T MGM_lane_entry_max_cross = 0.5F;/* Variable: lane_entry_max_cross
                                          * Referenced by: '<S1>/Chart'
                                          * Maximum GPS cross-track error for WAYPOINT -> LANE
                                          */
real32_T MGM_v_accel_zone = 1.0F;      /* Variable: v_accel_zone
                                        * Referenced by: '<S1>/Chart'
                                        * Target velocity in GPS acceleration zone
                                        */
real32_T MGM_v_base = 0.6F;            /* Variable: v_base
                                        * Referenced by: '<S1>/Chart'
                                        * Base target velocity
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
static void ADAS_MGR2_process_lane_state(uint8_T *out_state, uint8_T
  *out_path_source, real32_T *out_v_ref_req, boolean_T *out_immediate_stop,
  int32_T *out_lane_low_cnt, int32_T *out_lane_high_cnt);
static void ADAS_MGR_process_waypoint_state(uint8_T *out_state, uint8_T
  *out_path_source, real32_T *out_v_ref_req, boolean_T *out_immediate_stop,
  int32_T *out_lane_low_cnt, int32_T *out_lane_high_cnt);

/* Function for Chart: '<S1>/Chart' */
static void ADAS_MGR2_process_lane_state(uint8_T *out_state, uint8_T
  *out_path_source, real32_T *out_v_ref_req, boolean_T *out_immediate_stop,
  int32_T *out_lane_low_cnt, int32_T *out_lane_high_cnt)
{
  *out_state = ADAS_MGR2_State_Lane;
  *out_path_source = ADAS_MGR2_Source_Lane;
  *out_lane_high_cnt = 0;

  /* Inport: '<Root>/core_snapshot' */
  if (ADAS_MGR2_U.core_snapshot.lane_confidence < MGM_lane_conf_exit) {
    if (ADAS_MGR2_DW.lane_low_cnt > 2147483646) {
      *out_lane_low_cnt = MAX_int32_T;
    } else {
      *out_lane_low_cnt = ADAS_MGR2_DW.lane_low_cnt + 1;
    }
  } else {
    *out_lane_low_cnt = 0;
  }

  if (*out_lane_low_cnt >= MGM_n_cycles) {
    *out_state = ADAS_MGR2_State_Waypoint;
    *out_path_source = ADAS_MGR2_Source_GPS;
    *out_lane_low_cnt = 0;
  }

  /* Inport: '<Root>/core_snapshot' */
  if (ADAS_MGR2_U.core_snapshot.estop) {
    *out_v_ref_req = 0.0F;
    *out_immediate_stop = true;
  } else if (ADAS_MGR2_U.core_snapshot.traffic_stop_required ||
             ADAS_MGR2_U.core_snapshot.gps_at_end) {
    *out_v_ref_req = 0.0F;
    *out_immediate_stop = false;
  } else if (ADAS_MGR2_U.core_snapshot.gps_accel_zone) {
    *out_v_ref_req = MGM_v_accel_zone;
    *out_immediate_stop = false;
  } else {
    *out_v_ref_req = MGM_v_base;
    *out_immediate_stop = false;
  }
}

/* Function for Chart: '<S1>/Chart' */
static void ADAS_MGR_process_waypoint_state(uint8_T *out_state, uint8_T
  *out_path_source, real32_T *out_v_ref_req, boolean_T *out_immediate_stop,
  int32_T *out_lane_low_cnt, int32_T *out_lane_high_cnt)
{
  *out_state = ADAS_MGR2_State_Waypoint;
  *out_path_source = ADAS_MGR2_Source_GPS;
  *out_lane_low_cnt = 0;

  /* Inport: '<Root>/core_snapshot' */
  if (ADAS_MGR2_U.core_snapshot.lane_confidence > MGM_lane_conf_return) {
    if (ADAS_MGR2_DW.lane_high_cnt > 2147483646) {
      *out_lane_high_cnt = MAX_int32_T;
    } else {
      *out_lane_high_cnt = ADAS_MGR2_DW.lane_high_cnt + 1;
    }
  } else {
    *out_lane_high_cnt = 0;
  }

  if ((*out_lane_high_cnt >= MGM_n_cycles) && ((MGM_lane_entry_max_cross <= 0.0F)
       || ((ADAS_MGR2_U.core_snapshot.gps_path.n > 0) &&
           (ADAS_MGR2_U.core_snapshot.gps_cross_track <=
            MGM_lane_entry_max_cross)))) {
    *out_state = ADAS_MGR2_State_Lane;
    *out_path_source = ADAS_MGR2_Source_Lane;
    *out_lane_high_cnt = 0;
  }

  if (ADAS_MGR2_U.core_snapshot.estop) {
    *out_v_ref_req = 0.0F;
    *out_immediate_stop = true;
  } else if (ADAS_MGR2_U.core_snapshot.traffic_stop_required ||
             ADAS_MGR2_U.core_snapshot.gps_at_end) {
    *out_v_ref_req = 0.0F;
    *out_immediate_stop = false;
  } else if (ADAS_MGR2_U.core_snapshot.gps_accel_zone) {
    *out_v_ref_req = MGM_v_accel_zone;
    *out_immediate_stop = false;
  } else {
    *out_v_ref_req = MGM_v_base;
    *out_immediate_stop = false;
  }

  /* End of Inport: '<Root>/core_snapshot' */
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
  boolean_T is_stale_repeat;
  boolean_T rtb_path_valid;
  static const int32_T offsets[4] = { 0, 1, 2, 3 };

  __m128 tmp;
  static const int32_T offsets_0[4] = { 0, 1, 2, 3 };

  __m128 tmp_0;
  __m128 tmp_1;

  /* Chart: '<S1>/Chart' */
  switch (ADAS_MGR2_Y.core_output.state) {
   case ADAS_MGR2_State_Lane:
    ADAS_MGR2_process_lane_state(&ADAS_MGR2_Y.core_output.state,
      &ADAS_MGR2_B.path_source, &ADAS_MGR2_B.v_ref_req,
      &ADAS_MGR2_B.immediate_stop, &ADAS_MGR2_B.n_valid,
      &ADAS_MGR2_DW.lane_high_cnt);
    ADAS_MGR2_DW.lane_low_cnt = ADAS_MGR2_B.n_valid;
    break;

   case ADAS_MGR2_State_Waypoint:
    ADAS_MGR_process_waypoint_state(&ADAS_MGR2_Y.core_output.state,
      &ADAS_MGR2_B.path_source, &ADAS_MGR2_B.v_ref_req,
      &ADAS_MGR2_B.immediate_stop, &ADAS_MGR2_DW.lane_low_cnt,
      &ADAS_MGR2_B.n_valid);
    ADAS_MGR2_DW.lane_high_cnt = ADAS_MGR2_B.n_valid;
    break;
  }

  /* End of Chart: '<S1>/Chart' */

  /* UnitDelay: '<S8>/Unit Delay' */
  ADAS_MGR2_B.UnitDelay = ADAS_MGR2_DW.UnitDelay_DSTATE;

  /* MATLAB Function: '<S8>/MATLAB Function' incorporates:
   *  Constant: '<S8>/MGM_PERIOD_S'
   *  Constant: '<S8>/a_down'
   *  Constant: '<S8>/a_up'
   *  UnitDelay: '<S8>/Unit Delay'
   */
  if (ADAS_MGR2_B.immediate_stop) {
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

  /* End of MATLAB Function: '<S8>/MATLAB Function' */

  /* MATLAB Function: '<S6>/Select_Path' incorporates:
   *  Inport: '<Root>/core_snapshot'
   */
  switch (ADAS_MGR2_B.path_source) {
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

  /* End of MATLAB Function: '<S6>/Select_Path' */

  /* MATLAB Function: '<S6>/Normalize_Path' */
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

  /* End of MATLAB Function: '<S6>/Normalize_Path' */

  /* MATLAB Function: '<S7>/Ref_Hold_Blend_Core' incorporates:
   *  BusCreator: '<S4>/Bus Creator'
   *  Constant: '<S7>/blend_cycles'
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
      ADAS_MGR2_B.v_min = rt_atan2f_snf(ADAS_MGR2_B.rtb_selected_path_pts[0].y,
        ADAS_MGR2_B.rtb_selected_path_pts[0].x);
      ADAS_MGR2_B.rtb_selected_path_pts_m = ADAS_MGR2_B.rtb_selected_path_pts[0]
        .x;
      rtb_selected_path_pts = ADAS_MGR2_B.rtb_selected_path_pts[0].y;
      for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i <= 16; ADAS_MGR2_B.i += 4) {
        tmp_0 = _mm_div_ps(_mm_add_ps(_mm_cvtepi32_ps(_mm_add_epi32
          (_mm_set1_epi32(ADAS_MGR2_B.i), _mm_loadu_si128((const __m128i *)
          &offsets_0[0]))), _mm_set1_ps(1.0F)), _mm_set1_ps(20.0F));
        _mm_storeu_ps(&ADAS_MGR2_B.target_x[ADAS_MGR2_B.i], _mm_mul_ps
                      (_mm_set1_ps(ADAS_MGR2_B.rtb_selected_path_pts_m), tmp_0));
        _mm_storeu_ps(&ADAS_MGR2_B.target_y[ADAS_MGR2_B.i], _mm_mul_ps
                      (_mm_set1_ps(rtb_selected_path_pts), tmp_0));
        _mm_storeu_ps(&ADAS_MGR2_B.target_yaw[ADAS_MGR2_B.i], _mm_set1_ps
                      (ADAS_MGR2_B.v_min));
        _mm_storeu_ps(&ADAS_MGR2_B.target_curvature[ADAS_MGR2_B.i], _mm_set1_ps
                      (0.0F));
      }

      ADAS_MGR2_DW.n_out = 20;
    }

    switch (ADAS_MGR2_B.path_source) {
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

    if (ADAS_MGR2_B.path_source != ADAS_MGR2_DW.last_src) {
      memcpy(&ADAS_MGR2_DW.from_x[0], &ADAS_MGR2_DW.ref_x[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.from_y[0], &ADAS_MGR2_DW.ref_y[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.from_yaw[0], &ADAS_MGR2_DW.ref_yaw[0], 20U * sizeof
             (real32_T));
      memcpy(&ADAS_MGR2_DW.from_curvature[0], &ADAS_MGR2_DW.ref_curvature[0],
             20U * sizeof(real32_T));
      ADAS_MGR2_DW.blend_left = MGM_blend_cycles;
      ADAS_MGR2_DW.last_src = ADAS_MGR2_B.path_source;
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
        ADAS_MGR2_B.v_min = ADAS_MGR2_DW.ref_y[19];
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
                        (_mm_set1_ps(ADAS_MGR2_B.v_min), tmp_0));
        }
      } else {
        for (ADAS_MGR2_B.i = 0; ADAS_MGR2_B.i < 20; ADAS_MGR2_B.i++) {
          ADAS_MGR2_B.v_min = ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.i] -
            ADAS_MGR2_B.UnitDelay;
          if (ADAS_MGR2_B.v_min > 0.01F) {
            ADAS_MGR2_DW.ref_x[ADAS_MGR2_B.i] = ADAS_MGR2_B.v_min;
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
   *  MATLAB Function: '<S7>/Ref_Hold_Blend_Core'
   *  Outport: '<Root>/core_output'
   *  UnitDelay: '<S8>/Unit Delay'
   */
  ADAS_MGR2_Y.core_output.path_source = ADAS_MGR2_B.path_source;
  ADAS_MGR2_Y.core_output.immediate_stop = ADAS_MGR2_B.immediate_stop;
  ADAS_MGR2_Y.core_output.v_ref = ADAS_MGR2_DW.UnitDelay_DSTATE;
  ADAS_MGR2_Y.core_output.n_points = ADAS_MGR2_DW.n_out;
}

/* Model initialize function */
void ADAS_MGR2_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* SystemInitialize for MATLAB Function: '<S7>/Ref_Hold_Blend_Core' */
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
