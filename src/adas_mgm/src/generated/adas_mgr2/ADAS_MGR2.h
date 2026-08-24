/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ADAS_MGR2.h
 *
 * Code generated for Simulink model 'ADAS_MGR2'.
 *
 * Model version                  : 1.88
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Mon Aug 24 16:57:32 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef ADAS_MGR2_h_
#define ADAS_MGR2_h_
#ifndef ADAS_MGR2_COMMON_INCLUDES_
#define ADAS_MGR2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ADAS_MGR2_COMMON_INCLUDES_ */

#include "ADAS_MGR2_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  CorePointBus rtb_raw_path_pts[20];
  CorePointBus rtb_selected_path_pts[20];
  real32_T target_x[20];
  real32_T target_y[20];
  real32_T target_yaw[20];
  real32_T target_curvature[20];
  real32_T v_min;
  real32_T v_max;
  real32_T v_ref_req;
  real32_T rtb_selected_path_pts_m;
  int32_T n_valid;
  int32_T i;
} B_ADAS_MGR2_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T UnitDelay_DSTATE;           /* '<S8>/Unit Delay' */
  real32_T ref_x[20];                  /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T ref_y[20];                  /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T ref_yaw[20];                /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T ref_curvature[20];          /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T from_x[20];                 /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T from_y[20];                 /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T from_yaw[20];               /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T from_curvature[20];         /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T last_raw_x[20];             /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T last_raw_y[20];             /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T last_raw_yaw[20];           /* '<S7>/Ref_Hold_Blend_Core' */
  real32_T last_raw_curvature[20];     /* '<S7>/Ref_Hold_Blend_Core' */
  int32_T blend_left;                  /* '<S7>/Ref_Hold_Blend_Core' */
  int32_T n_out;                       /* '<S7>/Ref_Hold_Blend_Core' */
  int32_T raw_n;                       /* '<S7>/Ref_Hold_Blend_Core' */
  int32_T lane_low_cnt;                /* '<S1>/Chart' */
  int32_T lane_high_cnt;               /* '<S1>/Chart' */
  int32_T avoid_ticks;                 /* '<S1>/Chart' */
  int32_T return_hold_left;            /* '<S1>/Chart' */
  int32_T wrongway_cnt;                /* '<S1>/Chart' */
  int32_T wrongway_ok_cnt;             /* '<S1>/Chart' */
  int32_T stop_hold_left;              /* '<S1>/Chart' */
  uint8_T last_src;                    /* '<S7>/Ref_Hold_Blend_Core' */
  uint8_T stop_zone_done_id;           /* '<S1>/Chart' */
  uint8_T stop_zone_boot_id;           /* '<S1>/Chart' */
  boolean_T has_raw_target;            /* '<S7>/Ref_Hold_Blend_Core' */
  boolean_T wrongway_latched;          /* '<S1>/Chart' */
  boolean_T at_end_latched;            /* '<S1>/Chart' */
  boolean_T stop_zone_holding;         /* '<S1>/Chart' */
  boolean_T stop_zone_init;            /* '<S1>/Chart' */
} DW_ADAS_MGR2_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  CoreSnapshotBus core_snapshot;       /* '<Root>/core_snapshot' */
} ExtU_ADAS_MGR2_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  CoreOutputBus core_output;           /* '<Root>/core_output' */
} ExtY_ADAS_MGR2_T;

/* Real-time Model Data Structure */
struct tag_RTM_ADAS_MGR2_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_ADAS_MGR2_T ADAS_MGR2_B;

/* Block states (default storage) */
extern DW_ADAS_MGR2_T ADAS_MGR2_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_ADAS_MGR2_T ADAS_MGR2_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_ADAS_MGR2_T ADAS_MGR2_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern int32_T MGM_avoid_max_cycles;   /* Variable: avoid_max_cycles
                                        * Referenced by: '<S1>/Chart'
                                        * Maximum AVOID duration in ticks
                                        */
extern int32_T MGM_avoid_return_hold_cycles;/* Variable: avoid_return_hold_cycles
                                             * Referenced by: '<S1>/Chart'
                                             * WAYPOINT hold time after leaving AVOID
                                             */
extern int32_T MGM_avoid_zone_only;    /* Variable: avoid_zone_only
                                        * Referenced by: '<S1>/Chart'
                                        * Restrict AVOID entry to GPS avoid zone
                                        */
extern int32_T MGM_blend_cycles;       /* Variable: blend_cycles
                                        * Referenced by: '<S7>/blend_cycles'
                                        * Reference path blend duration in ticks
                                        */
extern int32_T MGM_n_cycles;           /* Variable: n_cycles
                                        * Referenced by: '<S1>/Chart'
                                        * Required consecutive lane confidence cycles
                                        */
extern int32_T MGM_stop_zone_hold_cycles;/* Variable: stop_zone_hold_cycles
                                          * Referenced by: '<S1>/Chart'
                                          * Stop-zone hold duration in ticks
                                          */
extern int32_T MGM_wrongway_cycles;    /* Variable: wrongway_cycles
                                        * Referenced by: '<S1>/Chart'
                                        * Consecutive cycles for wrong-way latch/set-clear
                                        */
extern real32_T MGM_a_down;            /* Variable: a_down
                                        * Referenced by: '<S8>/a_down'
                                        * Normal deceleration rate limit
                                        */
extern real32_T MGM_a_up;              /* Variable: a_up
                                        * Referenced by: '<S8>/a_up'
                                        * Acceleration rate limit
                                        */
extern real32_T MGM_lane_conf_exit;    /* Variable: lane_conf_exit
                                        * Referenced by: '<S1>/Chart'
                                        * LANE -> WAYPOINT confidence threshold
                                        */
extern real32_T MGM_lane_conf_return;  /* Variable: lane_conf_return
                                        * Referenced by: '<S1>/Chart'
                                        * WAYPOINT -> LANE confidence threshold
                                        */
extern real32_T MGM_lane_entry_max_cross;/* Variable: lane_entry_max_cross
                                          * Referenced by: '<S1>/Chart'
                                          * Maximum GPS cross-track error for WAYPOINT -> LANE
                                          */
extern real32_T MGM_ttc_stop;          /* Variable: ttc_stop
                                        * Referenced by: '<S1>/Chart'
                                        * Immediate stop TTC threshold
                                        */
extern real32_T MGM_v_accel_zone;      /* Variable: v_accel_zone
                                        * Referenced by: '<S1>/Chart'
                                        * Target velocity in GPS acceleration zone
                                        */
extern real32_T MGM_v_avoid;           /* Variable: v_avoid
                                        * Referenced by: '<S1>/Chart'
                                        * AVOID state velocity upper limit
                                        */
extern real32_T MGM_v_base;            /* Variable: v_base
                                        * Referenced by: '<S1>/Chart'
                                        * Base target velocity
                                        */
extern real32_T MGM_v_narrow;          /* Variable: v_narrow
                                        * Referenced by: '<S1>/Chart'
                                        * AVOID velocity upper limit in narrow gap
                                        */
extern real32_T MGM_wrongway_yaw;      /* Variable: wrongway_yaw
                                        * Referenced by: '<S1>/Chart'
                                        * Wrong-way latch yaw threshold
                                        */

/* Model entry point functions */
extern void ADAS_MGR2_initialize(void);
extern void mgm_step(void);
extern void ADAS_MGR2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_ADAS_MGR2_T *const ADAS_MGR2_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ADAS_MGR2'
 * '<S1>'   : 'ADAS_MGR2/ADAS_MGR'
 * '<S2>'   : 'ADAS_MGR2/ADAS_MGR/Chart'
 * '<S3>'   : 'ADAS_MGR2/ADAS_MGR/Input_Unpack'
 * '<S4>'   : 'ADAS_MGR2/ADAS_MGR/Output_Pack'
 * '<S5>'   : 'ADAS_MGR2/ADAS_MGR/Subsystem'
 * '<S6>'   : 'ADAS_MGR2/ADAS_MGR/Subsystem/Path_Select_Normalize'
 * '<S7>'   : 'ADAS_MGR2/ADAS_MGR/Subsystem/Ref_Hold_Blend'
 * '<S8>'   : 'ADAS_MGR2/ADAS_MGR/Subsystem/Velocity_Merge'
 * '<S9>'   : 'ADAS_MGR2/ADAS_MGR/Subsystem/Path_Select_Normalize/Normalize_Path'
 * '<S10>'  : 'ADAS_MGR2/ADAS_MGR/Subsystem/Path_Select_Normalize/Select_Path'
 * '<S11>'  : 'ADAS_MGR2/ADAS_MGR/Subsystem/Ref_Hold_Blend/Ref_Hold_Blend_Core'
 * '<S12>'  : 'ADAS_MGR2/ADAS_MGR/Subsystem/Velocity_Merge/MATLAB Function'
 */
#endif                                 /* ADAS_MGR2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
