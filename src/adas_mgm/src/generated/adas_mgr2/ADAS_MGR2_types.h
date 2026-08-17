/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ADAS_MGR2_types.h
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

#ifndef ADAS_MGR2_types_h_
#define ADAS_MGR2_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_CorePointBus_
#define DEFINED_TYPEDEF_FOR_CorePointBus_

typedef struct {
  real32_T x;
  real32_T y;
  real32_T yaw;
  real32_T curvature;
} CorePointBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CoreOutputBus_
#define DEFINED_TYPEDEF_FOR_CoreOutputBus_

typedef struct {
  uint8_T state;
  uint8_T path_source;
  boolean_T immediate_stop;
  real32_T v_ref;
  int32_T n_points;
  CorePointBus ref_points[20];
} CoreOutputBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CorePathBus_
#define DEFINED_TYPEDEF_FOR_CorePathBus_

typedef struct {
  int32_T n;
  CorePointBus pts[20];
} CorePathBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CoreSnapshotBus_
#define DEFINED_TYPEDEF_FOR_CoreSnapshotBus_

typedef struct {
  real32_T lane_confidence;
  CorePathBus lane_path;
  CorePathBus gps_path;
  boolean_T gps_accel_zone;
  boolean_T gps_parking_zone;
  boolean_T gps_at_end;
  real32_T gps_cross_track;
  boolean_T gps_heading_valid;
  boolean_T lane_updated;
  boolean_T gps_updated;
  boolean_T avoid_updated;
  boolean_T avoid_obstacle_detected;
  boolean_T avoid_avoidable;
  real32_T avoid_ttc;
  boolean_T avoid_narrow_gap;
  boolean_T avoid_maneuver_done;
  CorePathBus avoid_path;
  real32_T avoid_v_suggest;
  boolean_T parking_space_found;
  boolean_T parking_path_blocked;
  boolean_T parking_done;
  CorePathBus parking_path;
  real32_T parking_v_suggest;
  boolean_T traffic_stop_required;
  boolean_T estop;
  boolean_T estop_latch_release;
} CoreSnapshotBus;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_ADAS_MGR2_T RT_MODEL_ADAS_MGR2_T;

#endif                                 /* ADAS_MGR2_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
