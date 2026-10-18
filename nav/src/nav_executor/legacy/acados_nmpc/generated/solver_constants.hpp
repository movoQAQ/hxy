#pragma once
namespace nav_executor::mpc::generated {
inline constexpr int Horizon=30, MaxObstacles=128, ParameterCount=540;
inline constexpr double Dt=0.05;
inline constexpr char AcadosCommit[]="8ff7c2a17e235d6cad6671538e923ccb7428dc3e";
enum Parameter {
  REF_X=0,
  REF_Y=1,
  TANGENT_X=2,
  TANGENT_Y=3,
  VREF=4,
  Q_CROSS=5,
  Q_ALONG=6,
  R_SPEED=7,
  R_DELTA=8,
  R_BRAKE=9,
  R_DIRECTION=10,
  Q_OBSTACLE=11,
  TERMINAL_WEIGHT=12,
  TERMINAL_STOP=13,
  MEASURED_X=14,
  MEASURED_Y=15,
  TRACK_ACTIVE=16,
  MAX_SPEED=17,
  MAX_CHANGE=18,
  MAX_ACCEL=19,
  MAX_BRAKE=20,
  MAX_TRACKING_ACC=21,
  MIN_X=22,
  MAX_X=23,
  MIN_Y=24,
  MAX_Y=25,
  SOFT_DISTANCE=26,
  BRAKE_GATE=27,
  OBSTACLES=28
};
}
