#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/ClosedLoopTypes.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include <algorithm>
#include <optional>

class MotorRegulator {
 private:
  rev::spark::SparkMax *spark_max;
  rev::spark::SparkClosedLoopController *closed_loop;
  std::optional<double> target_up, target_down;

 public:
  MotorRegulator(rev::spark::SparkMax *spark_max,
                 rev::spark::SparkClosedLoopController *closed_loop)
      : spark_max(spark_max), closed_loop(closed_loop) {
    // WARNING: motor must stay in the same place on boot !!!
  }
  ~MotorRegulator() {}

  void SetTargets(double p_target_up, double p_target_down) {
    target_up = p_target_up;
    target_down = p_target_down;
  }

  void Zero() { spark_max->GetEncoder().SetPosition(0); }

  void Up() {
    if (target_up.has_value()) {
      printf("going to %f\n", target_up.value());
      printf("curently at %f\n", spark_max->GetEncoder().GetPosition());
      closed_loop->SetSetpoint(
          target_up.value(),
          rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl);
    }
  }
  void Down() {
    if (target_down.has_value()) {
      printf("going to %f\n", target_down.value());
      printf("curently at %f\n", spark_max->GetEncoder().GetPosition());
      closed_loop->SetSetpoint(
          target_down.value(),
          rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl);
    }
  }
  void Pause() {
    printf("paused\n");
    closed_loop->SetSetpoint(0,
                             rev::spark::SparkLowLevel::ControlType::kVelocity);
  }
};
