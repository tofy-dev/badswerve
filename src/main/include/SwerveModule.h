// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorId, int turningMotorId,
               int absoluteEncoderId, units::radian_t absoluteOffset);

  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(const frc::SwerveModuleState& state);

  units::radian_t GetAbsoluteEncoderRad();
  void ResetEncoders();


 private:
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkMaxRelativeEncoder m_driveEncoder;
  rev::SparkMaxRelativeEncoder m_turningEncoder;
  frc::AnalogInput m_absoluteEncoder;
  units::radian_t m_absoluteOffset;


  frc2::PIDController m_drivePIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.0,
      {k::caps::moduleMaxAngularVelocity, k::caps::moduleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};
};
