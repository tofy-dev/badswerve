// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveModule.h"
#include "Constants.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();
  void Reset();

 private:
  const k_internal::swerve_wrapper fl = k::data::fl, fr = k::data::fr, bl = k::data::bl, br = k::data::br;

  SwerveModule m_frontLeft{fl.driveId, fl.steeringId, fl.absoluteId, fl.offset};
  SwerveModule m_frontRight{fr.driveId, fr.steeringId, fr.absoluteId, fr.offset};
  SwerveModule m_backLeft{bl.driveId, bl.steeringId, bl.absoluteId, bl.offset};
  SwerveModule m_backRight{br.driveId, br.steeringId, br.absoluteId, br.offset};

  AHRS m_gyro{frc::SPI::Port::kMXP};


  // move this to constants at a future date
  frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
  frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};
