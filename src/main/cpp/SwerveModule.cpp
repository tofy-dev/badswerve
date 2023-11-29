// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"
#include "Constants.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>
#include <frc/RobotController.h>


SwerveModule::SwerveModule(int driveMotorId, int turningMotorId,
                           int absoluteEncoderId, units::radian_t absoluteOffset)
    : m_driveMotor(driveMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder()),
      m_turningEncoder(m_turningMotor.GetEncoder()),
      m_absoluteEncoder(absoluteEncoderId),
      m_absoluteOffset(absoluteOffset) {

  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetPositionConversionFactor(2 * std::numbers::pi * k::data::wheelRadius /
                                     k::data::encoderResolution);
  // +++++++ set inverted if necessary

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetPositionConversionFactor(2 * std::numbers::pi /
                                       k::data::encoderResolution);
  // +++++++ set inverted if necessary

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{m_turningEncoder.GetPosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t{m_turningEncoder.GetPosition()});

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetPosition()}, state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}

units::radian_t SwerveModule::GetAbsoluteEncoderRad() {
    units::radian_t angle = units::radian_t{m_absoluteEncoder.GetVoltage() / frc::RobotController::GetVoltage5V()};
    angle *= 2 * M_PI;
    angle -= m_absoluteOffset;
    return angle;
    // return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}

void SwerveModule::ResetEncoders() {
    m_driveEncoder.SetPosition(0);
    m_turningEncoder.SetPosition(GetAbsoluteEncoderRad().value());
}