// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.Consumer;

public record RobotConfig(
    String robotName,
    String canivoreName,
    ShooterConfig shooter,
    ClimberConfig climber,
    WristConfig wrist,
    ElevatorConfig elevator,
    IntakeConfig intake,
    ConveyorConfig conveyor,
    QueuerConfig queuer,
    SwerveConfig swerve,
    IMUConfig imu,
    LightsConfig lights) {

  public record ShooterConfig(
      int leftMotorID,
      int rightMotorID,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig,
      Consumer<InterpolatingDoubleTreeMap> speakerShotRpms,
      Consumer<InterpolatingDoubleTreeMap> floorShotRpms) {}

  public record ClimberConfig(
      int leftMotorID,
      int rightMotorID,
      int currentTaps,
      double homingCurrentThreshold,
      double homingVoltage,
      double minDistance,
      double maxDistance,
      double rotationsToDistance,
      double distanceTolerance,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig) {}

  public record IntakeConfig(
      int motorID,
      int sensorID,
      double debounceTime,
      DebounceType debounceType,
      TalonFXConfiguration motorConfig) {}

  public record ConveyorConfig(
      int motorID,
      int sensorID,
      Debouncer handoffDebouncer,
      Debouncer scoringDebouncer,
      TalonFXConfiguration motorConfig) {}

  public record QueuerConfig(
      int motorID,
      int sensorID,
      double debounceTime,
      DebounceType debounceType,
      TalonFXConfiguration motorConfig) {}

  public record WristConfig(
      int motorID,
      TalonFXConfiguration motorConfig,
      CurrentLimitsConfigs strictCurrentLimits,
      Rotation2d homingEndPosition,
      Rotation2d minAngle,
      Rotation2d maxAngle,
      Rotation2d tolerance,
      Consumer<InterpolatingDoubleTreeMap> distanceToAngleTolerance,
      Consumer<InterpolatingDoubleTreeMap> speakerShotAngles,
      Consumer<InterpolatingDoubleTreeMap> floorShotAngles) {}

  public record ElevatorConfig(
      int motorID,
      TalonFXConfiguration motorConfig,
      double homingEndPosition,
      double minHeight,
      double maxHeight,
      double rotationsToDistance,
      double tolerance) {}

  public record IMUConfig(
      int deviceID, Consumer<InterpolatingDoubleTreeMap> distanceToAngleTolerance) {}

  public record LightsConfig(int deviceID) {}

  public record SwerveConfig(
      CurrentLimitsConfigs steerMotorCurrentLimits,
      CurrentLimitsConfigs driveMotorCurrentLimits,
      PhoenixPIDController snapController,
      boolean invertRotation) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  private static final String PRACTICE_BOT_SERIAL_NUMBER = "(no practice bot yet)";
  public static final String SERIAL_NUMBER = System.getenv("serialnum");
  public static final boolean IS_PRACTICE_BOT =
      SERIAL_NUMBER != null && SERIAL_NUMBER.equals(PRACTICE_BOT_SERIAL_NUMBER);

  public static RobotConfig get() {
    return IS_PRACTICE_BOT ? PracticeConfig.config : CompConfig.competitionBot;
  }
}
