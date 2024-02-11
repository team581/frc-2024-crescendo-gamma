// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
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
      int mainMotorID, int followerMotorID, TalonFXConfiguration motorConfig) {}

  public record IntakeConfig(
      int topMotorID,
      int bottomMotorID,
      int sensorID,
      TalonFXConfiguration topMotorConfig,
      TalonFXConfiguration bottomMotorConfig) {}

  public record QueuerConfig(int motorID, int sensorID, TalonFXConfiguration motorConfig) {}
  
  public record WristConfig(
      int motorID,
      TalonFXConfiguration motorConfig,
      CurrentLimitsConfigs strictCurrentLimits,
      double homingVoltage,
      double homingCurrentThreshold,
      Rotation2d homingEndPosition,
      int currentTaps,
      Rotation2d minAngle,
      Rotation2d maxAngle,
      Consumer<InterpolatingDoubleTreeMap> distanceToAngleTolerance,
      Consumer<InterpolatingDoubleTreeMap> speakerShotAngles,
      Consumer<InterpolatingDoubleTreeMap> floorShotAngles) {}

  public record ElevatorConfig(
      int motorID,
      TalonFXConfiguration motorConfig,
      CurrentLimitsConfigs strictCurrentLimits,
      double homingVoltage,
      double homingCurrentThreshold,
      double homingEndPosition,
      int currentTaps,
      double minHeight,
      double maxHeight) {}

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
  private static final String PRACTICE_BOT_SERIAL_NUMBER = "0322443D"; // TODO: get serial number
  public static final String SERIAL_NUMBER = System.getenv("serialnum");
  public static final boolean IS_PRACTICE_BOT =
      SERIAL_NUMBER != null && SERIAL_NUMBER.equals(PRACTICE_BOT_SERIAL_NUMBER);

  public static RobotConfig get() {
    return IS_PRACTICE_BOT ? PracticeConfig.config : CompConfig.competitionBot;
  }
}
