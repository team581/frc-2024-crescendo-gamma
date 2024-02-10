// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.IMUConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.QueuerConfig;
import frc.robot.config.RobotConfig.ShooterConfig;
import frc.robot.config.RobotConfig.ShoulderConfig;
import frc.robot.config.RobotConfig.SwerveConfig;

class PracticeConfig {
  public static final RobotConfig config =
      new RobotConfig(
          "practice",
          "581CANivore",
          new ShooterConfig(
              18,
              19,
              // Bottom motor
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(120))
                  .withSlot0(new Slot0Configs().withKP(5).withKV(0).withKS(4.5))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast)),
              // Top motor
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(120))
                  .withSlot0(new Slot0Configs().withKP(5).withKV(0).withKS(5))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast)),
              speakerDistanceToRPM -> {
                speakerDistanceToRPM.put(0.92, 4000.0);
                speakerDistanceToRPM.put(6.0, 4000.0);
              },
              floorSpotDistanceToRPM -> {
                floorSpotDistanceToRPM.put(1.6, 1700.0);
                floorSpotDistanceToRPM.put(3.4, 2000.0);
                floorSpotDistanceToRPM.put(4.8, 2700.0);
              }),
          new ClimberConfig(20, 21, new TalonFXConfiguration()),
          new ShoulderConfig(
              14,
              15,
              new TalonFXConfiguration()
                  .withSlot0(
                      new Slot0Configs()
                          .withGravityType(GravityTypeValue.Arm_Cosine)
                          .withKG(0.4)
                          .withKP(200.0))
                  .withSlot1(
                      new Slot1Configs()
                          .withGravityType(GravityTypeValue.Arm_Cosine)
                          .withKG(0.4)
                          .withKP(200.0))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(2.0)
                          .withMotionMagicCruiseVelocity(1.5)
                          .withMotionMagicJerk(0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              1 / ((12.0 / 50.0) * (20.0 / 84.0) * (12.0 / 72.0))))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Brake)),
              new CurrentLimitsConfigs().withSupplyCurrentLimit(40),
              -0.05,
              10,
              Rotation2d.fromDegrees(-18),
              4,
              Rotation2d.fromDegrees(-18),
              Rotation2d.fromDegrees(95),
              distanceToAngleTolerance -> {
                distanceToAngleTolerance.put(0.85, 5.0);
                distanceToAngleTolerance.put(8.0, 0.5);
              },
              speakerDistanceToAngle -> {
                speakerDistanceToAngle.put(0.92, 0.0);
                speakerDistanceToAngle.put(1.5, 5.0);
                speakerDistanceToAngle.put(2.2, 17.0);
                speakerDistanceToAngle.put(2.57, 18.6);
                speakerDistanceToAngle.put(2.65, 20.0);
                speakerDistanceToAngle.put(3.3, 23.0);
                speakerDistanceToAngle.put(3.6, 24.0);
                speakerDistanceToAngle.put(3.8, 26.0);
                speakerDistanceToAngle.put(4.2, 26.0);
              },
              floorSpotDistanceToAngle -> {
                floorSpotDistanceToAngle.put(1.6, 70.0);
                floorSpotDistanceToAngle.put(3.4, 60.0);
                floorSpotDistanceToAngle.put(4.8, 50.0);
              }),
          new IntakeConfig(
              16,
              22,
              0,
              // Top Motor
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)),
              // Bottom Motor
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive))),
          new QueuerConfig(
              0,
              0,
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))),
          new SwerveConfig(
              new CurrentLimitsConfigs().withStatorCurrentLimit(80),
              new CurrentLimitsConfigs().withStatorCurrentLimit(80),
              new PhoenixPIDController(-10, 0, -0.5),
              false),
          new IMUConfig(
              1,
              distanceToAngleTolerance -> {
                // TODO: tune distance and angle
                distanceToAngleTolerance.put(1.0, 2.5);
                distanceToAngleTolerance.put(1.0, 2.5);
              }),
          new LightsConfig(23));
}
