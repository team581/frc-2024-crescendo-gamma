// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.ConveyorConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IMUConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.QueuerConfig;
import frc.robot.config.RobotConfig.ShooterConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.WristConfig;

class CompConfig {
  private static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP =
      new ClosedLoopRampsConfigs()
          .withDutyCycleClosedLoopRampPeriod(0.02)
          .withTorqueClosedLoopRampPeriod(0.02)
          .withVoltageClosedLoopRampPeriod(0.02);
  private static final OpenLoopRampsConfigs OPEN_LOOP_RAMP =
      new OpenLoopRampsConfigs()
          .withDutyCycleOpenLoopRampPeriod(0.02)
          .withTorqueOpenLoopRampPeriod(0.02)
          .withVoltageOpenLoopRampPeriod(0.02);

  public static final RobotConfig competitionBot =
      new RobotConfig(
          "competition",
          "581CANivore",
          new ShooterConfig(
              17,
              18,
              // Left motor
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(120))
                  .withSlot0(new Slot0Configs().withKP(10).withKV(0).withKS(15))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              // Right motor
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(120))
                  .withSlot0(new Slot0Configs().withKP(12).withKV(0).withKS(15))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              speakerDistanceToRPM -> {
                speakerDistanceToRPM.put(0.92, 3000.0);
                speakerDistanceToRPM.put(1.2, 3000.0);
                speakerDistanceToRPM.put(1.5, 4000.0);
                speakerDistanceToRPM.put(5.0, 4000.0);
                speakerDistanceToRPM.put(5.5, 5000.0);
                speakerDistanceToRPM.put(6.0, 5000.0);
              },
              floorSpotDistanceToRPM -> {
                floorSpotDistanceToRPM.put(1.6, 1700.0);
                floorSpotDistanceToRPM.put(3.4, 2000.0);
                floorSpotDistanceToRPM.put(4.8, 2700.0);
              }),
          new ClimberConfig(
              19,
              20,
              4,
              10,
              0.5,
              0.0,
              22.0,
              0.22398,
              1,
              new TalonFXConfiguration()
                  .withSlot0(new Slot0Configs().withKP(10))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0))
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)),
              new TalonFXConfiguration()
                  .withSlot0(new Slot0Configs().withKP(10))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0))
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new WristConfig(
              14,
              new TalonFXConfiguration()
                  .withSlot0(
                      new Slot0Configs()
                          .withGravityType(GravityTypeValue.Arm_Cosine)
                          .withKG(0.4)
                          .withKP(150.0))
                  .withSlot1(
                      new Slot1Configs()
                          .withGravityType(GravityTypeValue.Arm_Cosine)
                          .withKG(0.4)
                          .withKP(50.0))
                  .withFeedback(
                      new FeedbackConfigs().withSensorToMechanismRatio(60.0 / 8.0 * 100.0 / 10.0))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              new CurrentLimitsConfigs().withSupplyCurrentLimit(40),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(0),
              Rotation2d.fromDegrees(61),
              Rotation2d.fromDegrees(1),
              distanceToAngleTolerance -> {
                distanceToAngleTolerance.put(0.85, 5.0);
                distanceToAngleTolerance.put(8.0, 0.5);
              },
              speakerDistanceToAngle -> {
                speakerDistanceToAngle.put(0.92, 58.1);
                speakerDistanceToAngle.put(1.25, 58.1);
                speakerDistanceToAngle.put(2.0, 48.0);
                speakerDistanceToAngle.put(3.6, 33.4);
                speakerDistanceToAngle.put(4.4, 30.0);
                speakerDistanceToAngle.put(4.9, 29.0);
                speakerDistanceToAngle.put(5.3, 23.5);
              },
              floorSpotDistanceToAngle -> {
                floorSpotDistanceToAngle.put(1.6, 70.0);
                floorSpotDistanceToAngle.put(3.4, 60.0);
                floorSpotDistanceToAngle.put(4.8, 50.0);
              }),
          new ElevatorConfig(
              21,
              0.10,
              new TalonFXConfiguration()
                  .withSlot0(new Slot0Configs().withKP(20))
                  .withSlot1(new Slot1Configs().withKP(10))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((50.0 / 8.0) * (24.0 / 15.0)))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              0,
              0.0,
              22,
              4.0,
              0.75),
          new IntakeConfig(
              15,
              1,
              0.05,
              DebounceType.kBoth,
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(20))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new ConveyorConfig(
              2,
              2,
              0.05,
              new Debouncer(0.05, DebounceType.kBoth),
              new Debouncer(0.25, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(20))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new QueuerConfig(
              16,
              0,
              0.0,
              DebounceType.kBoth,
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new SwerveConfig(
              new CurrentLimitsConfigs().withStatorCurrentLimit(80),
              new CurrentLimitsConfigs().withStatorCurrentLimit(80),
              new PhoenixPIDController(10, 0, 0.5),
              true,
              false,
              false),
          new IMUConfig(
              1,
              distanceToAngleTolerance -> {
                // TODO: tune distance and angle
                distanceToAngleTolerance.put(1.0, 2.5);
                distanceToAngleTolerance.put(1.0, 2.5);
              }),
          new LightsConfig(3));

  private CompConfig() {}
}
