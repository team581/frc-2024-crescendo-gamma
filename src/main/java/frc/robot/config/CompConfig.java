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
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.ConveyorConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IMUConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.QueuerConfig;
import frc.robot.config.RobotConfig.ShooterConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.config.RobotConfig.WristConfig;
import frc.robot.vision.VisionStrategy;

class CompConfig {
  private static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP =
      new ClosedLoopRampsConfigs()
          .withDutyCycleClosedLoopRampPeriod(0.04)
          .withTorqueClosedLoopRampPeriod(0.04)
          .withVoltageClosedLoopRampPeriod(0.04);
  private static final OpenLoopRampsConfigs OPEN_LOOP_RAMP =
      new OpenLoopRampsConfigs()
          .withDutyCycleOpenLoopRampPeriod(0.04)
          .withTorqueOpenLoopRampPeriod(0.04)
          .withVoltageOpenLoopRampPeriod(0.04);

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
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(200)
                          .withPeakReverseTorqueCurrent(0))
                  .withSlot0(new Slot0Configs().withKP(15).withKV(0).withKS(15))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              // Right motor
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(100)
                          .withSupplyCurrentLimitEnable(true))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(200)
                          .withPeakReverseTorqueCurrent(0))
                  .withSlot0(new Slot0Configs().withKP(12.0).withKV(0).withKS(15.0))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              speakerDistanceToRPM -> {
                speakerDistanceToRPM.put(0.0, 3000.0);
                speakerDistanceToRPM.put(2.0, 3000.0);
                speakerDistanceToRPM.put(2.5, 4000.0);
                speakerDistanceToRPM.put(4.0, 4000.0);
                speakerDistanceToRPM.put(4.5, 4000.0);
                speakerDistanceToRPM.put(6.0, 4000.0);
                speakerDistanceToRPM.put(6.5, 4800.0);
                speakerDistanceToRPM.put(8.0, 4800.0);
              },
              floorSpotDistanceToRPM -> {
                floorSpotDistanceToRPM.put(0.0, 1000.0);
                floorSpotDistanceToRPM.put(1.0, 1000.0);
                floorSpotDistanceToRPM.put(1.2, 1500.0);
                floorSpotDistanceToRPM.put(3.0, 1800.0);
                floorSpotDistanceToRPM.put(5.8, 2700.0);
                floorSpotDistanceToRPM.put(6.5, 2700.0);
                floorSpotDistanceToRPM.put(500.0, 2700.0);
                // Evil hacky way to have alternate shooter RPM for the amp area shot
                floorSpotDistanceToRPM.put(581.0, 3200.0);
              }),
          new ClimberConfig(
              19,
              20,
              4,
              10,
              -0.5,
              0.0,
              20.0,
              0.22398,
              1,
              new TalonFXConfiguration()
                  .withSlot0(new Slot0Configs().withKP(7.0))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Brake)),
              new TalonFXConfiguration()
                  .withSlot0(new Slot0Configs().withKP(7.0))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withMotorOutput(
                      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))),
          new WristConfig(
              14,
              new TalonFXConfiguration()
                  .withSlot0(
                      new Slot0Configs()
                          .withGravityType(GravityTypeValue.Arm_Cosine)
                          .withKG(0.0)
                          .withKP(300.0))
                  .withSlot1(
                      new Slot1Configs()
                          .withGravityType(GravityTypeValue.Arm_Cosine)
                          .withKG(0.0)
                          .withKP(300.0))
                  .withFeedback(
                      new FeedbackConfigs().withSensorToMechanismRatio(60.0 / 8.0 * 100.0 / 10.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(25)
                          .withSupplyCurrentLimitEnable(true))
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              Rotation2d.fromDegrees(0.0),
              Rotation2d.fromDegrees(0.0),
              Rotation2d.fromDegrees(61.5),
              distanceToAngleTolerance -> {
                distanceToAngleTolerance.put(0.85, 5.0);
                distanceToAngleTolerance.put(8.0, 0.5);
              },
              speakerDistanceToAngle -> {
                speakerDistanceToAngle.put(1.38, 58.1);
                speakerDistanceToAngle.put(2.16, 47.8);
                speakerDistanceToAngle.put(2.5, 42.0);
                speakerDistanceToAngle.put(3.5, 33.9635); // 1.854 - adjusted by quarter
                speakerDistanceToAngle.put(4.5, 28.20125); // 1.605 - adjusted by quarter
                speakerDistanceToAngle.put(5.5, 25.84825); // 1.393 - adjusted by quarter
                speakerDistanceToAngle.put(6.5, 21.30525); // 1.221 - adjusted by quarter
                speakerDistanceToAngle.put(7.5, 20.27075); // 1.083 - adjusted by quarter
                speakerDistanceToAngle.put(9.0, 18.7305); // 0.922 - adjusted by quarter
              },
              floorSpotDistanceToAngle -> {
                floorSpotDistanceToAngle.put(3.4, 18.0);
                floorSpotDistanceToAngle.put(5.8, 27.0);
                floorSpotDistanceToAngle.put(6.5, 50.0);
              }),
          new ElevatorConfig(
              21,
              0.10,
              new TalonFXConfiguration()
                  .withSlot0(new Slot0Configs().withKP(20.0))
                  .withSlot1(new Slot1Configs().withKP(10.0))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((50.0 / 8.0)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(20)
                          .withSupplyCurrentLimitEnable(true))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(20)
                          .withSupplyCurrentLimitEnable(true))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Brake))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              0,
              0.0,
              20.9,
              4.0,
              0.75),
          new IntakeConfig(
              15,
              1,
              new Debouncer(0.025, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(50)
                          .withSupplyCurrentLimitEnable(true))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new ConveyorConfig(
              2,
              2,
              0.075,
              new Debouncer(0.05, DebounceType.kBoth),
              new Debouncer(0.25, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(50)
                          .withSupplyCurrentLimitEnable(true))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new QueuerConfig(
              16,
              0,
              new Debouncer(0.0, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(20)
                          .withSupplyCurrentLimitEnable(true))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new SwerveConfig(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(20)
                  .withStatorCurrentLimit(70)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true),
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(120)
                  .withStatorCurrentLimit(70)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true),
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(80)
                  .withPeakReverseTorqueCurrent(-80),
              // new PhoenixPIDController(50, 0, 5),
              new PhoenixPIDController(20, 0, 2),
              true,
              true,
              true),
          new IMUConfig(
              1,
              distanceToAngleTolerance -> {
                distanceToAngleTolerance.put(1.0, 2.5);
                distanceToAngleTolerance.put(1.0, 2.5);
              }),
          new LightsConfig(3),
          new VisionConfig(
              VisionStrategy.TX_TY_AND_MEGATAG,
              4,
              0.4,
              0.4,
              tyToNoteDistance -> {
                tyToNoteDistance.put(-16.0, Units.inchesToMeters(12 + 15.25));
                tyToNoteDistance.put(16.8, Units.inchesToMeters(120 + 15));
                tyToNoteDistance.put(15.0, Units.inchesToMeters(96.0 + 15));
                tyToNoteDistance.put(10.5, Units.inchesToMeters(60.0 + 15));
                tyToNoteDistance.put(12.5, Units.inchesToMeters(72.0 + 15));
              },
              // x=right, y= forward, z=up
              new Translation3d(-0.025, Units.inchesToMeters(-1), Units.inchesToMeters(23.25)),
              new Rotation3d(0.0, Units.degreesToRadians(15.2), Units.degreesToRadians(-1.0)),
              56.015,
              81.428,
              28.517,
              -4.226));

  private CompConfig() {}
}
