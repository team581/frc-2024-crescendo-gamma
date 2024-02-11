// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterSubsystem extends LifecycleSubsystem {
  private static final int IDLE_RPM = 1000;
  private static final double PASSING_RPM = 1500; // Make this not horrible
  private static final double OUTTAKE_RPM = 2000; // TODO: adjust for desirable outtake speeds
  private static final double SUBWOOFER_SHOOTING_RPM = 5000;
  private static final double TOLERANCE_RPM = 100;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final LoggedDashboardNumber ntRPM = new LoggedDashboardNumber("Shooter/RPMOverride", -1);
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
  private double speakerDistance = 0;
  private double floorSpotDistance = 0;
  private double goalRPM = 0;
  double usedGoalRPM = 0;

  private static final InterpolatingDoubleTreeMap speakerDistanceToRPM =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap floorSpotDistanceToRPM =
      new InterpolatingDoubleTreeMap();

  private ShooterMode goalMode = ShooterMode.IDLE;
  private final StaticBrake brakeRequest = new StaticBrake();

  public ShooterSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SHOOTER);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    RobotConfig.get().shooter().speakerShotRpms().accept(speakerDistanceToRPM);
    RobotConfig.get().shooter().floorShotRpms().accept(floorSpotDistanceToRPM);

    leftMotor.getConfigurator().apply(RobotConfig.get().shooter().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().shooter().rightMotorConfig());
  }

  @Override
  public void enabledPeriodic() {
    double overrideRPM = ntRPM.get();

    Logger.recordOutput("Shooter/OverrideRPM", overrideRPM);
    Logger.recordOutput("Shooter/UsedRPM", usedGoalRPM);

    usedGoalRPM = overrideRPM == -1 ? goalRPM : overrideRPM;

    if (goalMode == ShooterMode.INTAKING) {
      rightMotor.setControl(brakeRequest);
      leftMotor.setControl(brakeRequest);
    } else {
      rightMotor.setControl(velocityRequest.withVelocity((usedGoalRPM) / 60));
      leftMotor.setControl(velocityRequest.withVelocity((usedGoalRPM - 500) / 60));
    }
  }

  @Override
  public void robotPeriodic() {
    switch (goalMode) {
      case SPEAKER_SHOT:
        goalRPM = speakerDistanceToRPM.get(speakerDistance);
        break;
      case SUBWOOFER_SHOT:
        goalRPM = SUBWOOFER_SHOOTING_RPM;
        break;
      case FLOOR_SHOT:
        goalRPM = floorSpotDistanceToRPM.get(floorSpotDistance);
        break;
      case OUTTAKE:
        goalRPM = OUTTAKE_RPM;
        break;
      case IDLE:
        goalRPM = IDLE_RPM;
        break;
      case PASSING_NOTE:
        goalRPM = PASSING_RPM;
        break;
      default:
        break;
    }
    Logger.recordOutput("Shooter/DistanceToSpeaker", speakerDistance);
    Logger.recordOutput("Shooter/DistanceToFloorSpot", floorSpotDistance);
    Logger.recordOutput("Shooter/Mode", goalMode);
    Logger.recordOutput("Shooter/GoalRPM", goalRPM);
    Logger.recordOutput("Shooter/LeftMotor/Temperature", leftMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Shooter/LeftMotor/RPM", getRPM(leftMotor));
    Logger.recordOutput(
        "Shooter/LeftMotor/POutput", leftMotor.getClosedLoopProportionalOutput().getValue());
    Logger.recordOutput(
        "Shooter/LeftMotor/SupplierCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/LeftMotor/StatorCurrent", leftMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shooter/LeftMotor/Voltage", leftMotor.getMotorVoltage().getValue());
    Logger.recordOutput(
        "Shooter/RightMotor/StatorCurrent", rightMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shooter/RightMotor/Voltage", rightMotor.getMotorVoltage().getValue());
    Logger.recordOutput("Shooter/RightMotor/RPM", getRPM(rightMotor));
    Logger.recordOutput("Shooter/RightMotor/Temperature", rightMotor.getDeviceTemp().getValue());
    Logger.recordOutput(
        "Shooter/RightMotor/POutput", rightMotor.getClosedLoopProportionalOutput().getValue());
    Logger.recordOutput(
        "Shooter/RightMotor/SupplierCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/AtGoal", atGoal(goalMode));
  }

  public boolean atGoal(ShooterMode mode) {
    if (mode != goalMode) {
      return false;
    }

    if (goalMode == ShooterMode.IDLE) {
      return true;
    }

    if (Math.abs(goalRPM - getRPM(rightMotor)) < TOLERANCE_RPM
        && Math.abs(goalRPM - getRPM(leftMotor)) < TOLERANCE_RPM) {
      return true;
    }

    return false;
  }

  private double getRPM(TalonFX motor) {
    return motor.getVelocity().getValue() * 60.0;
  }

  public void setGoalMode(ShooterMode newMode) {
    goalMode = newMode;
  }

  public ShooterMode getGoalMode() {
    return goalMode;
  }

  public void setSpeakerDistance(double distance) {
    speakerDistance = distance;
  }

  public void setFloorSpotDistance(double distance) {
    floorSpotDistance = distance;
  }
}
