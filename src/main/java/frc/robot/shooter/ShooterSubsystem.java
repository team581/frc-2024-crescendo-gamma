// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends LifecycleSubsystem {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private boolean usingNoteSpin = true;
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
  private double speakerDistance = 0;
  private double floorSpotDistance = 0;
  private double goalRPM = 0;
  private double usedTolerance = ShooterRPMs.TOLERANCE;

  private final InterpolatingDoubleTreeMap speakerDistanceToRPM = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap floorSpotDistanceToRPM =
      new InterpolatingDoubleTreeMap();

  private ShooterMode goalMode = ShooterMode.IDLE;

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
  public void robotPeriodic() {
    if (goalMode == ShooterMode.SHOOTER_AMP) {
      usingNoteSpin = false;
      usedTolerance = ShooterRPMs.AMP_TOLERANCE;
    } else {
      usingNoteSpin = true;
      usedTolerance = ShooterRPMs.TOLERANCE;
    }
    switch (goalMode) {
      case SPEAKER_SHOT:
        goalRPM = speakerDistanceToRPM.get(speakerDistance);
        break;
      case SUBWOOFER_SHOT:
        goalRPM = ShooterRPMs.SUBWOOFER;
        break;
      case PODIUM_SHOT:
        goalRPM = ShooterRPMs.PODIUM;
        break;
      case FLOOR_SHOT:
        goalRPM = floorSpotDistanceToRPM.get(floorSpotDistance);
        break;
      case OUTTAKE:
        goalRPM = ShooterRPMs.OUTTAKE;
        break;
      case SHOOTER_AMP:
        goalRPM = ShooterRPMs.SHOOTER_AMP;
        break;
      case IDLE:
        if (DriverStation.isAutonomous()) {
          goalRPM = speakerDistanceToRPM.get(speakerDistance);
        } else {
          goalRPM = ShooterRPMs.IDLE;
        }
        break;
      case FULLY_STOPPED:
        goalRPM = ShooterRPMs.FULLY_STOPPED;
        break;
      default:
        break;
    }

    Logger.recordOutput("Shooter/Mode", goalMode);
    Logger.recordOutput("Shooter/GoalRPM", goalRPM);
    Logger.recordOutput(
        "Shooter/GoalRPMForRightMotor", goalRPM * (usingNoteSpin ? ShooterRPMs.SPIN_RATIO : 1.0));
    Logger.recordOutput("Shooter/LeftMotor/Temperature", leftMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Shooter/LeftMotor/RPM", getRPM(leftMotor));
    Logger.recordOutput(
        "Shooter/LeftMotor/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/LeftMotor/StatorCurrent", leftMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shooter/LeftMotor/Voltage", leftMotor.getMotorVoltage().getValue());
    Logger.recordOutput(
        "Shooter/RightMotor/StatorCurrent", rightMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shooter/RightMotor/Voltage", rightMotor.getMotorVoltage().getValue());
    Logger.recordOutput("Shooter/RightMotor/RPM", getRPM(rightMotor));
    Logger.recordOutput("Shooter/RightMotor/Temperature", rightMotor.getDeviceTemp().getValue());
    Logger.recordOutput(
        "Shooter/RightMotor/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/AtGoal", atGoal(goalMode));

    if (goalMode == ShooterMode.FULLY_STOPPED) {
      leftMotor.disable();
      rightMotor.disable();
    } else {
      leftMotor.setControl(velocityRequest.withVelocity((goalRPM) / 60));
      rightMotor.setControl(
          velocityRequest.withVelocity(
              (goalRPM * (usingNoteSpin ? ShooterRPMs.SPIN_RATIO : 1.0)) / 60));
    }
  }

  public boolean atGoal(ShooterMode mode) {
    if (mode != goalMode) {
      return false;
    }

    if (goalMode == ShooterMode.IDLE || goalMode == ShooterMode.FULLY_STOPPED) {
      return true;
    }

    if (Math.abs((goalRPM * (usingNoteSpin ? ShooterRPMs.SPIN_RATIO : 1.0)) - getRPM(rightMotor))
            < ShooterRPMs.TOLERANCE
        && Math.abs(goalRPM - getRPM(leftMotor)) < ShooterRPMs.TOLERANCE) {
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
