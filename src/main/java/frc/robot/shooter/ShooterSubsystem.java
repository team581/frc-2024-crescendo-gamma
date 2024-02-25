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
  private static final double SPIN_RATIO = 3.0 / 5.0;
  private static final int IDLE_RPM = 1000;
  private static final double OUTTAKE_RPM = 2000; // TODO: adjust for desirable outtake speeds
  private static final double SUBWOOFER_SHOOTING_RPM = 3000;
  private static final double TOLERANCE_RPM = 250;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
  private double speakerDistance = 0;
  private double floorSpotDistance = 0;
  private double goalRPM = 0;

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
        if (DriverStation.isAutonomous()) {
          goalRPM = speakerDistanceToRPM.get(speakerDistance);
        } else {
          goalRPM = IDLE_RPM;
        }
        break;
      case FULLY_STOPPED:
        goalRPM = 0;
      default:
        break;
    }
    Logger.recordOutput("Shooter/DistanceToSpeaker", speakerDistance);
    Logger.recordOutput("Shooter/DistanceToFloorSpot", floorSpotDistance);
    Logger.recordOutput("Shooter/Mode", goalMode);
    Logger.recordOutput("Shooter/GoalRPM", goalRPM);
    Logger.recordOutput("Shooter/GoalRPMForRightMotor", goalRPM * SPIN_RATIO);
    Logger.recordOutput("Shooter/LeftMotor/Temperature", leftMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Shooter/LeftMotor/RPM", getRPM(leftMotor));
    Logger.recordOutput(
        "Shooter/LeftMotor/POutput", leftMotor.getClosedLoopProportionalOutput().getValue());
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
        "Shooter/RightMotor/POutput", rightMotor.getClosedLoopProportionalOutput().getValue());
    Logger.recordOutput(
        "Shooter/RightMotor/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/AtGoal", atGoal(goalMode));

    if (goalMode == ShooterMode.FULLY_STOPPED) {
      leftMotor.disable();
      rightMotor.disable();
    } else {
      leftMotor.setControl(velocityRequest.withVelocity((goalRPM) / 60));
      rightMotor.setControl(velocityRequest.withVelocity((goalRPM * SPIN_RATIO) / 60));
    }
  }

  public boolean atGoal(ShooterMode mode) {
    if (mode != goalMode) {
      return false;
    }

    if (goalMode == ShooterMode.IDLE || goalMode == ShooterMode.FULLY_STOPPED) {
      return true;
    }

    if (Math.abs((goalRPM * SPIN_RATIO) - getRPM(rightMotor)) < TOLERANCE_RPM
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
