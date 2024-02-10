// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shoulder;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShoulderSubsystem extends LifecycleSubsystem {
  private static final Rotation2d PRE_MATCH_HOMING_MIN_MOVEMENT = Rotation2d.fromDegrees(60);
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;
  private final StrictFollower followRequest;
  private final LoggedDashboardNumber ntAngle =
      new LoggedDashboardNumber("Shoulder/AngleOverride", -1);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ShoulderPositions.STOWED_DOWN.getRotations()).withEnableFOC(true);
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private HomingState homingState = HomingState.PRE_MATCH_HOMING;
  private final LinearFilter currentFilter =
      LinearFilter.movingAverage(RobotConfig.get().shoulder().currentTaps());
  private static final InterpolatingDoubleTreeMap speakerDistanceToAngle =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap floorSpotDistanceToAngle =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToAngleTolerance =
      new InterpolatingDoubleTreeMap();

  private Rotation2d lowestSeenAngle = new Rotation2d();
  private Rotation2d highestSeenAngle = new Rotation2d();
  private int slot = 0;
  private Rotation2d TOLERANCE = Rotation2d.fromDegrees(5);

  private boolean preMatchHomingOccured = false;

  private Rotation2d goalAngle = new Rotation2d();

  public ShoulderSubsystem(TalonFX rightMotor, TalonFX leftMotor) {
    super(SubsystemPriority.SHOULDER);
    this.rightMotor = rightMotor;
    this.leftMotor = leftMotor;
    followRequest = new StrictFollower(rightMotor.getDeviceID());

    rightMotor.getConfigurator().apply(RobotConfig.get().shoulder().motorConfig());
    leftMotor.getConfigurator().apply(RobotConfig.get().shoulder().motorConfig());

    RobotConfig.get().shoulder().speakerShotAngles().accept(speakerDistanceToAngle);
    RobotConfig.get().shoulder().distanceToAngleTolerance().accept(distanceToAngleTolerance);
    RobotConfig.get().shoulder().floorShotAngles().accept(floorSpotDistanceToAngle);

    leftMotor.setInverted(false);
  }

  @Override
  public void disabledPeriodic() {
    Rotation2d currentAngle = getAngle();

    if (currentAngle.getDegrees() < lowestSeenAngle.getDegrees()) {
      lowestSeenAngle = currentAngle;
    }

    if (currentAngle.getDegrees() > highestSeenAngle.getDegrees()) {
      highestSeenAngle = currentAngle;
    }
  }

  @Override
  public void robotPeriodic() {
    double rawCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    Logger.recordOutput("Shoulder/RightMotor/FilteredCurrent", filteredCurrent);

    switch (homingState) {
      case NOT_HOMED:
        if (DriverStation.isDisabled()) {
          if (rangeOfMotionSeen()) {
            rightMotor.setControl(brakeNeutralRequest);
            leftMotor.setControl(brakeNeutralRequest);
          } else {
            rightMotor.setControl(coastNeutralRequest);
            leftMotor.setControl(coastNeutralRequest);
          }
        }
        break;
      case PRE_MATCH_HOMING:
        if (DriverStation.isDisabled()) {
          if (rangeOfMotionSeen()) {
            rightMotor.setControl(brakeNeutralRequest);
            leftMotor.setControl(brakeNeutralRequest);
          } else {
            rightMotor.setControl(coastNeutralRequest);
            leftMotor.setControl(coastNeutralRequest);
          }
        } else {
          rightMotor.setControl(brakeNeutralRequest);
          leftMotor.setControl(brakeNeutralRequest);

          if (!preMatchHomingOccured && rangeOfMotionSeen()) {
            // homingEndPosition + (currentAngle - minAngle)
            Rotation2d homingEndPosition = RobotConfig.get().shoulder().homingEndPosition();
            Rotation2d homedAngle =
                Rotation2d.fromDegrees(
                    homingEndPosition.getDegrees()
                        + (getAngle().getDegrees() - lowestSeenAngle.getDegrees()));
            rightMotor.setPosition(homedAngle.getRotations());
            leftMotor.setPosition(homedAngle.getRotations());
            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
        break;
      case MID_MATCH_HOMING:
        rightMotor.set(RobotConfig.get().shoulder().homingVoltage());
        leftMotor.setControl(followRequest);

        if (filteredCurrent > RobotConfig.get().shoulder().homingCurrentThreshold()) {
          homingState = HomingState.HOMED;
          goalAngle = ShoulderPositions.STOWED_UP;

          rightMotor.setPosition(RobotConfig.get().shoulder().homingEndPosition().getRotations());
          leftMotor.setPosition(RobotConfig.get().shoulder().homingEndPosition().getRotations());
        }

        break;
      case HOMED:
        Rotation2d usedGoalAngle =
            clampAngle(ntAngle.get() == -1 ? goalAngle : Rotation2d.fromDegrees(ntAngle.get()));

        slot = goalAngle.equals(RobotConfig.get().shoulder().minAngle()) ? 1 : 0;
        Logger.recordOutput("Shoulder/UsedGoalAngle", usedGoalAngle.getDegrees());
        Logger.recordOutput("Shoulder/NTAngle", ntAngle.get());

        rightMotor.setControl(
            positionRequest.withSlot(slot).withPosition(usedGoalAngle.getRotations()));
        leftMotor.setControl(followRequest);
        break;
    }

    Logger.recordOutput(
        "Shoulder/RightMotor/Position",
        Rotation2d.fromRotations(rightMotor.getPosition().getValue()).getDegrees());
    Logger.recordOutput(
        "Shoulder/LeftMotor/Position",
        Rotation2d.fromRotations(leftMotor.getPosition().getValue()).getDegrees());
    Logger.recordOutput("Shoulder/RightMotor/Current", rightMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shoulder/LeftMotor/Current", leftMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shoulder/RightMotor/Voltage", rightMotor.getMotorVoltage().getValue());
    Logger.recordOutput("Shoulder/LeftMotor/Voltage", leftMotor.getMotorVoltage().getValue());
    Logger.recordOutput("Shoulder/RightMotor/RPM", rightMotor.getVelocity().getValue() * 60.0);
    Logger.recordOutput("Shoulder/LeftMotor/RPM", leftMotor.getVelocity().getValue() * 60.0);
    Logger.recordOutput("Shoulder/HomingState", homingState);
    Logger.recordOutput("Shoulder/GoalAngle", goalAngle.getDegrees());
    Logger.recordOutput("Shoulder/RightMotor/Temperature", rightMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Shoulder/LeftMotor/Temperature", leftMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Shoulder/MotorPidSlot", slot);
    Logger.recordOutput("Shoulder/RightMotor/ControlMode", rightMotor.getControlMode().toString());
    Logger.recordOutput("Shoulder/LeftMotor/ControlMode", leftMotor.getControlMode().toString());
  }

  public boolean rangeOfMotionSeen() {
    return highestSeenAngle.getDegrees() - lowestSeenAngle.getDegrees()
        >= PRE_MATCH_HOMING_MIN_MOVEMENT.getDegrees();
  }

  public void setAngle(Rotation2d angle) {
    angle = clampAngle(angle);

    if (!angle.equals(goalAngle)) {
      if (angle.equals(RobotConfig.get().shoulder().minAngle())) {
        rightMotor.getConfigurator().apply(RobotConfig.get().shoulder().strictCurrentLimits());
        leftMotor.getConfigurator().apply(RobotConfig.get().shoulder().strictCurrentLimits());
      } else {
        rightMotor
            .getConfigurator()
            .apply(RobotConfig.get().shoulder().motorConfig().CurrentLimits);
        leftMotor.getConfigurator().apply(RobotConfig.get().shoulder().motorConfig().CurrentLimits);
      }
    }

    goalAngle = angle;
  }

  private static Rotation2d clampAngle(Rotation2d angle) {
    if (angle.getDegrees() < RobotConfig.get().shoulder().minAngle().getDegrees()) {
      angle = RobotConfig.get().shoulder().minAngle();

    } else if (angle.getDegrees() > RobotConfig.get().shoulder().maxAngle().getDegrees()) {
      angle = RobotConfig.get().shoulder().maxAngle();
    }
    return angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(rightMotor.getPosition().getValueAsDouble());
  }

  public boolean atAngle(Rotation2d angle) {
    return Math.abs(angle.getDegrees() - getAngle().getDegrees()) < TOLERANCE.getDegrees();
  }

  // convert distance to tolerance
  public boolean atAngle(Rotation2d angle, double distance) {
    setTolerance(getToleranceFromDistanceToSpeaker(distance));
    return atAngle(angle);
  }

  public void startMidMatchHoming() {
    homingState = HomingState.MID_MATCH_HOMING;
  }

  public void startPreMatchHoming() {
    homingState = HomingState.PRE_MATCH_HOMING;
  }

  public void resetHoming() {
    homingState = HomingState.NOT_HOMED;
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public Rotation2d getAngleFromDistanceToSpeaker(double distance) {
    return Rotation2d.fromDegrees(speakerDistanceToAngle.get(distance));
  }

  public Rotation2d getToleranceFromDistanceToSpeaker(double distance) {
    return distance > 8
        ? Rotation2d.fromDegrees(0.5)
        : distance < 0.85
            ? Rotation2d.fromDegrees(5.0)
            : Rotation2d.fromDegrees(distanceToAngleTolerance.get(distance));
  }

  public void setTolerance(Rotation2d tolerance) {
    TOLERANCE = tolerance;
  }

  public Rotation2d getAngleFromDistanceToFloorSpot(double distance) {
    return Rotation2d.fromDegrees(floorSpotDistanceToAngle.get(distance));
  }
}
