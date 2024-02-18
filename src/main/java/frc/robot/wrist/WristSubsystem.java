// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class WristSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final LoggedDashboardNumber ntAngle =
      new LoggedDashboardNumber("Wrist/AngleOverride", -1);
  private final PositionVoltage positionRequest =
      new PositionVoltage(WristPositions.STOWED.getRotations()).withEnableFOC(true);

  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();

  private HomingState homingState = HomingState.PRE_MATCH_HOMING;

  private static final InterpolatingDoubleTreeMap speakerDistanceToAngle =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap floorSpotDistanceToAngle =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToAngleTolerance =
      new InterpolatingDoubleTreeMap();

  private Rotation2d lowestSeenAngle = new Rotation2d();
  // TODO: This should be a variable, not a field
  private int slot = 0;
  // TODO: Delete this
  private Rotation2d TOLERANCE = Rotation2d.fromDegrees(5);

  private boolean preMatchHomingOccured = false;

  private Rotation2d goalAngle = new Rotation2d();

  public WristSubsystem(TalonFX motor) {
    super(SubsystemPriority.WRIST);
    this.motor = motor;

    motor.getConfigurator().apply(RobotConfig.get().wrist().motorConfig());

    RobotConfig.get().wrist().speakerShotAngles().accept(speakerDistanceToAngle);
    RobotConfig.get().wrist().distanceToAngleTolerance().accept(distanceToAngleTolerance);
    RobotConfig.get().wrist().floorShotAngles().accept(floorSpotDistanceToAngle);
  }

  @Override
  public void disabledPeriodic() {
    Rotation2d currentAngle = getAngle();

    if (currentAngle.getDegrees() < lowestSeenAngle.getDegrees()) {
      lowestSeenAngle = currentAngle;
    }
  }

  @Override
  public void robotPeriodic() {
    switch (homingState) {
      case NOT_HOMED:
        motor.setControl(coastNeutralRequest);
        break;
      case PRE_MATCH_HOMING:
        if (DriverStation.isDisabled()) {
          motor.setControl(coastNeutralRequest);
        } else {
          motor.setControl(brakeNeutralRequest);

          if (!preMatchHomingOccured) {
            Rotation2d homingEndPosition = RobotConfig.get().wrist().homingEndPosition();
            Rotation2d homedAngle =
                Rotation2d.fromDegrees(
                    homingEndPosition.getDegrees()
                        + (getAngle().getDegrees() - lowestSeenAngle.getDegrees()));
            motor.setPosition(homedAngle.getRotations());

            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
        break;
      case MID_MATCH_HOMING:
        if (preMatchHomingOccured) {
          Rotation2d homingEndPosition = RobotConfig.get().wrist().homingEndPosition();
          Rotation2d homedAngle =
              Rotation2d.fromDegrees(
                  homingEndPosition.getDegrees()
                      + (getAngle().getDegrees() - lowestSeenAngle.getDegrees()));
          motor.setPosition(homedAngle.getRotations());

          homingState = HomingState.HOMED;
        }
        break;
      case HOMED:
        Rotation2d usedGoalAngle =
            clampAngle(ntAngle.get() == -1 ? goalAngle : Rotation2d.fromDegrees(ntAngle.get()));

        slot = goalAngle.equals(RobotConfig.get().wrist().minAngle()) ? 1 : 0;
        Logger.recordOutput("Wrist/UsedGoalAngle", usedGoalAngle.getDegrees());
        Logger.recordOutput("Wrist/NTAngle", ntAngle.get());

        motor.setControl(positionRequest.withSlot(slot).withPosition(usedGoalAngle.getRotations()));
        break;
    }

    Logger.recordOutput(
        "Wrist/Position", Rotation2d.fromRotations(motor.getPosition().getValue()).getDegrees());
    Logger.recordOutput("Wrist/Current", motor.getStatorCurrent().getValue());
    Logger.recordOutput("Wrist/Voltage", motor.getMotorVoltage().getValue());
    Logger.recordOutput("Wrist/RPM", motor.getVelocity().getValue() * 60.0);
    Logger.recordOutput("Wrist/HomingState", homingState);
    Logger.recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
    Logger.recordOutput("Wrist/Temperature", motor.getDeviceTemp().getValue());
    Logger.recordOutput("Wrist/MotorPidSlot", slot);
    Logger.recordOutput("Wrist/ControlMode", motor.getControlMode().toString());
  }

  public void setAngle(Rotation2d angle) {
    angle = clampAngle(angle);

    if (!angle.equals(goalAngle)) {
      if (angle.equals(RobotConfig.get().wrist().minAngle())) {
        motor.getConfigurator().apply(RobotConfig.get().wrist().strictCurrentLimits());
      } else {
        motor.getConfigurator().apply(RobotConfig.get().wrist().motorConfig().CurrentLimits);
      }
    }

    goalAngle = angle;
  }

  private static Rotation2d clampAngle(Rotation2d angle) {
    if (angle.getDegrees() < RobotConfig.get().wrist().minAngle().getDegrees()) {
      angle = RobotConfig.get().wrist().minAngle();

    } else if (angle.getDegrees() > RobotConfig.get().wrist().maxAngle().getDegrees()) {
      angle = RobotConfig.get().wrist().maxAngle();
    }
    return angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
  }

  public boolean atAngle(Rotation2d angle) {
    return Math.abs(angle.getDegrees() - getAngle().getDegrees()) < TOLERANCE.getDegrees();
  }

  // TODO: Rename this to atAngleForSpeaker()
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

  // TODO: Make this a private method
  public Rotation2d getToleranceFromDistanceToSpeaker(double distance) {
    return distance > 8
        ? Rotation2d.fromDegrees(0.5)
        : distance < 0.85
            ? Rotation2d.fromDegrees(5.0)
            : Rotation2d.fromDegrees(distanceToAngleTolerance.get(distance));
  }

  // TODO: Delete this. The wrist tolerance should not be a concern of other subsystems.
  public void setTolerance(Rotation2d tolerance) {
    TOLERANCE = tolerance;
  }

  public Rotation2d getAngleFromDistanceToFloorSpot(double distance) {
    return Rotation2d.fromDegrees(floorSpotDistanceToAngle.get(distance));
  }
}
