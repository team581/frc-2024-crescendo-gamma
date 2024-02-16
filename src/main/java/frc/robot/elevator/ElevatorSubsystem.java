// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;

  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final PositionVoltage positionRequest =
      new PositionVoltage(ElevatorPositions.STOWED);
  private final LoggedDashboardNumber ntDistance =
      new LoggedDashboardNumber("Elevator/DistanceOverride", -1);

  // TODO: Put in config
  private static final double TOLERANCE = 0.0;

  // Homing
  double currentHeight = 0.0;
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = 0.0;
  private HomingState homingState = HomingState.PRE_MATCH_HOMING;

  private double goalHeight = ElevatorPositions.STOWED;

  private int slot = 0;

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR);

    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());

    this.motor = motor;
  }

  @Override
  public void disabledPeriodic() {
    currentHeight = getHeight();

    if (currentHeight < lowestSeenHeight) {
      lowestSeenHeight = currentHeight;
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
            double homingEndPosition = RobotConfig.get().elevator().homingEndPosition();
            double homedPosition = homingEndPosition + (getHeight() - lowestSeenHeight);
            motor.setPosition(inchesToRotations(homedPosition).getRotations());

            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
        break;

      case HOMED:
        double usedGoalPosition =
            clampHeight(ntDistance.get() == -1 ? goalHeight : ntDistance.get());

        slot = goalHeight == RobotConfig.get().elevator().minHeight() ? 1 : 0;
        Logger.recordOutput("Elevator/UsedGoalPosition", usedGoalPosition);

        motor.setControl(
            positionRequest.withSlot(slot).withPosition(inchesToRotations(usedGoalPosition).getRotations()));

        break;
      case MID_MATCH_HOMING:
        throw new IllegalStateException("Elevator can't do mid match homing");
    }
  }

  public void setGoalHeight(double newHeight) {
    goalHeight = clampHeight(newHeight);
  }

  public double getHeight() {
    return rotationsToInches(getRotations());
  }

  private Rotation2d getRotations() {
    return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public void startPreMatchHoming() {
    homingState = HomingState.PRE_MATCH_HOMING;
  }

  public boolean atGoal(double distance) {
    return Math.abs(getHeight() - distance) < TOLERANCE;
  }

  // Tune the radius in inches later
  private double rotationsToInches(Rotation2d rotations) {
    return rotations.getRadians() * (RobotConfig.get().elevator().rotationsToDistance());
  }

  private Rotation2d inchesToRotations(double inches) {
    return Rotation2d.fromRadians(inches / (RobotConfig.get().elevator().rotationsToDistance()));
  }

  private static double clampHeight(double height) {
    return MathUtil.clamp(height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }
}
