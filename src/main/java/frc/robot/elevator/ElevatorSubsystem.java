// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;

  private final LinearFilter currentFilter =
      LinearFilter.movingAverage(RobotConfig.get().elevator().currentTaps());
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final MotionMagicTorqueCurrentFOC positionRequest =
      new MotionMagicTorqueCurrentFOC(ElevatorPositions.STOWED);
  private final LoggedDashboardNumber ntposition =
      new LoggedDashboardNumber("Elevator/positionOverride", -1);

  private static final double PRE_MATCH_HOMING_MIN_MOVEMENT = 0.0;

  private static final double TOLERANCE = 0.0;

  // Add to config
  private static double minHeight = RobotConfig.get().elevator().maxHeight();
  private static double maxHeight = RobotConfig.get().elevator().minHeight();

  // Homing
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = 0.0;
  private double highestSeenHeight = 0.0;
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
    double currentPosition = getHeight();

    if (currentPosition < lowestSeenHeight) {
      lowestSeenHeight = currentPosition;
    }
  }

  @Override
  public void robotPeriodic() {
    double rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    switch (homingState) {
      case NOT_HOMED:
        motor.setControl(coastNeutralRequest);
        break;
      case PRE_MATCH_HOMING:
        if (DriverStation.isDisabled()) {
          motor.setControl(coastNeutralRequest);
        } else {
          motor.setControl(brakeNeutralRequest);

          if (!preMatchHomingOccured && rangeOfMotionSeen()) {
            // homingEndPosition + (currentAngle - minAngle)
            double homingEndPosition = RobotConfig.get().elevator().homingEndPosition();
            double homedPosition = homingEndPosition + (getHeight() - lowestSeenHeight);
            motor.setPosition(inchesToRotations(homedPosition));

            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
        break;
      case MID_MATCH_HOMING:
        motor.set(RobotConfig.get().elevator().homingVoltage());

        if (filteredCurrent > RobotConfig.get().elevator().homingCurrentThreshold()) {
          homingState = HomingState.HOMED;
          goalHeight = ElevatorPositions.STOWED;

          motor.setPosition(inchesToRotations(RobotConfig.get().elevator().homingEndPosition()));
        }
        break;
      case HOMED:
        double usedGoalPosition =
            ntposition.get() == -1 ? clampHeight(goalHeight) : ntposition.get();

        slot = goalHeight == minHeight ? 1 : 0;
        Logger.recordOutput("Elevator/UsedGoalPosition", usedGoalPosition);

        motor.setControl(
            positionRequest.withSlot(slot).withPosition(inchesToRotations(usedGoalPosition)));

        break;
    }
  }

  public boolean rangeOfMotionSeen() {
    return highestSeenHeight - lowestSeenHeight >= PRE_MATCH_HOMING_MIN_MOVEMENT;
  }

  public void setGoalHeight(double newHeight) {
    goalHeight = clampHeight(newHeight);
  }

  public double getHeight() {
    return rotationsToInches(motor.getPosition().getValueAsDouble());
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public void startPreMatchHoming() {
    homingState = HomingState.PRE_MATCH_HOMING;
  }

  public void startMidMatchHoming() {
    homingState = HomingState.MID_MATCH_HOMING;
  }

  public boolean atGoal() {
    return Math.abs(getHeight() - goalHeight) < TOLERANCE;
  }

  public boolean atGoal(double inches) {
    return Math.abs(getHeight() - inches) < TOLERANCE;
  }

  // Tune the radius in inches later
  private double rotationsToInches(double rotations) {
    return rotations * (2 * Math.PI * 0);
  }

  private double inchesToRotations(double inches) {
    return inches / (2 * Math.PI * 0);
  }

  private static double clampHeight(double height) {
    if (height < minHeight) {
      height = minHeight;
    } else if (height > maxHeight) {
      height = maxHeight;
    }
    return height;
  }
}
