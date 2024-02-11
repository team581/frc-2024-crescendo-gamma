// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
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
  private static double minHeight = 0.0;
  private static double maxHeight = 0.0;

  // Homing
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = 0.0;
  private double highestSeenHeight = 0.0;
  private HomingState homingState = HomingState.PRE_MATCH_HOMING;

  private double goalHeight = ElevatorPositions.STOWED;

  private int slot = 0;

  public ElevatorSubsystem(TalonFX motor) {

    super(SubsystemPriority.ELEVATOR);
    this.motor = motor;
    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());

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
            motor.setPosition(homedPosition);

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

          motor.setPosition(RobotConfig.get().elevator().homingEndPosition());
        }
        break;
      case HOMED:
        double usedGoalPosition = ntposition.get() == -1 ? clampHeight(goalHeight) : ntposition.get();

        slot = goalHeight == minHeight ? 1 : 0;
        Logger.recordOutput("Elevator/UsedGoalAngle", usedGoalPosition);

        motor.setControl(positionRequest.withSlot(slot).withPosition(usedGoalPosition));

        break;
    }
  }

  public boolean rangeOfMotionSeen() {
    return highestSeenHeight - lowestSeenHeight >= PRE_MATCH_HOMING_MIN_MOVEMENT;
  }

  public void setHeight(double newHeight) {
      goalHeight = clampHeight(newHeight);
    }


  public double getHeight() {
    return rotationsToInches(motor.getPosition().getValueAsDouble());
  }

  public boolean atGoal() {
    return Math.abs(getHeight() - goalHeight) < TOLERANCE;
  }

  // Tune later
  private double rotationsToInches(double rotations) {
    return rotations * 0;
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
