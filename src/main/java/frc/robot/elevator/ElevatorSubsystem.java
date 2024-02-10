// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.controls.CoastOut;
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
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorPositions.STOWED_DOWN).withEnableFOC(true);
  private final LoggedDashboardNumber ntHeight =
      new LoggedDashboardNumber("Elevator/HeightOverride", -1);

  private static final double PRE_MATCH_HOMING_MIN_MOVEMENT = 75.0;

  private double rotationsPerElevatorInch = 0.0;
  private double lowestSeenHeight = 0.0;
  private double highestSeenHeight = 0.0;
  private static double minHeight = 0.0;
  private static double maxHeight = 100.0;
  private double goalHeight = 50.0;
  private boolean preMatchHomingOccured = false;
  private static double StowedHeight = 10.0;
  private HomingState homingState = HomingState.PRE_MATCH_HOMING;
  private int slot = 0;

  public ElevatorSubsystem(TalonFX motor) {

    super(SubsystemPriority.ELEVATOR);
    this.motor = motor;
    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());

  }

  @Override
  public void disabledPeriodic() {

    double currentHeight = getHeight();

    if (currentHeight < lowestSeenHeight) {
      lowestSeenHeight = currentHeight;
    }

    if (currentHeight > highestSeenHeight) {
      highestSeenHeight = currentHeight;
    }
  }

  @Override
  public void robotPeriodic() {
    double rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    switch (homingState) {
      case NOT_HOMED:
        if (DriverStation.isDisabled()) {
          if (rangeOfMotionSeen()) {
            motor.setControl(brakeNeutralRequest);
          } else {
            motor.setControl(coastNeutralRequest);
          }
        }
        break;
      case PRE_MATCH_HOMING:
        if (DriverStation.isDisabled()) {
          if (rangeOfMotionSeen()) {
            motor.setControl(brakeNeutralRequest);

          } else {
            motor.setControl(coastNeutralRequest);
          }
        } else {
          motor.setControl(brakeNeutralRequest);

          if (!preMatchHomingOccured && rangeOfMotionSeen()) {
            // homingEndPosition + (currentAngle - minAngle)
            Rotation2d homingEndPosition = RobotConfig.get().elevator().homingEndPosition();
            Rotation2d homedAngle =
                Rotation2d.fromDegrees(
                    homingEndPosition.getDegrees() + (getHeight() - lowestSeenHeight));
            motor.setPosition(homedAngle.getRotations());

            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
        break;
      case MID_MATCH_HOMING:
        motor.set(RobotConfig.get().elevator().homingVoltage());

        if (filteredCurrent > RobotConfig.get().elevator().homingCurrentThreshold()) {
          homingState = HomingState.HOMED;
          goalHeight = StowedHeight;

          motor.setPosition(RobotConfig.get().elevator().homingEndPosition().getRotations());
        }

        break;
      case HOMED:
        double usedGoalHeight = clampHeight(ntHeight.get() == -1 ? goalHeight : ntHeight.get());

        slot = goalHeight == minHeight ? 1 : 0;
        Logger.recordOutput("Elevator/UsedGoalAngle", usedGoalHeight);

        motor.setControl(positionRequest.withSlot(slot).withPosition(usedGoalHeight));

        break;
    }
  }

  public boolean rangeOfMotionSeen() {
    return highestSeenHeight - lowestSeenHeight >= PRE_MATCH_HOMING_MIN_MOVEMENT;
  }

  public void setHeight(double height) {
    height = clampHeight(height);
    if (height == goalHeight) {
      if (height == minHeight) {

        motor.getConfigurator().apply(RobotConfig.get().elevator().strictCurrentLimits());

      } else {

        motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig().CurrentLimits);
      }
    }

    goalHeight = height;
  }

  private static double clampHeight(double height) {
    if (height < minHeight) {
      height = minHeight;

    } else if (height > maxHeight) {
      height = maxHeight;
    }
    return height;
  }

  public double getHeight() {
    return motor.getPosition().getValueAsDouble() / rotationsPerElevatorInch;
  }
}
