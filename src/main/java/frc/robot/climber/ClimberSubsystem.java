// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ClimberSubsystem extends LifecycleSubsystem {
  private static final ClimberConfig CONFIG = RobotConfig.get().climber();
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private StrictFollower followRequest;
  private LinearFilter currentFilter = LinearFilter.movingAverage(CONFIG.currentTaps());
  private final LoggedDashboardNumber ntDistance =
      new LoggedDashboardNumber("Climber/PositionOverride", -1);
  private double goalDistance = 0.0;
  private PositionVoltage positionRequest = new PositionVoltage(goalDistance);
  private VoltageOut voltageRequest = new VoltageOut(0.0);

  private ClimberMode goalMode = ClimberMode.IDLE;
  private HomingState homingState = HomingState.NOT_HOMED;

  public ClimberSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.CLIMBER);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftMotor.getConfigurator().apply(CONFIG.leftMotorConfig());
    rightMotor.getConfigurator().apply(CONFIG.rightMotorConfig());

    followRequest = new StrictFollower(leftMotor.getDeviceID());
  }

  @Override
  public void robotPeriodic() {
    double rawCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    switch (homingState) {
      case NOT_HOMED:
        leftMotor.disable();
        rightMotor.disable();
        break;
      case MID_MATCH_HOMING:
        leftMotor.setControl(voltageRequest.withOutput(CONFIG.homingVoltage()));
        rightMotor.setControl(followRequest);
        if (filteredCurrent > CONFIG.homingCurrentThreshold()) {
          leftMotor.setPosition(0);
          rightMotor.setPosition(0);
          homingState = HomingState.HOMED;
        }
        break;
      case HOMED:
        switch (goalMode) {
          case IDLE:
            setGoalDistance(CONFIG.idlePosition());
            break;
          case RAISED:
            setGoalDistance(CONFIG.raisedPosition());
            break;
          case HANGING:
            setGoalDistance(CONFIG.hangingPosition());
            break;
          default:
            break;
        }
        double usedGoalDistance = clamp(ntDistance.get() == -1 ? goalDistance : ntDistance.get());

        Logger.recordOutput("Climber/UsedGoalPosition", usedGoalDistance);

        leftMotor.setControl(
            positionRequest.withPosition(inchesToRotations(usedGoalDistance).getRotations()));
        rightMotor.setControl(followRequest);

        break;
      case PRE_MATCH_HOMING:
        throw new IllegalStateException("Climber can't do pre match homing");
    }
  }

  public boolean atGoal(ClimberMode goal) {
    if (goalMode != goal) {
      return false;
    }
    if (goalMode == ClimberMode.IDLE) {
      return true;
    }
    if (goal == goalMode
        && Math.abs(
                rotationsToInches(
                    Rotation2d.fromRotations(leftMotor.getAcceleration().getValueAsDouble())))
            < CONFIG.accelerationTolerance()) {
      return true;
    }
    return false;
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public void resetHoming() {
    homingState = HomingState.NOT_HOMED;
  }

  public void startHoming() {
    homingState = HomingState.MID_MATCH_HOMING;
  }

  public double getDistance() {
    return rotationsToInches(Rotation2d.fromRotations(leftMotor.getPosition().getValueAsDouble()));
  }

  private void setGoalDistance(double distance) {
    goalDistance = clamp(distance);
  }

  public void setGoalMode(ClimberMode mode) {
    goalMode = mode;
  }

  private static double clamp(double pos) {
    return MathUtil.clamp(pos, CONFIG.minPosition(), CONFIG.maxPosition());
  }

  // Tune the radius in inches later
  private static double rotationsToInches(Rotation2d angle) {
    return angle.getRadians() * (CONFIG.rotationsToDistance());
  }

  private static Rotation2d inchesToRotations(double inches) {
    return Rotation2d.fromRadians(inches / (CONFIG.rotationsToDistance()));
  }
}
