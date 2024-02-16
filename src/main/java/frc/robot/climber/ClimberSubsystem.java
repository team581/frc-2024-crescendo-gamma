// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ClimberSubsystem extends LifecycleSubsystem {
  private final TalonFX mainMotor;
  private final TalonFX followerMotor;
  private Follower followRequest;
  private CoastOut coastNeutralRequest = new CoastOut();
  private StaticBrake brakeNeutralRequest = new StaticBrake();
  private LinearFilter currentFilter =
      LinearFilter.movingAverage(RobotConfig.get().climber().currentTaps());
  private final LoggedDashboardNumber ntposition =
      new LoggedDashboardNumber("Climber/positionOverride", -1);
  private int slot = 0;

  private double ACCEL_TOLERANCE = RobotConfig.get().climber().accelerationTolerance(); // inches per second
  private double goalPosition = 0.0;
  private PositionVoltage positionRequest = new PositionVoltage(goalPosition);
  private VoltageOut voltageRequest = new VoltageOut(0.0);

  private double IDLE = RobotConfig.get().climber().idlePosition();
  private double RAISED = RobotConfig.get().climber().raisedPosition();
  private double HANGING = RobotConfig.get().climber().hangingPosition();

  private ClimberMode goalMode = ClimberMode.IDLE;
  private HomingState homingState = HomingState.NOT_HOMED;
  private double homingCurrentThreshold = RobotConfig.get().climber().homingCurrentThreshold();
  private double homingVoltage = RobotConfig.get().climber().homingVoltage();
  private boolean preMatchHomingOccured = false;

  private double maxPosition = RobotConfig.get().climber().maxPosition();
  private double minPositon = RobotConfig.get().climber().minPosition();

  public ClimberSubsystem(TalonFX mainMotor, TalonFX followerMotor) {
    super(SubsystemPriority.CLIMBER);

    this.mainMotor = mainMotor;
    this.followerMotor = followerMotor;

    mainMotor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());
    followerMotor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());

    followRequest =
        new Follower(mainMotor.getDeviceID(), RobotConfig.get().climber().opposeMasterDirection());
  }

  @Override
  public void robotPeriodic() {
    double rawCurrent = mainMotor.getStatorCurrent().getValueAsDouble();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    switch (homingState) {
      case NOT_HOMED:
        mainMotor.setControl(coastNeutralRequest);
        followerMotor.setControl(coastNeutralRequest);
        break;
      case PRE_MATCH_HOMING:
        if (DriverStation.isDisabled()) {
          mainMotor.setControl(coastNeutralRequest);
          followerMotor.setControl(coastNeutralRequest);
        } else {
          mainMotor.setControl(brakeNeutralRequest);
          followerMotor.setControl(brakeNeutralRequest);

          if (!preMatchHomingOccured) {
            mainMotor.setControl(voltageRequest.withOutput(homingVoltage));
            followerMotor.setControl(followRequest);
            if (filteredCurrent > homingCurrentThreshold) {
            mainMotor.setPosition(inchesToRotations(0.0));
            followerMotor.setPosition(inchesToRotations(0.0));

            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
      }
        break;
      case MID_MATCH_HOMING:
        if (preMatchHomingOccured) {
            mainMotor.setControl(voltageRequest.withOutput(homingVoltage));
            followerMotor.setControl(followRequest);
            if (filteredCurrent > homingCurrentThreshold) {
            mainMotor.setPosition(inchesToRotations(0.0));
            followerMotor.setPosition(inchesToRotations(0.0));
            homingState = HomingState.HOMED;
          }
        }
        break;
      case HOMED:
        switch (goalMode) {
          case IDLE:
            setGoalPosition(IDLE);
            break;
          case RAISED:
            setGoalPosition(RAISED);
            break;
          case HANGING:
            setGoalPosition(HANGING);
            break;
          default:
          break;
        }
        double usedGoalPosition = ntposition.get() == -1 ? clamp(goalPosition) : ntposition.get();

        slot = goalPosition == minPositon ? 1 : 0;
        Logger.recordOutput("Climber/UsedGoalPosition", usedGoalPosition);

        mainMotor.setControl(
            positionRequest.withSlot(slot).withPosition(inchesToRotations(usedGoalPosition)));
        followerMotor.setControl(followRequest);

        break;
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
        && Math.abs(rotationsToInches(mainMotor.getAcceleration().getValueAsDouble()))
            < ACCEL_TOLERANCE) {
      return true;
    }
    return false;
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

  public double getPosition() {
    return rotationsToInches(mainMotor.getPosition().getValueAsDouble());
  }

  private void setGoalPosition(double pos) {
    goalPosition = clamp(pos);
  }

  public void setGoalMode(ClimberMode mode) {
    goalMode = mode;
  }

  private double clamp(double pos) {
    return pos < minPositon ? minPositon : pos > maxPosition ? maxPosition : pos;
  }

  // Tune the radius in inches later
  private double rotationsToInches(double rotations) {
    return rotations * (2 * Math.PI * 0);
  }

  private double inchesToRotations(double inches) {
    return inches / (2 * Math.PI * 0);
  }
}
