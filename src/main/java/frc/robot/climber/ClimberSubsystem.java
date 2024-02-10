// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends LifecycleSubsystem implements IClimberSubsystem {
  private static final Rotation2d TOLERANCE = Rotation2d.fromRotations(0.05);
  private final TalonFX mainMotor;
  private final TalonFX followerMotor;

  private ClimberMode goalMode = ClimberMode.IDLE;
  private Rotation2d goalPosition = Rotation2d.fromRotations(0);
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private boolean enabledOnce = false;

  public ClimberSubsystem(TalonFX mainMotor, TalonFX followerMotor) {
    super(SubsystemPriority.CLIMBER);
    this.mainMotor = mainMotor;
    this.followerMotor = followerMotor;

    mainMotor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());
    followerMotor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());

    mainMotor.setInverted(true);
    mainMotor.setPosition(0.0);
    followerMotor.setPosition(0.0);
  }

  public void setGoal(ClimberMode newMode) {
    goalMode = newMode;
  }

  public boolean atGoal(ClimberMode mode) {
    if (mode != goalMode) {
      return false;
    }

    if (goalMode == ClimberMode.IDLE) {
      return true;
    }

    if ((goalPosition.getRotations() - mainMotor.getPosition().getValue())
        < TOLERANCE.getRotations()) {
      return true;
    }

    return false;
  }

  @Override
  public void robotPeriodic() {
    enabledOnce = true;
    // Current
    Logger.recordOutput("Climber/MainMotor/Current", mainMotor.getStatorCurrent().getValue());
    Logger.recordOutput(
        "Climber/FollowerMotor/Current", followerMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Climber/MainMotor/OutputVoltage", mainMotor.getMotorVoltage().getValue());
    Logger.recordOutput(
        "Climber/FollowerMotor/OutputVoltage", followerMotor.getMotorVoltage().getValue());
    Logger.recordOutput("Climber/MainMotor/RPM", mainMotor.getVelocity().getValue() * 60.0);
    Logger.recordOutput("Climber/FollowerMotor/RPM", followerMotor.getVelocity().getValue() * 60.0);
    Logger.recordOutput("Climber/MainMotor/Position", mainMotor.getPosition().getValue());
    Logger.recordOutput("Climber/FollowerMotor/Position", followerMotor.getPosition().getValue());
    Logger.recordOutput("Climber/GoalPositionRotations", goalPosition.getRotations());
    Logger.recordOutput("Climber/GoalMode", goalMode);
    Logger.recordOutput("Climber/MainMotor/Tempereature", mainMotor.getDeviceTemp().getValue());
    Logger.recordOutput(
        "Climber/FollowerMotor/Tempereature", followerMotor.getDeviceTemp().getValue());

    if (goalMode == ClimberMode.IDLE) {
      goalPosition = Rotation2d.fromRotations(0);
    } else if (goalMode == ClimberMode.RAISED) {
      goalPosition = Rotation2d.fromRotations(6.5);
    } else if (goalMode == ClimberMode.HANGING) {
      goalPosition = Rotation2d.fromRotations(22);
    }
  }

  @Override
  public void enabledPeriodic() {
    // placeholder voltage
    if (atGoal(goalMode)) {
      if (enabledOnce) {
        mainMotor.setControl(brakeNeutralRequest);
        followerMotor.setControl(brakeNeutralRequest);
      } else {
        mainMotor.setControl(coastNeutralRequest);
        followerMotor.setControl(coastNeutralRequest);
      }

    } else {
      double currentPosition = mainMotor.getPosition().getValue();
      double goal = goalPosition.getRotations();
      double tolerance = TOLERANCE.getRotations();

      if (currentPosition < goal - tolerance) {
        mainMotor.setControl(voltageRequest.withOutput(12));
        followerMotor.setControl(voltageRequest.withOutput(12));
      } else if (currentPosition > goal - tolerance && goal > currentPosition) {
        mainMotor.setControl(voltageRequest.withOutput(3));
        followerMotor.setControl(voltageRequest.withOutput(3));
      } else {
        mainMotor.disable();
        followerMotor.disable();
      }
    }
  }
}
