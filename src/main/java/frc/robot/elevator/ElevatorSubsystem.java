// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;

  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final PositionVoltage positionRequest = new PositionVoltage(ElevatorPositions.STOWED);

  // Homing
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = 0.0;
  private HomingState homingState = HomingState.PRE_MATCH_HOMING;

  private double goalHeight = ElevatorPositions.STOWED;

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR);

    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());

    this.motor = motor;
  }

  @Override
  public void enabledInit() {
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void disabledPeriodic() {
    double currentHeight = getHeight();

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
        int slot = goalHeight == RobotConfig.get().elevator().minHeight() ? 1 : 0;

        motor.setControl(
            positionRequest
                .withSlot(slot)
                .withPosition(inchesToRotations(clampHeight(goalHeight)).getRotations()));

        break;
      case MID_MATCH_HOMING:
        throw new IllegalStateException("Elevator can't do mid match homing");
    }

    Logger.recordOutput("Elevator/Voltage", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Elevator/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Elevator/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Elevator/Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Elevator/Height", getHeight());
    Logger.recordOutput("Elevator/GoalHeight", goalHeight);
    Logger.recordOutput("Elevator/Rotations", getMechanismRotations().getRotations());
  }

  public void setGoalHeight(double newHeight) {
    goalHeight = clampHeight(newHeight);
  }

  public double getHeight() {
    return rotationsToInches(getMechanismRotations());
  }

  private Rotation2d getMechanismRotations() {
    return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public boolean atPosition(double distance) {
    return Math.abs(getHeight() - distance) < RobotConfig.get().elevator().tolerance();
  }

  private static double rotationsToInches(Rotation2d rotations) {
    return rotations.getRotations() * (RobotConfig.get().elevator().rotationsToDistance());
  }

  private static Rotation2d inchesToRotations(double inches) {
    return Rotation2d.fromRotations(inches / (RobotConfig.get().elevator().rotationsToDistance()));
  }

  private static double clampHeight(double height) {
    return MathUtil.clamp(
        height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }
}
