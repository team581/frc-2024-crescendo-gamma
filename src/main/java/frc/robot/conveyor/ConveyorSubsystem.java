// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.conveyor;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class ConveyorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;

  private ConveyorMode goalMode = ConveyorMode.IDLE;
  private boolean holdingNote = false;
  private double goalPercentage = 0.00;

  public ConveyorSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.CONVEYOR);

    this.motor = motor;
    this.sensor = sensor;
  }

  @Override
  public void enabledPeriodic() {
    if (goalMode == ConveyorMode.IDLE) {
      motor.disable();
    } else {
      motor.set(goalPercentage);

      setHoldingNote(sensorHasNote());
    }
  }

  @Override
  public void robotPeriodic() {
    switch (goalMode) {
      case IDLE:
        goalPercentage = 0.0;
        break;
      case PASS_TO_CONVEYOR:
        goalPercentage = 0.0;
        break;
      case WAITING_AMP_SHOT:
        goalPercentage = 0.0;
        break;
      case AMP_SHOT:
        goalPercentage = 0.0;
        break;
      case PASS_TO_INTAKE:
      case PASS_TO_SHOOTER:
        goalPercentage = 0.0;
        break;
      default:
        break;
    }
  }

  public void setMode(ConveyorMode mode) {
    goalMode = mode;
  }

  public ConveyorMode getMode() {
    return goalMode;
  }

  public boolean atGoal(ConveyorMode mode) {
    if (goalMode != mode) {
      return false;
    }

    switch (goalMode) {
      case IDLE:
        return true;
      case AMP_SHOT:
      case PASS_TO_INTAKE:
      case PASS_TO_SHOOTER:
        return !holdingNote;
      case PASS_TO_CONVEYOR:
      case WAITING_AMP_SHOT:
        return holdingNote;
      default:
        break;
    }

    return false;
  }

  public void setHoldingNote(boolean newValue) {
    holdingNote = newValue;
  }

  public boolean hasNote() {
    return holdingNote;
  }

  private boolean sensorHasNote() {
    return sensor.get();
  }
}
