// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.conveyor;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class ConveyorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private MotionMagicVelocityDutyCycle velocityRequest =
      new MotionMagicVelocityDutyCycle(0.00).withSlot(0);

  private ConveyorMode goalMode = ConveyorMode.IDLE;
  private boolean hasNote = false;
  private double goalVelocity = 0.00;

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
      velocityRequest.withVelocity(goalVelocity);
    }

    setHasNote(sensorHasNote());
  }

  @Override
  public void robotPeriodic() {
    if (goalMode == ConveyorMode.OUTTAKE) {
      goalVelocity = 0.20;
    } else if (goalMode == ConveyorMode.CONVEYORING) {
      goalVelocity = 0.35;
    } else if (goalMode == ConveyorMode.PASS_TO_INTAKE) {
      goalVelocity = -0.30;
    } else {
      goalVelocity = 0.00;
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
    if (goalMode == ConveyorMode.IDLE) {
      return true;
    }
    if (hasNote) {
      return true;
    }
    if (!hasNote && Math.abs(goalVelocity) > 0.00) {
      return true;
    }
    return false;
  }

  public void setHasNote(boolean bool) {
    hasNote = bool;
  }

  public boolean getHasNote() {
    return hasNote;
  }

  private boolean sensorHasNote() {
    return sensor.get();
  }
}
