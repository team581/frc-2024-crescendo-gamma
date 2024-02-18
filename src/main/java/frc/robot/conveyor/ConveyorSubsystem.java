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

  private ConveyorState goalState = ConveyorState.IDLE;
  private double goalPercentage = 0.00;

  public ConveyorSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.CONVEYOR);

    this.motor = motor;
    this.sensor = sensor;

    // TODO: We don't actually set the motor config here
  }

  @Override
  public void enabledPeriodic() {
    if (goalState == ConveyorState.IDLE) {
      motor.disable();
    } else {
      motor.set(goalPercentage);
    }
  }

  @Override
  public void robotPeriodic() {
    // TODO: Move this logic to enabledPeriodic(). Stop using goalPercentage", just command the motor directly
    switch (goalState) {
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

  // TODO: Logging

  public void setState(ConveyorState state) {
    goalState = state;
  }

  public ConveyorState getMode() {
    return goalState;
  }

  public boolean hasNote() {
    return sensor.get();
  }
}
