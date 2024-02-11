// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class QueuerSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private QueuerState goalState = QueuerState.IDLE;
  private boolean hasNote = false;

  public QueuerSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.QUEUER);

    motor.getConfigurator().apply(RobotConfig.get().queuer().motorConfig());

    this.sensor = sensor;
    this.motor = motor;
  }

  @Override
  public void enabledPeriodic() {
    if (goalState == QueuerState.IDLE) {
      motor.disable();
    } else if (goalState == QueuerState.PASSING_NOTE) {
      voltageRequest.withOutput(3.0);
    } else if (goalState == QueuerState.PASS_TO_INTAKE) {
      voltageRequest.withOutput(-3.0);
    }

    setHasNote(sensorHasNote());
  }

  public void setState(QueuerState state) {
    goalState = state;
  }

  public boolean atGoal(QueuerState state) {
    if (goalState != state) {
      return false;
    }

    if (goalState == QueuerState.IDLE) {
      return true;
    }
    if (hasNote) {
      return true;
    }
    if (Math.abs(motor.getMotorVoltage().getValue()) > 0.0 && !hasNote) {
      return true;
    }
    return false;
  }

  public boolean getHasNote() {
    return hasNote;
  }

  public void setHasNote(boolean bool) {
    hasNote = bool;
  }

  private boolean sensorHasNote() {
    return sensor.get();
  }

  public QueuerState getState() {
    return goalState;
  }
}
