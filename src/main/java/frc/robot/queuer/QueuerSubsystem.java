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
    } else if (goalState == QueuerState.PASS_TO_SHOOTER) {
      voltageRequest.withOutput(0.0);
    } else if (goalState == QueuerState.PASS_TO_INTAKE) {
      voltageRequest.withOutput(0.0);
    }
  }

  public void setState(QueuerState state) {
    goalState = state;
  }

  public boolean hasNote() {
    return sensor.get();
  }

  public QueuerState getState() {
    return goalState;
  }
}
