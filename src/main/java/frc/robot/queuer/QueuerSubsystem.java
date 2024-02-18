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
import org.littletonrobotics.junction.Logger;

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
  public void robotPeriodic() {
    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case INTAKING:
        if (hasNote()) {
          motor.disable();
        } else {
          motor.setControl(voltageRequest.withOutput(0.0));
        }
        break;
      case PASS_TO_INTAKE:
        if (hasNote()) {
          motor.setControl(voltageRequest.withOutput(0.0));
        } else {
          motor.disable();
        }
        break;
      case PASS_TO_SHOOTER:
        if (hasNote()) {
          motor.setControl(voltageRequest.withOutput(0.0));
        } else {
          motor.disable();
        }
        break;
      default:
        break;
    }

    Logger.recordOutput("Queuer/Voltage", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Queuer/State", goalState);
    Logger.recordOutput("Queuer/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Queuer/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Queuer/Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Queuer/HasNote", hasNote());
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
