// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private VoltageOut voltageRequest = new VoltageOut(0.0);
  private IntakeState goalState = IntakeState.IDLE;

  public IntakeSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.INTAKE);

    motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());

    this.motor = motor;
    this.sensor = sensor;
  }

  @Override
  public void enabledPeriodic() {
    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case OUTTAKING:
        if (hasNote()) {
          motor.setControl(voltageRequest.withOutput(0));

        } else {
          motor.disable();
        }
        break;
      case FROM_QUEUER:
      case FROM_CONVEYOR:
        if (hasNote()) {
          motor.disable();
        } else {
          motor.setControl(voltageRequest.withOutput(0));
        }
        break;
      case TO_QUEUER:
      case TO_CONVEYOR:
        if (hasNote()) {
          motor.setControl(voltageRequest.withOutput(0));
        } else {
          motor.disable();
        }
        break;
      case TO_QUEUER_SHOOTING:
        if (hasNote()) {
          motor.setControl(voltageRequest.withOutput(0));
        } else {
          motor.disable();
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Intake/State", goalState);
    Logger.recordOutput("Intake/HasNote", hasNote());
    Logger.recordOutput("Intake/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/Voltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public void setState(IntakeState state) {
    goalState = state;
  }

  public boolean hasNote() {
    return sensor.get();
  }

  public IntakeState getState() {
    return goalState;
  }
}
