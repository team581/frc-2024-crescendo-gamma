// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.conveyor;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ConveyorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final Debouncer debouncer =
      new Debouncer(
          RobotConfig.get().conveyor().debounceTime(), RobotConfig.get().conveyor().debounceType());
  private boolean debouncedSensor = false;
  private ConveyorState goalState = ConveyorState.IDLE;

  public ConveyorSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.CONVEYOR);

    motor.getConfigurator().apply(RobotConfig.get().conveyor().motorConfig());

    this.motor = motor;
    this.sensor = sensor;
  }

  @Override
  public void enabledPeriodic() {
    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case INTAKE_TO_SELF:
      case WAITING_AMP_SHOT:
        if (hasNote()) {
          motor.disable();
        } else {
          motor.setControl(voltageRequest.withOutput(-4));
        }
        break;
      case INTAKE_TO_QUEUER:
        motor.setControl(voltageRequest.withOutput(3));
        break;
      case QUEUER_TO_INTAKE:
        motor.setControl(voltageRequest.withOutput(-3));
        break;
      case AMP_SHOT:
        motor.setControl(voltageRequest.withOutput(-3));
        break;
      default:
        break;
    }
  }

  @Override
  public void robotPeriodic() {
    debouncedSensor = debouncer.calculate(sensorHasNote());
    Logger.recordOutput("Conveyor/State", goalState);
    Logger.recordOutput("Conveyor/DebouncedHasNote", hasNote());
    Logger.recordOutput("Conveyor/SensorHasNote", sensorHasNote());
    Logger.recordOutput("Conveyor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Conveyor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Conveyor/Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Conveyor/Voltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public void setState(ConveyorState state) {
    goalState = state;
  }

  public ConveyorState getState() {
    return goalState;
  }

  public boolean hasNote() {
    return debouncedSensor;
  }

  private boolean sensorHasNote() {
    return sensor.get();
  }
}
