// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private final Debouncer debouncer = RobotConfig.get().intake().debouncer();
  private boolean debouncedSensor = false;
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
        motor.setVoltage(-6);
        break;
      case FROM_QUEUER:
        motor.setVoltage(-4); // -3
        break;
      case FROM_CONVEYOR:
        motor.setVoltage(-8);
        break;
      case TO_QUEUER:
        if (hasNote()) {
          motor.setVoltage(10);
        } else {
          motor.setVoltage(12);
        }
        break;
      case TO_QUEUER_SLOW:
        if (hasNote()) {
          motor.setVoltage(10);
        } else {
          motor.setVoltage(5);
        }
        break;
      case TO_CONVEYOR:
        motor.setVoltage(3); // 2
        break;
      case TO_QUEUER_SHOOTING:
        motor.setVoltage(8);
        break;
      default:
        break;
    }
  }

  @Override
  public void robotPeriodic() {
    debouncedSensor = debouncer.calculate(sensorHasNote());
    Logger.recordOutput("Intake/State", goalState);
    Logger.recordOutput("Intake/DebouncedHasNote", debouncedSensor);
    Logger.recordOutput("Intake/HasNote", hasNote());
    Logger.recordOutput("Intake/SensorHasNote", sensorHasNote());
    Logger.recordOutput("Intake/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/Voltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public void setState(IntakeState state) {
    goalState = state;
  }

  public boolean hasNote() {
    switch (goalState) {
      case TO_QUEUER:
      case TO_QUEUER_SHOOTING:
      case TO_QUEUER_SLOW:
        // Bypass debouncer when we are sending game piece to queuer
        return sensorHasNote();
      default:
        return debouncedSensor;
    }
  }

  private boolean sensorHasNote() {
    return sensor.get();
  }

  public IntakeState getState() {
    return goalState;
  }
}
