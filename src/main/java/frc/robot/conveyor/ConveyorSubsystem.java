// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.conveyor;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ConveyorSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private final Debouncer handoffDebouncer = RobotConfig.get().conveyor().handoffDebouncer();
  private final Debouncer scoringDebouncer = RobotConfig.get().conveyor().scoringDebouncer();
  private final Timer timer = new Timer();
  private boolean handoffDebouncedSensor = false;
  private boolean scoringDebouncedSensor = false;
  private ConveyorState goalState = ConveyorState.IDLE;

  public ConveyorSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.CONVEYOR);

    motor.getConfigurator().apply(RobotConfig.get().conveyor().motorConfig());

    this.motor = motor;
    this.sensor = sensor;

    timer.start();
  }

  @Override
  public void robotPeriodic() {
    scoringDebouncedSensor = scoringDebouncer.calculate(sensorHasNote());
    handoffDebouncedSensor = handoffDebouncer.calculate(sensorHasNote());
    Logger.recordOutput("Conveyor/State", goalState);
    Logger.recordOutput("Conveyor/SensorHasNote", sensorHasNote());
    Logger.recordOutput("Conveyor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Conveyor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Conveyor/Voltage", motor.getMotorVoltage().getValueAsDouble());

    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case INTAKE_TO_SELF:
        motor.setVoltage(-12);
        break;
      case WAITING_AMP_SHOT:
        if (hasNote()) {
          motor.disable();
        } else {
          motor.setVoltage(-8);
        }
        break;
      case INTAKE_TO_QUEUER:
        motor.setVoltage(3);
        break;
      case SHUFFLE_ASSIST_WHEN_QUEUER_SENSOR_TURNS_OFF:
        motor.setVoltage(4);
        break;
      case QUEUER_TO_INTAKE:
        motor.setVoltage(-12);
        break;
      case CONVEYOR_TO_INTAKE:
        motor.setVoltage(3);
        break;
      case AMP_SHOT:
        motor.setVoltage(-12);
        break;
      case TRAP_SHOT_PULSE:
        var inVoltage = 4;
        var outVoltage = -12;
        if (timer.hasElapsed(RobotConfig.get().conveyor().pulseDuration())) {
          if (timer.hasElapsed(RobotConfig.get().conveyor().pulseDuration() * 2.0)) {
            motor.setVoltage(outVoltage);
            timer.reset();
          } else {
            motor.setVoltage(inVoltage);
          }
        } else {
          motor.setVoltage(outVoltage);
        }
        break;
      case SHUFFLE:
        motor.setVoltage(-0.5);
        break;
      default:
        break;
    }
  }

  public void setState(ConveyorState state) {
    goalState = state;
  }

  public ConveyorState getState() {
    return goalState;
  }

  public boolean hasNote() {
    switch (goalState) {
      case AMP_SHOT:
        return scoringDebouncedSensor;
      default:
        return handoffDebouncedSensor;
    }
  }

  private boolean sensorHasNote() {
    return sensor.get();
  }
}
