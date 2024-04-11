// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class QueuerSubsystem extends LifecycleSubsystem {
  private static final double NOTE_SHUFFLE_ON_DURATION = 0.5;
  private static final double NOTE_SHUFFLE_OFF_DURATION = 0.5;
  private static final double NOTE_SHUFFLE_TIMEOUT_DURATION = 5;
  private boolean noteShuffleOn = false;
  private final TalonFX motor;
  private final DigitalInput sensor;
  private QueuerState goalState = QueuerState.IDLE;
  private final Debouncer debouncer = RobotConfig.get().queuer().debouncer();
  private boolean debouncedSensor = false;
  private final Timer shuffleTimer = new Timer();
  private final Timer shuffleTimeoutTimer = new Timer();

  public QueuerSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.QUEUER);

    motor.getConfigurator().apply(RobotConfig.get().queuer().motorConfig());

    this.sensor = sensor;
    this.motor = motor;

    shuffleTimer.start();

    shuffleTimeoutTimer.start();
  }

  @Override
  public void enabledPeriodic() {
    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case INTAKING:
        if (hasNote()) {
          motor.disable();
        } else {
        shuffleTimeoutTimer.reset();
          motor.setVoltage(1);
        }
        break;
      case SHUFFLE:
        if (shuffleTimeoutTimer.hasElapsed(NOTE_SHUFFLE_TIMEOUT_DURATION)) {
        motor.disable();
        } else {
            if (sensorHasNote()) {
            if (noteShuffleOn) {
              // Push note towards intake
              motor.setVoltage(-1.5);

              if (shuffleTimer.hasElapsed(NOTE_SHUFFLE_ON_DURATION)) {
                shuffleTimer.reset();
                noteShuffleOn = false;
              }
            } else {
              // Allow note to expand
              motor.disable();

              if (shuffleTimer.hasElapsed(NOTE_SHUFFLE_OFF_DURATION)) {
                shuffleTimer.reset();
                noteShuffleOn = true;
              }
            }
          } else {
            // Note is out of sensor range, try intaking it
            motor.setVoltage(1);
          }
        }
        break;
      case PASS_TO_INTAKE:
        motor.setVoltage(-1);
        shuffleTimeoutTimer.reset();
        break;
      case PASS_TO_SHOOTER:
        motor.setVoltage(12);
        shuffleTimeoutTimer.reset();
        break;
      case PASS_TO_CONVEYOR:
        motor.setVoltage(-8); // TODO: adjust voltage
        break;
      default:
        break;
    }
  }

  @Override
  public void robotPeriodic() {
    debouncedSensor = debouncer.calculate(sensorHasNote());
    Logger.recordOutput("Queuer/Voltage", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Queuer/State", goalState);
    Logger.recordOutput("Queuer/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Queuer/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Queuer/Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Queuer/DebouncedHasNote", debouncer.calculate(sensorHasNote()));
    Logger.recordOutput("Queuer/SensorHasNote", sensorHasNote());
  }

  public void setState(QueuerState state) {
    goalState = state;
  }

  public boolean hasNote() {
    return debouncedSensor;
  }

  public boolean sensorHasNote() {
    return sensor.get();
  }

  public QueuerState getState() {
    return goalState;
  }
}
