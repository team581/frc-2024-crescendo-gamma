package frc.robot.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private VoltageOut voltageRequest = new VoltageOut(0.0);
  private IntakeState goalState = IntakeState.IDLE;
  private double voltageUsed = 0.0;
  private boolean hasNote = false;

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
        voltageUsed = 0.0;
        break;
      case OUTTAKING:
        voltageUsed = -4.0;
        break;
      case PASS_TO_CONVEYOR:
        voltageUsed = 2.0;
        break;
      case PASS_TO_QUEUER:
        voltageUsed = 2.0;
        break;
      case INTAKING:
        voltageUsed = 4.0;
        break;
      default:
        break;
    }
    if (goalState == IntakeState.IDLE) {
      motor.disable();
    } else {
      voltageRequest.withOutput(voltageUsed);
    }
    setHasNote(sensorHasNote());
  }

  public void setState(IntakeState state) {
    goalState = state;
  }

  public boolean atGoal(IntakeState state) {
    if (goalState != state) {
      return false;
    }

    if (goalState == IntakeState.IDLE) {
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

  public IntakeState getState() {
    return goalState;
  }
}
