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
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final DigitalInput sensor;
  private IntakeState state = IntakeState.IDLE_NO_GP;
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  private boolean idleFlag = false;
  private boolean idleWithGPFlag = false;
  private boolean idleNoGPFlag = false;
  private boolean intakingFlag = false;
  private boolean outtakingFlag = false;
  private boolean shootingFlag = false;
  private boolean trapOuttakeFlag = false;
  private boolean climbingFlag = false;

  public IntakeSubsystem(TalonFX topMotor, TalonFX bottomMotor, DigitalInput sensor) {
    super(SubsystemPriority.INTAKE);

    this.topMotor = topMotor;
    this.sensor = sensor;
    this.bottomMotor = bottomMotor;

    topMotor.getConfigurator().apply(RobotConfig.get().intake().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().intake().bottomMotorConfig());
  }

  @Override
  public void enabledPeriodic() {
    Logger.recordOutput("Intake/State", state);

    Logger.recordOutput("Intake/IdleFlag", idleFlag);
    Logger.recordOutput("Intake/IdleWithGPFlag", idleWithGPFlag);
    Logger.recordOutput("Intake/IdleNoGPFlag", idleNoGPFlag);
    Logger.recordOutput("Intake/IntakingFlag", intakingFlag);
    Logger.recordOutput("Intake/OuttakingFlag", outtakingFlag);
    Logger.recordOutput("Intake/ShootingFlag", shootingFlag);
    Logger.recordOutput("Intake/TrapOuttakeFlag", trapOuttakeFlag);
    Logger.recordOutput("Intake/ClimbingFlag", climbingFlag);

    Logger.recordOutput("Intake/SensorHasNote", sensorHasNote());

    Logger.recordOutput("Intake/TopMotor/Current", topMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/TopMotor/Velocity", topMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Intake/TopMotor/OutputVoltage", topMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Intake/TopMotor/Temperature", topMotor.getDeviceTemp().getValue());

    Logger.recordOutput(
        "Intake/BottomMotor/Current", bottomMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Intake/BottomMotor/Velocity", bottomMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Intake/BottomMotor/OutputVoltage", bottomMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Intake/BottomMotor/Temperature", bottomMotor.getDeviceTemp().getValue());

    // State transitions from requests
    if (idleFlag) {
      switch (state) {
        case IDLE_NO_GP:
        case INTAKING_GP_WAITING_FOR_SENSOR_ON:
        case OUTTAKING:
        case SHOOTING:
        case TRAP_OUTTAKE:
          state = IntakeState.IDLE_NO_GP;
          break;

        default:
          state = IntakeState.IDLE_WITH_GP;
          break;
      }
    }
    if (intakingFlag) {
      // Don't interrupt the intake sequence if we are already in it
      if (state != IntakeState.INTAKING_GP_WAITING_FOR_SENSOR_OFF
          && state != IntakeState.INTAKING_GP_FINAL_WAITING_FOR_SENSOR_ON) {

        state = IntakeState.INTAKING_GP_WAITING_FOR_SENSOR_ON;
      }
    }
    if (outtakingFlag) {
      state = IntakeState.OUTTAKING;
    }
    if (trapOuttakeFlag) {
      state = IntakeState.TRAP_OUTTAKE;
    }
    if (climbingFlag) {
      state = IntakeState.CLIMBING;
    }
    if (shootingFlag) {
      state = IntakeState.SHOOTING;
    }
    if (idleWithGPFlag) {
      state = IntakeState.IDLE_WITH_GP;
    }
    if (idleNoGPFlag) {
      state = IntakeState.IDLE_NO_GP;
    }

    // Automatic state transitions
    switch (state) {
      case IDLE_WITH_GP:
      case CLIMBING:
      case IDLE_NO_GP:
        // Do nothing
        break;
      case INTAKING_GP_WAITING_FOR_SENSOR_ON:
        if (sensorHasNote()) {
          state = IntakeState.INTAKING_GP_WAITING_FOR_SENSOR_OFF;
        }
        break;
      case INTAKING_GP_WAITING_FOR_SENSOR_OFF:
        if (!sensorHasNote()) {
          state = IntakeState.INTAKING_GP_FINAL_WAITING_FOR_SENSOR_ON;
        }
        break;
      case INTAKING_GP_FINAL_WAITING_FOR_SENSOR_ON:
        if (sensorHasNote()) {
          state = IntakeState.IDLE_WITH_GP;
        }
        break;
      case OUTTAKING:
      case SHOOTING:
      case TRAP_OUTTAKE:
        if (!sensorHasNote()) {
          state = IntakeState.IDLE_NO_GP;
        }
        break;
    }

    // State actions

    // negative voltage intake
    // positive voltage outtake
    switch (state) {
      case IDLE_WITH_GP:
      case IDLE_NO_GP:
        topMotor.disable();
        bottomMotor.disable();
        break;
      case INTAKING_GP_WAITING_FOR_SENSOR_ON:
        topMotor.setControl(voltageRequest.withOutput(-3.5));
        bottomMotor.setControl(voltageRequest.withOutput(-3.5));
        break;
      case INTAKING_GP_WAITING_FOR_SENSOR_OFF:
        topMotor.setControl(voltageRequest.withOutput(-1.0));
        bottomMotor.setControl(voltageRequest.withOutput(-1.0));
        break;
      case INTAKING_GP_FINAL_WAITING_FOR_SENSOR_ON:
        topMotor.setControl(voltageRequest.withOutput(1.0));
        bottomMotor.setControl(voltageRequest.withOutput(1.0));
        break;
      case OUTTAKING:
        topMotor.setControl(voltageRequest.withOutput(6.0));
        bottomMotor.setControl(voltageRequest.withOutput(6.0));
        break;
      case SHOOTING:
        topMotor.setControl(voltageRequest.withOutput(-10.0));
        bottomMotor.setControl(voltageRequest.withOutput(-10.0));
        break;
      case TRAP_OUTTAKE:
        topMotor.setControl(voltageRequest.withOutput(6.0));
        bottomMotor.setControl(voltageRequest.withOutput(-6.0));
        break;
      case CLIMBING:
        topMotor.disable();
        bottomMotor.setControl(voltageRequest.withOutput(-10.0));
        break;
    }

    // Reset all flags
    idleFlag = false;
    idleNoGPFlag = false;
    idleWithGPFlag = false;
    intakingFlag = false;
    outtakingFlag = false;
    shootingFlag = false;
    trapOuttakeFlag = false;
    climbingFlag = false;
  }

  public void idleRequest() {
    idleFlag = true;
  }

  public void intakingRequest() {
    intakingFlag = true;
  }

  public void outtakingRequest() {
    outtakingFlag = true;
  }

  public void shootingRequest() {
    shootingFlag = true;
  }

  public void trapOuttakeRequest() {
    trapOuttakeFlag = true;
  }

  public void climbingRequest() {
    climbingFlag = true;
  }

  public void idleWithGPRequest() {
    idleWithGPFlag = true;
  }

  public void idleNoGPRequest() {
    idleNoGPFlag = true;
  }

  private boolean sensorHasNote() {
    return sensor.get();
  }

  public IntakeState getState() {
    return state;
  }
}
