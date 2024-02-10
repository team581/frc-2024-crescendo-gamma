// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

public enum IntakeState {
  IDLE_NO_GP,
  IDLE_WITH_GP,

  // Intaking, waitinf or sensor to trigger for the first time
  INTAKING_GP_WAITING_FOR_SENSOR_ON,
  // Intaking, wait for the note to pass all the way through, and stop triggering the sensor
  INTAKING_GP_WAITING_FOR_SENSOR_OFF,
  // Intaking in reverse, waiting for note to hit sensor again, then we transition to IDLE_WITH_GP
  INTAKING_GP_FINAL_WAITING_FOR_SENSOR_ON,

  OUTTAKING,
  SHOOTING,
  TRAP_OUTTAKE,
  CLIMBING;
}
