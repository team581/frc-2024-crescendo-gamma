// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

public enum RobotState {
  UNHOMED(false, false),
  HOMING(false, false),

  IDLE_NO_GP(false),
  IDLE_WITH_GP(true),

  GROUND_INTAKING(false),

  OUTTAKING_SHOOTER(true),
  OUTTAKING_INTAKE(true),

  QUEUER_TO_INTAKE_FOR_AMP(true),
  CONVEYOR_TO_INTAKE_FOR_SHOOTER(true),
  INTAKE_TO_QUEUER_FOR_SHOOTER(true),
  INTAKE_TO_CONVEYOR_FOR_AMP(true),

  WAITING_FLOOR_SHOT(true),
  PREPARE_FLOOR_SHOT(true),
  FLOOR_SHOOT(true),

  /**
   * Get ready for subwoofer shot, wait for driver/operator (?) to confirm, then go to
   * PREPARE_SUBWOOFER_SHOT.
   */
  WAITING_SUBWOOFER_SHOT(true),
  /** Get ready for subwoofer shot, automatically go to SUBWOOFER_SHOOT when ready. */
  PREPARE_SUBWOOFER_SHOT(true),
  SUBWOOFER_SHOOT(true),

  PREPARE_TRAP_OUTTAKE(true),
  TRAP_OUTTAKE(true),

  /** Get ready for speaker shot, wait for driver to confirm, then go to PREPARE_SPEAKER_SHOT. */
  WAITING_SPEAKER_SHOT(true),
  /** Get ready for speaker shot, automatically go to SPEAKER_SHOOT when ready. */
  PREPARE_SPEAKER_SHOT(true),
  SPEAKER_SHOOT(true),

  WAITING_AMP_SHOT(true),
  AMP_SHOT(true),

  /** Arm moves to chain height, climber hooks are touching the chain */
  WAITING_CLIMBER_RAISED(true),

  /** On the ground, arm goes up */
  PREPARE_CLIMBER_RAISED(true),
  CLIMBER_RAISED(true),

  /** Actually climbing then hanging */
  PREPARE_CLIMBER_HANGING(true),
  CLIMBER_HANGING(true);

  public final boolean hasNote;
  public final boolean homed;

  RobotState(boolean hasNote) {
    this(hasNote, true);
  }

  RobotState(boolean hasNote, boolean homed) {
    this.hasNote = hasNote;
    this.homed = homed;
  }
}
