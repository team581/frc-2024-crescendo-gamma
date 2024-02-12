// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

public enum RobotState {
  UNHOMED,
  HOMING,

  IDLE_NO_GP,
  IDLE_WITH_GP,

  GROUND_INTAKING,

  OUTTAKING_SHOOTER,
  OUTTAKING_CONVEYOR,
  OUTTAKING_INTAKE,

  WAITING_FLOOR_SHOT,
  PREPARE_FLOOR_SHOT,
  FLOOR_SHOOT,

  /**
   * Get ready for subwoofer shot, wait for driver/operator (?) to confirm, then go to
   * PREPARE_SUBWOOFER_SHOT.
   */
  WAITING_SUBWOOFER_SHOT,
  /** Get ready for subwoofer shot, automatically go to SUBWOOFER_SHOOT when ready. */
  PREPARE_SUBWOOFER_SHOT,
  SUBWOOFER_SHOOT,

  PREPARE_TRAP_SHOT,
  TRAP_SHOOT,

  /** Get ready for speaker shot, wait for driver to confirm, then go to PREPARE_SPEAKER_SHOT. */
  WAITING_SPEAKER_SHOT,
  /** Get ready for speaker shot, automatically go to SPEAKER_SHOOT when ready. */
  PREPARE_SPEAKER_SHOT,
  SPEAKER_SHOOT,

  WAITING_AMP_SHOT,
  PREPARE_AMP_SHOT,
  AMP_SHOOT,

  /** Arm moves to chain height, climber hooks are touching the chain */
  WAITING_CLIMBER_RAISED,

  /** On the ground, arm goes up */
  PREPARE_CLIMBER_RAISED,
  CLIMBER_RAISED,

  /** Actually climbing then hanging */
  PREPARE_CLIMBER_HANGING,
  CLIMBER_HANGING;
}
