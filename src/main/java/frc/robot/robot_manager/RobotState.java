// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lights.BlinkPattern;
import frc.robot.lights.LightsState;
import java.util.Optional;

public enum RobotState {
  /** Not homed. */
  UNHOMED(false, false, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_SLOW)),
  /** Homing. */
  HOMING(false, false, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_SLOW)),

  /** Idling without a note. */
  IDLE_NO_GP(false, false, new LightsState(Color.kOrangeRed, BlinkPattern.BLINK_SLOW)),
  /** Idling with a note in the conveyor, going to stow everything. */
  PREPARE_IDLE_WITH_GP_FROM_CONVEYOR(
      true, false, new LightsState(Color.kOrangeRed, BlinkPattern.SOLID)),
  /** Idling with a note in the queuer. */
  IDLE_WITH_GP(true, false, new LightsState(Color.kOrangeRed, BlinkPattern.SOLID)),

  /** Intaking a game piece. Transition to INTAKE_TO_QUEUER when done. */
  GROUND_INTAKING(
      false,
      false,
      new LightsState(Color.kOrangeRed, BlinkPattern.BLINK_SLOW),
      new LightsState(Color.kWhite, BlinkPattern.BLINK_FAST)),

  /** Outtaking via the shooter. Game piece should be in queuer at start. */
  OUTTAKING_SHOOTER(true, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),
  /** Outtaking via the intake. Game piece should be in queuer at start. */
  OUTTAKING(true, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),

  /** Preparing for floor shot, waiting for driver to commit. */
  WAITING_FLOOR_SHOT(true, false, new LightsState(Color.kGreen, BlinkPattern.SOLID)),
  /** Preparing for floor shot, should shoot when ready. */
  PREPARE_FLOOR_SHOT(true, true, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),
  /** Actively doing the floor shot. */
  FLOOR_SHOOT(
      true,
      true,
      new LightsState(Color.kGreen, BlinkPattern.BLINK_SLOW),
      new LightsState(Color.kWhite, BlinkPattern.BLINK_FAST)),

  /**
   * , Get ready for subwoofer shot, wait for driver/operator (?) to confirm, then go to
   * PREPARE_SUBWOOFER_SHOT.
   */
  WAITING_SUBWOOFER_SHOT(true, false, new LightsState(Color.kOrange, BlinkPattern.SOLID)),
  /** Get ready for subwoofer shot, automatically go to SUBWOOFER_SHOOT when ready. */
  PREPARE_SUBWOOFER_SHOT(true, true, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),
  /** Actively doing the subwoofer shot. */
  SUBWOOFER_SHOOT(
      true,
      true,
      new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST),
      new LightsState(Color.kWhite, BlinkPattern.BLINK_FAST)),

  PREPARE_TRAP_OUTTAKE(true, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_SLOW)),
  TRAP_OUTTAKE(
      true,
      false,
      new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST),
      new LightsState(Color.kWhite, BlinkPattern.BLINK_FAST)),

  /** Get ready for speaker shot, wait for driver to confirm, then go to PREPARE_SPEAKER_SHOT. */
  WAITING_SPEAKER_SHOT(true, false, new LightsState(Color.kOrange, BlinkPattern.SOLID)),
  /** Get ready for speaker shot, automatically go to SPEAKER_SHOOT when ready. */
  PREPARE_SPEAKER_SHOT(true, true, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),
  SPEAKER_SHOOT(
      true,
      true,
      new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST),
      new LightsState(Color.kWhite, BlinkPattern.BLINK_FAST)),

  /** Note maybe in queuer, need to move it to conveyor, and then transition to WAITING_AMP_SHOT. */
  PREPARE_WAITING_AMP_SHOT(true, false, new LightsState(Color.kGreen, BlinkPattern.SOLID)),
  /** Note in conveyor, waiting for driver to commit to amp score. */
  WAITING_AMP_SHOT(true, false, new LightsState(Color.kGreen, BlinkPattern.SOLID)),
  /** Get ready for amp shot, automatically go to AMP_SHOT when ready. */
  PREPARE_AMP_SHOT(true, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),
  /** Actively scoring in the amp. */
  AMP_SHOT(
      true,
      false,
      new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST),
      new LightsState(Color.kWhite, BlinkPattern.BLINK_FAST)),

  /**
   * Ensures that the shooter is stowed before transitioning to WAITING_CLIMBER_RAISED and moving
   * the elevator up.
   */
  PREPARE_WAITING_CLIMBER_RAISED(
      true, false, new LightsState(Color.kCyan, BlinkPattern.BLINK_SLOW)),
  /** Arm moves to chain height, climber hooks are touching the chain */
  WAITING_CLIMBER_RAISED(true, false, new LightsState(Color.kCyan, BlinkPattern.SOLID)),

  /** On the ground, arm goes up */
  PREPARE_CLIMBER_RAISED(true, false, new LightsState(Color.kCyan, BlinkPattern.BLINK_SLOW)),
  CLIMBER_RAISED(true, false, new LightsState(Color.kCyan, BlinkPattern.BLINK_FAST)),

  /** Actually climbing then hanging */
  PREPARE_CLIMBER_HANGING(true, false, new LightsState(Color.kCyan, BlinkPattern.BLINK_SLOW)),
  CLIMBER_HANGING(true, false, new LightsState(Color.kCyan, BlinkPattern.BLINK_FAST));

  public final boolean hasNote;
  public final boolean homed;
  public final boolean shootingMode;
  public final LightsState lightsState;
  public final Optional<LightsState> lightsOnExit;

  RobotState(boolean hasNote, boolean shootingMode, LightsState lights, LightsState lightsOnExit) {
    this(hasNote, true, shootingMode, lights, Optional.of(lightsOnExit));
  }

  RobotState(boolean hasNote, boolean shootingMode, LightsState lights) {
    this(hasNote, true, shootingMode, lights, Optional.empty());
  }

  RobotState(boolean hasNote, boolean homed, boolean shootingMode, LightsState lights) {
    this(hasNote, homed, shootingMode, lights, Optional.empty());
  }

  RobotState(
      boolean hasNote,
      boolean homed,
      boolean shootingMode,
      LightsState lights,
      Optional<LightsState> lightsOnExit) {
    this.hasNote = hasNote;
    this.homed = homed;
    this.shootingMode = shootingMode;
    this.lightsState = lights;
    this.lightsOnExit = lightsOnExit;
  }
}
