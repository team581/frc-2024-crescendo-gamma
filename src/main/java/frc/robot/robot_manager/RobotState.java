// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lights.BlinkPattern;
import frc.robot.lights.LightsState;

public enum RobotState {
  /** Idling without a note. */
  IDLE_NO_GP(false, false, new LightsState(Color.kOrangeRed, BlinkPattern.BLINK_SLOW)),
  /** Idling with a note in the conveyor, going to stow everything. */
  PREPARE_IDLE_WITH_GP_FROM_CONVEYOR(
      true, false, new LightsState(Color.kOrangeRed, BlinkPattern.SOLID)),
  /** Idling with a note in the queuer. */
  IDLE_WITH_GP(true, false, new LightsState(Color.kOrangeRed, BlinkPattern.SOLID)),

  /** Intaking a game piece. Transition to INTAKE_TO_QUEUER when done. */
  INTAKING(false, false, new LightsState(Color.kOrangeRed, BlinkPattern.BLINK_SLOW)),

  INTAKING_SLOW(false, false, new LightsState(Color.kOrangeRed, BlinkPattern.BLINK_SLOW)),

  /** Outtaking via the shooter. Game piece should be in queuer at start. */
  OUTTAKING_SHOOTER(true, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),
  /** Outtaking via the intake. Game piece should be in queuer at start. */
  OUTTAKING(true, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_FAST)),

  /** Preparing for floor shot, waiting for driver to commit. */
  WAITING_FLOOR_SHOT(true, false, new LightsState(null, BlinkPattern.SOLID)),
  /** Preparing for floor shot, should shoot when ready. */
  PREPARE_FLOOR_SHOT(true, true, new LightsState(null, BlinkPattern.SOLID)),
  /** Actively doing the floor shot. */
  FLOOR_SHOOT(true, true, new LightsState(null, BlinkPattern.BLINK_FAST)),

  /** Get ready for subwoofer shot, wait for drive to confirm, then go to PREPARE_SUBWOOFER_SHOT. */
  WAITING_SUBWOOFER_SHOT(true, false, new LightsState(null, BlinkPattern.SOLID)),
  /** Get ready for subwoofer shot, automatically go to SUBWOOFER_SHOOT when ready. */
  PREPARE_SUBWOOFER_SHOT(true, true, new LightsState(null, BlinkPattern.SOLID)),
  /** Actively doing the subwoofer shot. */
  SUBWOOFER_SHOOT(true, true, new LightsState(null, BlinkPattern.BLINK_FAST)),

  /** Get ready for speaker shot, wait for driver to confirm, then go to PREPARE_SPEAKER_SHOT. */
  WAITING_SPEAKER_SHOT(true, false, new LightsState(null, BlinkPattern.SOLID)),
  /** Get ready for speaker shot, automatically go to SPEAKER_SHOOT when ready. */
  PREPARE_SPEAKER_SHOT(true, true, new LightsState(null, BlinkPattern.SOLID)),
  SPEAKER_SHOOT(true, true, new LightsState(null, BlinkPattern.BLINK_FAST)),

  /** Note maybe in queuer, need to move it to conveyor, and then transition to WAITING_AMP_SHOT. */
  PREPARE_WAITING_AMP_SHOT(true, false, new LightsState(null, BlinkPattern.SOLID)),
  /** Note in conveyor, waiting for driver to commit to amp score. */
  WAITING_AMP_SHOT(true, false, new LightsState(null, BlinkPattern.SOLID)),
  /** Get ready for amp shot, automatically go to AMP_SHOT when ready. */
  PREPARE_AMP_SHOT(true, false, new LightsState(null, BlinkPattern.SOLID)),
  /** Actively scoring in the amp. */
  AMP_SHOT(true, false, new LightsState(null, BlinkPattern.BLINK_FAST)),

  /** Hooks move to top of the robot so we can start to grab the chain. */
  CLIMB_1_LINEUP_OUTER(true, false, new LightsState(Color.kRed, BlinkPattern.SOLID)),

  /** Climber moves down a little, so you can fully grab the chain. */
  CLIMB_2_LINEUP_INNER(true, false, new LightsState(Color.kOrangeRed, BlinkPattern.SOLID)),

  /** Elevator goes up and we do final alignment before climbing. */
  CLIMB_3_LINEUP_FINAL(true, false, new LightsState(Color.kYellow, BlinkPattern.SOLID)),

  PREPARE_CLIMB_4_HANGING(true, false, new LightsState(Color.kGreen, BlinkPattern.BLINK_SLOW)),
  /** Hooks go all the way down, we are fully hanging. */
  CLIMB_4_HANGING(true, false, new LightsState(Color.kGreen, BlinkPattern.SOLID)),

  PREPARE_CLIMB_5_HANGING_TRAP_SCORE(
      true, false, new LightsState(Color.kBlue, BlinkPattern.BLINK_SLOW)),
  CLIMB_5_HANGING_TRAP_SCORE(true, false, new LightsState(Color.kBlue, BlinkPattern.BLINK_FAST)),
  CLIMB_6_HANGING_FINISHED(false, false, new LightsState(Color.kIndigo, BlinkPattern.SOLID));

  public final boolean hasNote;
  public final boolean shootingMode;
  public final LightsState lightsState;

  RobotState(boolean hasNote, boolean shootingMode, LightsState lights) {
    this.hasNote = hasNote;
    this.shootingMode = shootingMode;
    this.lightsState = lights;
  }
}
