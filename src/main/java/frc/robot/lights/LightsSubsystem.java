// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.util.logging.advantagekit.Logger;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;
import java.util.Optional;

public class LightsSubsystem extends LifecycleSubsystem {
  private static final LightsState FLASH_LIGHTS =
      new LightsState(Color.kWhite, BlinkPattern.BLINK_FAST);
  private static final double DRIVER_SIGNAL_BLINK_DURATION = 1;
  private static final double FAST_BLINK_DURATION = 0.08;
  private static final double SLOW_BLINK_DURATION = 0.25;

  private final CANdle candle;
  private final RobotManager robotManager;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private final Timer blinkTimer = new Timer();
  private final Timer lightsOnExitTimer = new Timer();

  private LightsState state = new LightsState(Color.kWhite, BlinkPattern.SOLID);
  private Optional<LightsState> lightsOnExit = Optional.empty();
  private RobotState previousState = RobotState.IDLE_NO_GP;

  public LightsSubsystem(
      CANdle candle, RobotManager robotManager, VisionSubsystem vision, IntakeSubsystem intake) {
    super(SubsystemPriority.LIGHTS);

    this.candle = candle;
    this.robotManager = robotManager;
    this.vision = vision;
    this.intake = intake;

    blinkTimer.start();

    CANdleConfiguration config = new CANdleConfiguration();
    config.disableWhenLOS = true;

    candle.configAllSettings(config);
  }

  private Color getVisionLightsColor() {
    if (vision.getState() == VisionState.SEES_TAGS) {
      return Color.kGreen;
    } else if (vision.getState() == VisionState.ONLINE_NO_TAGS) {
      return Color.kYellow;
    } else {
      return Color.kRed;
    }
  }

  @Override
  public void robotPeriodic() {
    RobotState robotState = robotManager.getState();

    if (DriverStation.isDisabled()) {
      state =
          new LightsState(
              getVisionLightsColor(),
              vision.getState() == VisionState.OFFLINE
                  ? BlinkPattern.BLINK_SLOW
                  : BlinkPattern.SOLID);
    } else {
      switch (previousState) {
        case INTAKING:
        case INTAKING_SLOW:
          if (intake.hasNote()) {
            lightsOnExit = Optional.of(FLASH_LIGHTS);
          }
          break;
        case OUTTAKING:
        case FLOOR_SHOOT:
        case SUBWOOFER_SHOOT:
        case SPEAKER_SHOOT:
        case AMP_SHOT:
          if (robotState == RobotState.IDLE_NO_GP) {
            lightsOnExit = Optional.of(FLASH_LIGHTS);
          }
          break;
        default:
          break;
      }

      if (previousState != robotState && lightsOnExit.isPresent()) {
        lightsOnExitTimer.start();
      }

      if (lightsOnExitTimer.hasElapsed(DRIVER_SIGNAL_BLINK_DURATION)) {
        lightsOnExit = Optional.empty();
        lightsOnExitTimer.stop();
        lightsOnExitTimer.reset();
      }

      if (lightsOnExit.isPresent()) {
        state = lightsOnExit.get();
      } else if (robotState.lightsState.color() == null) {
        // Use vision color
        state = new LightsState(getVisionLightsColor(), robotState.lightsState.pattern());
      } else {
        state = robotState.lightsState;
      }
    }

    Logger.recordOutput("Lights/Color", state.color().toString());
    Logger.recordOutput("Lights/Pattern", state.pattern());

    Color8Bit color8Bit = new Color8Bit(state.color());

    if (state.color() == Color.kWhite) {
      LimelightHelpers.setLEDMode_ForceBlink("");
    } else {
      LimelightHelpers.setLEDMode_ForceOff("");
    }

    if (state.pattern() == BlinkPattern.SOLID) {
      candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
    } else {
      double time = blinkTimer.get();
      double onDuration = 0;
      double offDuration = 0;

      if (state.pattern() == BlinkPattern.BLINK_FAST) {
        onDuration = FAST_BLINK_DURATION;
        offDuration = FAST_BLINK_DURATION * 2;
      } else if (state.pattern() == BlinkPattern.BLINK_SLOW) {
        onDuration = SLOW_BLINK_DURATION;
        offDuration = SLOW_BLINK_DURATION * 2;
      }

      if (time >= offDuration) {
        blinkTimer.reset();
        candle.setLEDs(0, 0, 0);
      } else if (time >= onDuration) {
        candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
      }
    }

    previousState = robotState;
  }
}
