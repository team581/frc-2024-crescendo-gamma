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
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class LightsSubsystem extends LifecycleSubsystem {

  private static final double FAST_BLINK_DURATION = 0.08;
  private static final double SLOW_BLINK_DURATION = 0.25;

  private final CANdle candle;
  private final RobotManager robotManager;
  private final VisionSubsystem vision;
  private final Timer blinkTimer = new Timer();

  private Color color = Color.kWhite;
  private BlinkPattern blinkPattern = BlinkPattern.SOLID;

  public LightsSubsystem(CANdle candle, RobotManager robotManager, VisionSubsystem vision) {
    super(SubsystemPriority.LIGHTS);

    this.candle = candle;
    this.robotManager = robotManager;
    this.vision = vision;

    blinkTimer.start();

    CANdleConfiguration config = new CANdleConfiguration();
    config.disableWhenLOS = true;

    candle.configAllSettings(config);
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Lights/Color", color.toString());
    Logger.recordOutput("Lights/BlinkPattern", blinkPattern);

    RobotState state = robotManager.getState();

    if (DriverStation.isDisabled()) {
      if (robotManager.wrist.getHomingState() == HomingState.PRE_MATCH_HOMING) {
        color = Color.kRed;
        blinkPattern = BlinkPattern.SOLID;
      } else if (!vision.isWorking()) {
        color = Color.kYellow;
        blinkPattern = BlinkPattern.BLINK_SLOW;
      } else {
        color = Color.kGreen;
        blinkPattern = BlinkPattern.SOLID;
      }
    } else {
      switch (state) {
        case IDLE_NO_GP:
        case GROUND_INTAKING:
          color = Color.kOrange;
          blinkPattern = BlinkPattern.BLINK_SLOW;
          break;
        case IDLE_WITH_GP:
        case WAITING_SPEAKER_SHOT:
        case WAITING_SUBWOOFER_SHOT:
        case WAITING_AMP_SHOT:
        case WAITING_FLOOR_SHOT:
          color = Color.kOrange;
          blinkPattern = BlinkPattern.SOLID;
          break;
        case PREPARE_SPEAKER_SHOT:
        case PREPARE_SUBWOOFER_SHOT:
        case PREPARE_FLOOR_SHOT:
        case PREPARE_AMP_SHOT:
          if (vision.isWorking()) {
            color = Color.kGreen;
            blinkPattern = BlinkPattern.BLINK_SLOW;
          } else {
            color = Color.kRed;
            blinkPattern = BlinkPattern.BLINK_SLOW;
          }
          break;
        case AMP_SHOT:
        case OUTTAKING:
        case OUTTAKING_SHOOTER:
        case TRAP_OUTTAKE:
        case SUBWOOFER_SHOOT:
          color = Color.kGreen;
          blinkPattern = BlinkPattern.BLINK_FAST;
          break;
        case SPEAKER_SHOOT:
        case FLOOR_SHOOT:
          if (vision.isWorking()) {
            color = Color.kGreen;
            blinkPattern = BlinkPattern.BLINK_FAST;
          } else {
            color = Color.kRed;
            blinkPattern = BlinkPattern.BLINK_FAST;
          }
          break;
        case UNHOMED:
        case HOMING:
          color = Color.kRed;
          blinkPattern = BlinkPattern.BLINK_SLOW;
          break;
        case WAITING_CLIMBER_RAISED:
          color = Color.kCyan;
          blinkPattern = BlinkPattern.SOLID;
          break;
        case PREPARE_CLIMBER_HANGING:
        case PREPARE_CLIMBER_RAISED:
          color = Color.kCyan;
          blinkPattern = BlinkPattern.BLINK_SLOW;
          break;
        case CLIMBER_HANGING:
        case CLIMBER_RAISED:
          color = Color.kCyan;
          blinkPattern = BlinkPattern.BLINK_FAST;
          break;
        case PREPARE_TRAP_OUTTAKE:
          color = Color.kGreen;
          blinkPattern = BlinkPattern.BLINK_SLOW;
          break;
        default:
          color = Color.kWhite;
          blinkPattern = BlinkPattern.SOLID;
          break;
      }
    }

    Color8Bit color8Bit = new Color8Bit(color);

    if (blinkPattern == BlinkPattern.SOLID) {
      candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
    } else {
      double time = blinkTimer.get();
      double onDuration = 0;
      double offDuration = 0;

      if (blinkPattern == BlinkPattern.BLINK_FAST) {
        onDuration = FAST_BLINK_DURATION;
        offDuration = FAST_BLINK_DURATION * 2;
      } else if (blinkPattern == BlinkPattern.BLINK_SLOW) {
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
  }
}
