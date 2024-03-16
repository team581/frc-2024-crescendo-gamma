// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class RumbleControllerSubsystem extends LifecycleSubsystem {
  private final Timer matchTimer = new Timer();
  private final GenericHID controller;
  public static final double MATCH_DURATION_TELEOP = 135;

  @Override
  public void teleopInit() {
    matchTimer.reset();
    matchTimer.start();
  }

  @Override
  public void disabledInit() {
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  public RumbleControllerSubsystem(CommandXboxController controller, boolean matchTimeRumble) {
    this(controller.getHID(), matchTimeRumble);
  }

  public RumbleControllerSubsystem(GenericHID controller, boolean matchTimeRumble) {
    super(SubsystemPriority.RUMBLE_CONTROLLER);
    this.controller = controller;

    if (matchTimeRumble) {
      // Rumble based on match time
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 90))
          .onTrue(getRumbleLongCommand());
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 60))
          .onTrue(getRumbleLongCommand());
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 30))
          .onTrue(getRumbleLongCommand());
    }
  }

  private Command getVibrateOnceCommand() {
    return Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1))
        .andThen(Commands.waitSeconds(0.2))
        .andThen(() -> controller.setRumble(RumbleType.kBothRumble, 0))
        .andThen(Commands.waitSeconds(0.1))
        .finallyDo(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 0);
            });
  }

  public Command getRumbleShortCommand() {
    return getVibrateOnceCommand();
  }

  public Command getRumbleLongCommand() {
    return getVibrateOnceCommand()
        .andThen(getVibrateOnceCommand())
        .andThen(getVibrateOnceCommand());
  }
}
