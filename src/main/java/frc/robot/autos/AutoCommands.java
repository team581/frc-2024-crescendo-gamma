// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.RobotCommands;

public class AutoCommands {
  private final RobotCommands actions;

  public AutoCommands(RobotCommands actions) {
    this.actions = actions;
  }

  public Command subwooferShotWithTimeout() {
    return actions
        .subwooferShotCommand()
        .withTimeout(2)
        .andThen(actions.outtakeShooterCommand().withTimeout(1))
        .withName("SubwooferShotWithTimeout");
  }

  public Command speakerShotWithTimeout() {
    return actions
        .speakerShotCommand()
        .withTimeout(2)
        .andThen(actions.outtakeShooterCommand().withTimeout(1))
        .withName("SpeakerShotWithTimeout");
  }
}
