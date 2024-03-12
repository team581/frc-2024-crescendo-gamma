// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.logging.advantagekit.Logger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class LifecycleSubsystemManager {
  private static LifecycleSubsystemManager instance;

  public static LifecycleSubsystemManager getInstance() {
    if (instance == null) {
      instance = new LifecycleSubsystemManager();
    }

    return instance;
  }

  public static LifecycleStage getStage() {
    if (DriverStation.isTeleopEnabled()) {
      return LifecycleStage.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      return LifecycleStage.AUTONOMOUS;
    } else if (DriverStation.isTestEnabled()) {
      return LifecycleStage.TEST;
    } else {

      return LifecycleStage.DISABLED;
    }
  }

  private final List<LifecycleSubsystem> subsystems = new ArrayList<>();
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  private LifecycleSubsystemManager() {}

  public void ready() {
    Collections.sort(
        subsystems,
        (Comparator.comparingInt((LifecycleSubsystem subsystem) -> subsystem.priority.value)
            .reversed()));

    for (LifecycleSubsystem lifecycleSubsystem : subsystems) {
      commandScheduler.registerSubsystem(lifecycleSubsystem);
    }
  }

  public void log() {
    Logger.recordOutput("Scheduler/Stage", getStage().toString());
  }

  void registerSubsystem(LifecycleSubsystem subsystem) {
    subsystems.add(subsystem);
    commandScheduler.unregisterSubsystem(subsystem);
  }
}
