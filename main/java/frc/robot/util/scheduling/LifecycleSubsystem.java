// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Stopwatch;

/**
 * Extends {@link SubsystemBase} by adding in lifecycle methods for robotInit, teleopPeriodic, etc.,
 * similar to {@link Robot}.
 */
public class LifecycleSubsystem extends SubsystemBase {
  final SubsystemPriority priority;

  private final Stopwatch stopwatch = Stopwatch.getInstance();
  private final String loggerName;

  private LifecycleStage previousStage = null;

  public LifecycleSubsystem(SubsystemPriority priority) {
    this.priority = priority;

    LifecycleSubsystemManager.getInstance().registerSubsystem(this);

    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    // First string concat causes lag spike. This shifts lag spike to code init.
    loggerName = "Scheduler/LifecycleSubsystem/" + name + ".periodic()";

    robotInit();
  }

  /** {@link IterativeRobotBase#robotInit()} */
  public void robotInit() {}

  /** {@link IterativeRobotBase#robotPeriodic()} */
  public void robotPeriodic() {}

  public void enabledInit() {}

  public void enabledPeriodic() {}

  /** {@link IterativeRobotBase#autonomousInit()} */
  public void autonomousInit() {}

  /** {@link IterativeRobotBase#autonomousPeriodic()} */
  public void autonomousPeriodic() {}

  /** {@link IterativeRobotBase#teleopInit()} */
  public void teleopInit() {}

  /** {@link IterativeRobotBase#teleopPeriodic()} */
  public void teleopPeriodic() {}

  /** {@link IterativeRobotBase#disabledInit()} */
  public void disabledInit() {}

  /** {@link IterativeRobotBase#disabledPeriodic()} */
  public void disabledPeriodic() {}

  @Override
  public void periodic() {
    stopwatch.start(loggerName);
    LifecycleStage stage;

    stage = LifecycleSubsystemManager.getStage();

    boolean isInit = previousStage != stage;

    robotPeriodic();

    if (stage == LifecycleStage.DISABLED) {
      if (isInit) {
        disabledInit();
      }

      disabledPeriodic();
    } else {
      if (isInit) {
        enabledInit();
      }

      enabledPeriodic();

      if (stage == LifecycleStage.TELEOP) {
        if (isInit) {
          teleopInit();
        }

        teleopPeriodic();
      } else if (stage == LifecycleStage.AUTONOMOUS) {
        if (isInit) {
          autonomousInit();
        }

        autonomousPeriodic();
      } else if (stage == LifecycleStage.TEST) {
        if (isInit) {
          testInit();
        }

        testPeriodic();
      }
    }

    stopwatch.stop(loggerName);

    previousStage = stage;
  }

  /** {@link IterativeRobotBase#testInit()} */
  public void testInit() {}

  /** {@link IterativeRobotBase#testPeriodic()} */
  public void testPeriodic() {}
}
