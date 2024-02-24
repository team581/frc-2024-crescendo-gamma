// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoClimbManager extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final RobotCommands actions;
  private final RobotManager robot;
  private final SwerveSubsystem swerve;

  public AutoClimbManager(
      LocalizationSubsystem localization,
      RobotCommands actions,
      RobotManager robot,
      SwerveSubsystem swerve) {
    super(SubsystemPriority.CLIMBER);

    this.localization = localization;
    this.actions = actions;
    this.robot = robot;
    this.swerve = swerve;
  }

  private List<ClimbPath> getPaths() {
    if (FmsSubsystem.isRedAlliance()) {
      return ClimbAlignPaths.RED_PATHS;
    } else {
      return ClimbAlignPaths.BLUE_PATHS;
    }
  }

  private ClimbPath getClosestTarget() {
    Pose2d current = localization.getPose();

    ClimbPath closest = getPaths().get(0);
    double currentDistance = Double.POSITIVE_INFINITY;

    for (ClimbPath target : getPaths()) {
      double distance = target.target().getTranslation().getDistance(current.getTranslation());
      if (distance < currentDistance) {
        closest = target;
        currentDistance = distance;
      }
    }
    return closest;
  }

  private Command getAlignForClimbCommand() {
    return swerve
        .driveToPoseCommand(() -> getClosestTarget().target(), localization::getPose)
        .andThen(
            swerve.driveToPoseCommand(
                () -> getClosestTarget().secondTarget(), localization::getPose));
  }

  public Command getClimbSequenceCommand() {
    // TODO: Uncomment climb stuff once path following is tested
    return getAlignForClimbCommand();
    // .andThen(actions.getClimberCommand())
    // .andThen(robot.waitForStateCommand(RobotState.CLIMBER_HANGING));
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("AutoClimb/ClosestPath", getClosestTarget().target());
    Logger.recordOutput("AutoClimb/ClosestPathRotation", getClosestTarget().target().getRotation());
  }
}
