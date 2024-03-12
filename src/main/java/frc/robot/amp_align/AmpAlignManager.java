// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.amp_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.logging.advantagekit.Logger;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;

public class AmpAlignManager extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;

  public AmpAlignManager(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.CLIMBER);

    this.localization = localization;
    this.swerve = swerve;
  }

  private AmpAlignPath getClosestTarget() {
    Pose2d current = localization.getPose();

    List<AmpAlignPath> paths = AmpAlignPaths.PATHS;

    AmpAlignPath closest = AmpAlignPaths.PATHS.get(0);
    double currentDistance = Double.POSITIVE_INFINITY;

    for (AmpAlignPath target : paths) {
      double distance =
          target.behindAmpPose().getTranslation().getDistance(current.getTranslation());
      if (distance < currentDistance) {
        closest = target;
        currentDistance = distance;
      }
    }
    return closest;
  }

  public Command getAlignForAmpCommand() {
    return swerve.driveToPoseCommand(
        () -> getClosestTarget().behindAmpPose(), localization::getPose);
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("AmpAlign/ClosestPose", getClosestTarget().behindAmpPose());
    Logger.recordOutput(
        "AmpAlign/ClosestPathRotation", getClosestTarget().behindAmpPose().getRotation());
  }
}
