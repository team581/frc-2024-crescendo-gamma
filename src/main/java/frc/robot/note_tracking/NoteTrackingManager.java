// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class NoteTrackingManager extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private static final String LIMELIGHT_NAME = "note";
  private final InterpolatingDoubleTreeMap tyToDistance = new InterpolatingDoubleTreeMap();

  public NoteTrackingManager(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.NOTE_TRACKING);

    this.localization = localization;
    this.swerve = swerve;
    RobotConfig.get().vision().tyToNoteDistance().accept(tyToDistance);
  }

  private Pose2d getNotePose() {
    var robotPose = getPose();
    double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
    double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);

    Logger.recordOutput("NoteTracking/TY", ty);

    double forwardDistanceToNote = tyToDistance.get(ty);
    Rotation2d angleFromNote = Rotation2d.fromDegrees(tx);

    double sidewaysDistanceToNote =
        forwardDistanceToNote
            * Math.sin(angleFromNote.getRadians())
            / Math.sin(Units.degreesToRadians(90) - angleFromNote.getRadians());

    var notePoseWithoutRotation =
        new Translation2d(
            robotPose.getX() - forwardDistanceToNote, robotPose.getY() - sidewaysDistanceToNote);

    // Uses distance angle math to aim, inverses the angle for intake
    double rotation =
          VisionSubsystem.distanceToTargetPose(
                new Pose2d(notePoseWithoutRotation, new Rotation2d()), robotPose)
            .angle()
            .getRadians();
    return new Pose2d(
        notePoseWithoutRotation,
        new Rotation2d(rotation + getPose().getRotation().getRadians() + Math.PI));
  }

  public Command driveToNotePose() {
    return swerve.driveToPoseCommand(this::getNotePose, this::getPose);
  }

  private Pose2d getPose() {
    return localization.getOdometryPose();
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("NoteTracking/NotePose", getNotePose());
  }
}
