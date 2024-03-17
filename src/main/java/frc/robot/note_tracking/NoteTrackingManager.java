// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.littletonrobotics.junction.Logger;

public class NoteTrackingManager extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private static final String LIMELIGHT_NAME = "limelight-note";
  private final InterpolatingDoubleTreeMap tyToDistance = new InterpolatingDoubleTreeMap();

  public NoteTrackingManager(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.NOTE_TRACKING);

    this.localization = localization;
    this.swerve = swerve;
    RobotConfig.get().vision().tyToNoteDistance().accept(tyToDistance);
  }

  private Optional<Pose2d> getNotePose() {
    //TODO: update limelight so v works
    //long v = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("v").getInteger(0);
    double ty =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("ty").getDouble(0);
    double tx =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tx").getDouble(0);
    
        if (tx
        == 0) {
      return Optional.empty();
    } else {
    var robotPose = getPose();

    Logger.recordOutput("NoteTracking/TY", ty);
    Logger.recordOutput("NoteTracking/TX", tx);

    double forwardDistanceToNote = tyToDistance.get(ty);
    Rotation2d angleFromNote = Rotation2d.fromDegrees(tx);
    Logger.recordOutput("NoteTracking/ForwardDistance", forwardDistanceToNote);

    var c = forwardDistanceToNote / Math.cos(angleFromNote.getRadians());
    double sidewaysDistanceToNote = 0;
    if (tx > 0) {
      sidewaysDistanceToNote = -(Math.sqrt(Math.pow(c, 2) - Math.pow(forwardDistanceToNote, 2)));
    } else {
      sidewaysDistanceToNote = Math.sqrt(Math.pow(c, 2) - Math.pow(forwardDistanceToNote, 2));
    }

    Logger.recordOutput("NoteTracking/SidewaysDistance", sidewaysDistanceToNote);
    var notePoseWithoutRotation =
        new Translation2d(
            robotPose.getX() - forwardDistanceToNote, robotPose.getY() - sidewaysDistanceToNote);

    // Uses distance angle math to aim, inverses the angle for intake
    double rotation =
        VisionSubsystem.distanceToTargetPose(
                new Pose2d(notePoseWithoutRotation, new Rotation2d()), robotPose)
            .angle()
            .getRadians();
    return Optional.of(
        new Pose2d(
            notePoseWithoutRotation,
            new Rotation2d(rotation + getPose().getRotation().getRadians() + Math.PI)));
  }
}

  public Command driveToNotePose() {
    return swerve.driveToPoseCommand(() -> getNotePose().get(), this::getPose);
  }

  private Pose2d getPose() {
    return localization.getPose();
  }

  @Override
  public void robotPeriodic() {
    Optional<Pose2d> notePose = getNotePose();
    if (notePose.isPresent()){
      Logger.recordOutput("NoteTracking/NotePose", notePose.get());
    }

  }
}
