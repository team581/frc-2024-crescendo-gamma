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
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class NoteTrackingManager extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final RobotCommands actions;
  private final RobotManager robot;
  private static final String LIMELIGHT_NAME = "limelight-note";
  private final InterpolatingDoubleTreeMap tyToDistance = new InterpolatingDoubleTreeMap();
  private Pose2d lastNotePose = new Pose2d();

  private double midlineXValue = 8.3;

  public NoteTrackingManager(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      RobotCommands actions,
      RobotManager robot) {
    super(SubsystemPriority.NOTE_TRACKING);

    this.localization = localization;
    this.swerve = swerve;
    this.actions = actions;
    this.robot = robot;
    RobotConfig.get().vision().tyToNoteDistance().accept(tyToDistance);
  }

  private Optional<Pose2d> getNotePose() {
    // TODO: update limelight so v works
    // long v =
    // NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("v").getInteger(0);
    double ty =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("ty").getDouble(0);
    double tx =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tx").getDouble(0);

    if (tx == 0) {
      return Optional.empty();
    }

    if (ty < -15.0) {
      return Optional.empty();
    }

    // TODO: This should probably be the robot pose at the time the image was taken, since there is
    // latency
    var robotPose = getPose();

    Logger.recordOutput("NoteTracking/TY", ty);
    Logger.recordOutput("NoteTracking/TX", tx);

    double forwardDistanceToNote = tyToDistance.get(ty);
    Rotation2d angleFromNote = Rotation2d.fromDegrees(tx);
    Logger.recordOutput("NoteTracking/ForwardDistance", forwardDistanceToNote);

    var c = forwardDistanceToNote / Math.cos(angleFromNote.getRadians());
    double sidewaysDistanceToNote = Math.sqrt(Math.pow(c, 2) - Math.pow(forwardDistanceToNote, 2));

    // Flips side of robot note is on based on if tx is positive or negative
    if (tx > 0) {
      sidewaysDistanceToNote *= -1.0;
    }

    Logger.recordOutput("NoteTracking/SidewaysDistance", sidewaysDistanceToNote);
    var notePoseWithoutRotation =
        new Translation2d(-forwardDistanceToNote, -sidewaysDistanceToNote)
            .rotateBy(Rotation2d.fromDegrees(getPose().getRotation().getDegrees()));

    var notePoseWithRobot =
        new Translation2d(
            getPose().getX() + notePoseWithoutRotation.getX(),
            getPose().getY() + notePoseWithoutRotation.getY());
    // Uses distance angle math to aim, inverses the angle for intake
    double rotation =
        VisionSubsystem.distanceToTargetPose(
                new Pose2d(notePoseWithRobot, new Rotation2d()), robotPose)
            .targetAngle()
            .getRadians();
    return Optional.of(new Pose2d(notePoseWithRobot, new Rotation2d(rotation + Math.PI)));
  }

  private boolean pastMidline() {
    Pose2d robotPose = getPose();
    double pastMidlineThresholdMeters = 0.65;

    // Red alliance
    if (FmsSubsystem.isRedAlliance()) {
      if (robotPose.getX() < (midlineXValue - pastMidlineThresholdMeters)) {
        return true;
      }
      return false;
    }

    // Blue alliance
    return robotPose.getX() > (midlineXValue + pastMidlineThresholdMeters);
  }

  public Command intakeDetectedNote() {
    return swerve
        .driveToPoseCommand(() -> lastNotePose, this::getPose)
        .until(() -> pastMidline())
        .raceWith(actions.intakeCommand().withTimeout(2))
        .finallyDo(robot::stowRequest);
  }

  private Pose2d getPose() {
    return localization.getPose();
  }

  @Override
  public void robotPeriodic() {
    Optional<Pose2d> notePose = getNotePose();
    if (notePose.isPresent()) {
      Logger.recordOutput("NoteTracking/NotePose", notePose.get());
      lastNotePose = notePose.get();
    }
  }
}
