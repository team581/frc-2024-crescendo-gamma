// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class NoteTracking extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;

  InterpolatingDoubleTreeMap distanceToNote = new InterpolatingDoubleTreeMap();

  public NoteTracking(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.VISION);

    this.localization = localization;
    this.swerve = swerve;

    distanceToNote.put(0.0, 0.0);
  }

  private Pose2d robotPose = new Pose2d();

  public double distanceFromNoteToRobot() {
    return distanceToNote.get(
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
  }

  public double angleFromRobotToNote() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double forwardDistanceToNote() {
    return distanceFromNoteToRobot() * Math.sin(angleFromRobotToNote());
  }

  public double sidewaysDistanceToNote() {
    return distanceFromNoteToRobot() * Math.cos(angleFromRobotToNote());
  }

  public Transform2d noteTransform() {
    return new Transform2d(
        forwardDistanceToNote(), sidewaysDistanceToNote(), Rotation2d.fromDegrees(180));
  }

  public Pose2d notePose(Pose2d robotPose) {
    return robotPose.plus(noteTransform());
  }

  public Command driveToNotePose() {
    return swerve.driveToPoseCommand(() -> notePose(robotPose), localization::getPose);
  }
}
