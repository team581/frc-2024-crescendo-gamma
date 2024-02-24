// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class ClimbAlignPaths {
  public static final ClimbPath RED_LEFT =
      new ClimbPath(
          // PathPlannerPath.bezierFromPoses(
          new Pose2d(12.69, 2.37, Rotation2d.fromDegrees(-60)),
          new Pose2d(12.0, 3.5, Rotation2d.fromDegrees(-60)),
          Rotation2d.fromDegrees(-60));
  public static final ClimbPath RED_FORWARD =
      new ClimbPath(
          // PathPlannerPath.bezierFromPoses(
          new Pose2d(9.48, 3.94, Rotation2d.fromDegrees(180)),
          new Pose2d(10.80, 3.94, Rotation2d.fromDegrees(180)),
          Rotation2d.fromDegrees(180));
  public static final ClimbPath RED_RIGHT =
      new ClimbPath(
          //  PathPlannerPath.bezierFromPoses(
          new Pose2d(12.6, 5.74, Rotation2d.fromDegrees(60)),
          new Pose2d(12.14, 4.95, Rotation2d.fromDegrees(60)),
          Rotation2d.fromDegrees(60));
  public static final ClimbPath BLUE_LEFT =
      new ClimbPath(
          // PathPlannerPath.bezierFromPoses(
          new Pose2d(5.64, 5.5, Rotation2d.fromDegrees(0)),
          new Pose2d(5.00, 4.75, Rotation2d.fromDegrees(0)),
          Rotation2d.fromDegrees(120));
  public static final ClimbPath BLUE_FORWARD =
      new ClimbPath(
          // PathPlannerPath.bezierFromPoses(
          new Pose2d(6.32, 4.11, Rotation2d.fromDegrees(0)),
          new Pose2d(5.55, 4.11, Rotation2d.fromDegrees(0)),
          Rotation2d.fromDegrees(240));
  public static final ClimbPath BLUE_RIGHT =
      new ClimbPath(
          // PathPlannerPath.bezierFromPoses(
          new Pose2d(3.85, 2.49, Rotation2d.fromDegrees(-120)),
          new Pose2d(4.36, 3.26, Rotation2d.fromDegrees(-120)),
          Rotation2d.fromDegrees(-120));

  public static final List<ClimbPath> RED_PATHS = List.of(RED_LEFT, RED_FORWARD, RED_RIGHT);

  public static final List<ClimbPath> BLUE_PATHS = List.of(BLUE_LEFT, BLUE_FORWARD, BLUE_RIGHT);

  private ClimbAlignPaths() {}
}
