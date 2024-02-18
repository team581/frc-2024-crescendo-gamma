// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import com.jsoniter.JsonIterator;
import com.jsoniter.any.Any;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.fms.FmsSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends LifecycleSubsystem {
  private static final Timer limelightTimer = new Timer();

  static {
    limelightTimer.start();
  }

  public static final Pose2d RED_SPEAKER = new Pose2d(16.58, 5.53, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_SPEAKER = new Pose2d(0, 5.53, Rotation2d.fromDegrees(0));

  public static final Pose2d RED_FLOOR_SPOT = new Pose2d(15.5, 6.9, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_FLOOR_SPOT = new Pose2d(1, 6.9, Rotation2d.fromDegrees(0));
  private static double currentTimestamp = 0;
  private static double previousTimestamp = 0;

  private static Pose3d fixEvilPose(Pose3d pose) {
    return new Pose3d(
        new Translation3d(
            pose.getX() + (16.54175 / 2.0), pose.getY() + (8.2042 / 2.0), pose.getZ()),
        pose.getRotation());
  }

  private static FastLimelightResults getFastResults() {
    String jsonStr = LimelightHelpers.getJSONDump("");
    Any json = JsonIterator.deserialize(jsonStr);

    Any results = json.get("Results");

    double tl = results.get("tl").toDouble();
    double ts = results.get("ts").toDouble();
    previousTimestamp = currentTimestamp;
    currentTimestamp = ts;

    if (previousTimestamp == currentTimestamp) {
      // Vision data hasn't gotten updated
    } else {
      // Vision data is different
      limelightTimer.reset();
    }

    List<Any> fiducial = results.get("Fiducial").asList();

    // do a for loop, just get thenumber arary things
    // dont even do anything with them. we can figure out how to package them up later
    // just focus on parsing

    ArrayList<PoseWithTagID> tags = new ArrayList<>(fiducial.size());
    for (Any fiducialEntry : fiducial) {
      int fiducialID = fiducialEntry.get("fID").toInt();
      List<Any> robotPoseFieldSpaceArray = fiducialEntry.get("t6r_fs").asList();
      List<Any> cameraPoseTargetSpaceArray = fiducialEntry.get("t6r_ts").asList();

      if (robotPoseFieldSpaceArray.size() < 6 || cameraPoseTargetSpaceArray.size() < 6) {
        continue;
      }

      Pose3d robotPoseFieldSpace =
          new Pose3d(
              new Translation3d(
                  robotPoseFieldSpaceArray.get(0).toDouble(),
                  robotPoseFieldSpaceArray.get(1).toDouble(),
                  robotPoseFieldSpaceArray.get(2).toDouble()),
              new Rotation3d(
                  Units.degreesToRadians(robotPoseFieldSpaceArray.get(3).toDouble()),
                  Units.degreesToRadians(robotPoseFieldSpaceArray.get(4).toDouble()),
                  Units.degreesToRadians(robotPoseFieldSpaceArray.get(5).toDouble())));

      Pose3d cameraPoseTargetSpace =
          new Pose3d(
              new Translation3d(
                  cameraPoseTargetSpaceArray.get(0).toDouble(),
                  cameraPoseTargetSpaceArray.get(1).toDouble(),
                  cameraPoseTargetSpaceArray.get(2).toDouble()),
              new Rotation3d(
                  Units.degreesToRadians(cameraPoseTargetSpaceArray.get(3).toDouble()),
                  Units.degreesToRadians(cameraPoseTargetSpaceArray.get(4).toDouble()),
                  Units.degreesToRadians(cameraPoseTargetSpaceArray.get(5).toDouble())));

      tags.add(
          new PoseWithTagID(fiducialID, fixEvilPose(robotPoseFieldSpace), cameraPoseTargetSpace));
    }

    return new FastLimelightResults(tl, tags);
  }

  private static DistanceAngle distanceToTargetPose(Pose2d target, Pose2d current) {
    return new DistanceAngle(
        Math.sqrt(
            (Math.pow(target.getY() - current.getY(), 2))
                + (Math.pow(target.getX() - current.getX(), 2))),
        new Rotation2d(
            Math.atan((target.getY() - current.getY()) / (target.getX() - current.getX()))
                - current.getRotation().getRadians()));
  }

  private Pose2d robotPose = new Pose2d();
  private boolean hasTags = false;

  public VisionSubsystem() {
    super(SubsystemPriority.VISION);
  }

  public DistanceAngle getDistanceAngleSpeaker() {
    Pose2d goalPose;
    if (FmsSubsystem.isRedAlliance()) {
      goalPose = RED_SPEAKER;
    } else {
      goalPose = BLUE_SPEAKER;
    }

    return distanceToTargetPose(goalPose, robotPose);
  }

  public DistanceAngle getDistanceAngleFloorSpot() {
    Pose2d goalPose;
    if (FmsSubsystem.isRedAlliance()) {
      goalPose = RED_FLOOR_SPOT;
    } else {
      goalPose = BLUE_FLOOR_SPOT;
    }

    Logger.recordOutput("Vision/FloorSpot", goalPose);

    return distanceToTargetPose(goalPose, robotPose);
  }

  public void setRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public FastLimelightResults getFilteredAprilTags() {
    FastLimelightResults results = getFastResults();
    List<PoseWithTagID> allTargets = results.tags();
    List<PoseWithTagID> goodTargets = new ArrayList<>();

    for (int i = 0; i < allTargets.size(); i++) {
      PoseWithTagID tag = allTargets.get(i);

      Pose3d cameraPoseFromTarget = tag.cameraPoseTargetSpace();
      double distanceToCamera =
          cameraPoseFromTarget.getTranslation().getDistance(new Translation3d());
      Logger.recordOutput("Vision/Tag" + tag.fiducialID() + "/DistanceToCamera", distanceToCamera);
      goodTargets.add(tag);
      /*if (distanceToCamera < 4.5) {
        if (FmsSubsystem.isRedAlliance()) {
          if (tag.fiducialID() == 3 || tag.fiducialID() == 4) {
            goodTargets.add(tag);
          }
        } else {
          if (tag.fiducialID() == 7 || tag.fiducialID() == 8) {
            goodTargets.add(tag);
          }
        }
      }*/
    }

    if (goodTargets.size() > 0) {
      hasTags = true;
    } else {
      hasTags = false;
    }

    double latencyTotal = results.latency();

    // TODO: Fix latency
    return new FastLimelightResults(Timer.getFPGATimestamp(), goodTargets);
  }

  @Override
  public void robotPeriodic() {
    try {
      for (PoseWithTagID tag : getFilteredAprilTags().tags()) {
        Logger.recordOutput("Vision/Tag" + tag.fiducialID() + "/RobotPose", tag.robotPose());
      }
    } catch (Exception e) {
    }

    Logger.recordOutput("Vision/DistanceFromSpeaker", getDistanceAngleSpeaker().distance());
    Logger.recordOutput("Vision/AngleFromSpeaker", getDistanceAngleSpeaker().angle());

    Logger.recordOutput("Vision/DistanceFromFloorSpot", getDistanceAngleFloorSpot().distance());
    Logger.recordOutput("Vision/AngleFromFloorSpot", getDistanceAngleFloorSpot().angle());
  }

  public boolean isWorking() {
    return !limelightTimer.hasElapsed(5);
  }
}
