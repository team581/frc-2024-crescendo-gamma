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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
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

  InterpolatingDoubleTreeMap distanceToDev = new InterpolatingDoubleTreeMap();

  private final ImuSubsystem imu;

  private static Pose3d fixEvilPose(Pose3d pose) {
    return new Pose3d(
        new Translation3d(
            pose.getX() + (16.54175 / 2.0), pose.getY() + (8.2042 / 2.0), pose.getZ()),
        pose.getRotation());
  }

  private Optional<FastLimelightResults> storedResults = Optional.empty();

  private static Optional<FastLimelightResults> getFastResults() {
    try {
      double start = Timer.getFPGATimestamp();
      String jsonStr = LimelightHelpers.getJSONDump("");
      Any json = JsonIterator.deserialize(jsonStr);

      Any results = json.get("Results");

      double ts = results.get("ts").toDouble();

      previousTimestamp = currentTimestamp;
      currentTimestamp = ts;

      if (previousTimestamp == currentTimestamp) {
        // Vision data hasn't changed
      } else {
        // Vision data is different
        limelightTimer.reset();
      }

      double v = results.get("v").toDouble();

      boolean valid = v == 1;

      if (!valid) {
        return Optional.empty();
      }

      List<Any> megaTagPose = results.get("botpose").asList();

      Pose3d robotPoseEvilSpace =
          new Pose3d(
              new Translation3d(
                  megaTagPose.get(0).toDouble(),
                  megaTagPose.get(1).toDouble(),
                  megaTagPose.get(2).toDouble()),
              new Rotation3d(
                  Units.degreesToRadians(megaTagPose.get(3).toDouble()),
                  Units.degreesToRadians(megaTagPose.get(4).toDouble()),
                  Units.degreesToRadians(megaTagPose.get(5).toDouble())));

      List<Any> fiducial = results.get("Fiducial").asList();
      ArrayList<Double> distances = new ArrayList<>(fiducial.size());
      for (Any fiducialEntry : fiducial) {
        List<Any> cameraPoseTargetSpaceArray = fiducialEntry.get("t6c_ts").asList();

        if (cameraPoseTargetSpaceArray.size() != 6) {
          continue;
        }

        Pose3d cameraPoseTargetSpace =
            new Pose3d(
                cameraPoseTargetSpaceArray.get(0).toDouble(),
                cameraPoseTargetSpaceArray.get(1).toDouble(),
                cameraPoseTargetSpaceArray.get(2).toDouble(),
                new Rotation3d(
                    Units.degreesToRadians(cameraPoseTargetSpaceArray.get(3).toDouble()),
                    Units.degreesToRadians(cameraPoseTargetSpaceArray.get(4).toDouble()),
                    Units.degreesToRadians(cameraPoseTargetSpaceArray.get(5).toDouble())));
        double distance =
            cameraPoseTargetSpace.getTranslation().getDistance(new Translation3d(0, 0, 0));
        distances.add(distance);
      }

      double[] distanceArray = new double[distances.size()];
      for (int i = 0; i < distances.size(); i++) {
        distanceArray[i] = distances.get(i);
      }

      Logger.recordOutput("Vision/Distances", distanceArray);

      double minDistance = Collections.min(distances);

      double captureLatency = results.get("cl").toDouble() / 1000.0;
      double pipelineLatency = results.get("tl").toDouble() / 1000.0;
      double end = Timer.getFPGATimestamp();
      double jsonParseLatency = (end - start);

      double totalLatency = jsonParseLatency + captureLatency + pipelineLatency;

      Pose3d robotPoseFieldSpace = fixEvilPose(robotPoseEvilSpace);

      Logger.recordOutput("Vision/FilteredRobotPose", robotPoseFieldSpace);
      Logger.recordOutput("Localization/Valid", valid);

      return Optional.of(new FastLimelightResults(totalLatency, robotPoseFieldSpace, minDistance));
    } catch (Exception e) {
      return Optional.empty();
    }
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

  public VisionSubsystem(ImuSubsystem imu) {
    super(SubsystemPriority.VISION);
    this.imu = imu;

    distanceToDev.put(1.0, 0.4);
    distanceToDev.put(5.0, 6.0);
    distanceToDev.put(3.8, 2.6);
    distanceToDev.put(4.5, 3.0);
    distanceToDev.put(3.37, 2.45);
    distanceToDev.put(7.0, 10.0);
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

  public DistanceAngle getDistanceAngleFloorShot() {
    Pose2d goalPose;
    if (FmsSubsystem.isRedAlliance()) {
      goalPose = RED_FLOOR_SPOT;
    } else {
      goalPose = BLUE_FLOOR_SPOT;
    }

    Logger.recordOutput("Vision/FloorSpot", goalPose);

    return distanceToTargetPose(goalPose, robotPose);
  }

  public double getStandardDeviation(double distance) {

    return distanceToDev.get(distance);
  }

  public void setRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public boolean isResultValid(FastLimelightResults results) {
    return (getState() == VisionState.ONLINE
        && imu.getRobotAngularVelocity(Timer.getFPGATimestamp() - results.latency()).getDegrees()
            < 3.0);
  }

  @Override
  public void robotPeriodic() {
    storedResults = getFastResults();
    Logger.recordOutput("Vision/DistanceFromSpeaker", getDistanceAngleSpeaker().distance());
    Logger.recordOutput("Vision/AngleFromSpeaker", getDistanceAngleSpeaker().angle());

    Logger.recordOutput("Vision/DistanceFromFloorSpot", getDistanceAngleFloorShot().distance());
    Logger.recordOutput("Vision/AngleFromFloorSpot", getDistanceAngleFloorShot().angle());
    if (storedResults.isPresent()) {
      var data = storedResults.get();
      Logger.recordOutput("Vision/Latency", data.latency());
      Logger.recordOutput("Vision/TimestampWithLatency", Timer.getFPGATimestamp() - data.latency());
      Logger.recordOutput(
          "Vision/AngularVelocityWithLatency",
          imu.getRobotAngularVelocity(Timer.getFPGATimestamp() - data.latency()));
      Logger.recordOutput("Vision/DistanceFromTag", data.distanceToTag());
    }
  }

  public Optional<FastLimelightResults> getResults() {
    return storedResults;
  }

  public VisionState getState() {
    if (!limelightTimer.hasElapsed(5)) {
      return VisionState.ONLINE;
    }

    if (storedResults.isPresent()) {
      return VisionState.SEES_TAGS;
    }

    return VisionState.OFFLINE;
  }
}
