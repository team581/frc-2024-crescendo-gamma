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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  public static final Pose2d RED_SPEAKER_DOUBLE_TAG_CENTER = new Pose2d(16.58, 5.53 + 0.283, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_SPEAKER_DOUBLE_TAG_CENTER = new Pose2d(0.0, 5.53 + 0.283, Rotation2d.fromDegrees(0));

  private static double currentTimestamp = 0;
  private static double previousTimestamp = 0;

  InterpolatingDoubleTreeMap distanceToDev = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap angleToDistance = new InterpolatingDoubleTreeMap();

  private final ImuSubsystem imu;

  private static Pose3d fixEvilPose(Pose3d pose) {
    return new Pose3d(
        new Translation3d(
            pose.getX() + (16.54175 / 2.0), pose.getY() + (8.2042 / 2.0), pose.getZ()),
        pose.getRotation());
  }

  private Optional<FastLimelightResults> storedResults = Optional.empty();

  private Optional<Pose2d> getSimpleSpeakerBasePose() {
    // If not valid, return None
    double validReadings = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    if(validReadings == 0.0){
      return Optional.empty();
    }


    // Get corners from Limelight

    // Make sure we have 2 sets of corners
    double[] corners = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornxy").getDoubleArray(new double[16]);
    if(corners.length<16){
      return Optional.empty();
    }
    if(corners[0]==0.0 & corners[15]==0.0){
      return Optional.empty();
    }
    // COmbine corners into larger rectangle
    double highestX = corners[0];
    double lowestX = corners[0];
    double highestY = corners[1];
    double lowestY = corners[1];
    for (int i=0;i<16;i=i+2){
      if (highestX<corners[i]){
        highestX = corners[i];
      }
      else if (lowestX>corners[i]){
        lowestX = corners[i];
      }
      if (highestY<corners[i+1]){
        highestY = corners[i+1];
      }
      else if (lowestY>corners[i+1]){
        lowestY = corners[i+1];
      }
    }

    // Find center coordinates of rectange
    double [] centerCoordinatesPixels = new double[2] ;
    centerCoordinatesPixels [0]=lowestX+((highestX-lowestX)/2);
    centerCoordinatesPixels [1]=lowestY+((highestY-lowestY)/2);
    // Convert center coordinates into Angle and Distance

    // Pixel X : 0 to 1280
    // Angle X : -31.75 to 31.75


    // Pixel Y : 0 to 960
    // Angle Y : -24.85 to 24.85
    double pixelX = centerCoordinatesPixels[0];
    double pixelY = centerCoordinatesPixels[1];

    double angleX = 0.0;
    double angleY = 0.0;

    angleX = -1* (((pixelX/1280.0)*63.5)-31.5);
    angleY = -1*(((pixelY/960.0)*49.7)-24.85);


    Rotation2d cameraToAngle = Rotation2d.fromDegrees(angleX+this.imu.getRobotHeading().getDegrees());
    double distanceFromSpeaker = getDistanceFromAngle(angleY);

    double distanceX = Math.cos(cameraToAngle.getRadians())*distanceFromSpeaker;
    double distanceY = Math.sin(cameraToAngle.getRadians())*distanceFromSpeaker;

    Pose2d fieldPosition = new Pose2d(RED_SPEAKER_DOUBLE_TAG_CENTER.getX() - distanceX, RED_SPEAKER_DOUBLE_TAG_CENTER.getY() - distanceY, this.imu.getRobotHeading());


    Logger.recordOutput("Vision/Field Position", fieldPosition);
    Logger.recordOutput("Vision/Center Coordinates of both april tags",centerCoordinatesPixels);
    Logger.recordOutput("Vision/Angle X", angleX);
    Logger.recordOutput("Vision/Angle Y", angleY);
    Logger.recordOutput("Vision/Distance X", distanceX);
    Logger.recordOutput("Vision/Distance Y", distanceY);
    Logger.recordOutput("Vision/Camera Angle To Target", cameraToAngle);
    Logger.recordOutput("Vision/distance from speaker", distanceFromSpeaker);

    return Optional.of(fieldPosition) ;


  }

  private Optional<FastLimelightResults> getFastResults() {
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

      Logger.recordOutput("Localization/Valid", valid);

      var output = new FastLimelightResults(totalLatency, robotPoseFieldSpace, minDistance);

      if (isResultValid(output)) {
        return Optional.of(output);
      }

      return Optional.empty();
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  private static DistanceAngle distanceToTargetPose(Pose2d target, Pose2d current) {
    double distance =
        Math.sqrt(
            (Math.pow(target.getY() - current.getY(), 2))
                + (Math.pow(target.getX() - current.getX(), 2)));
    Rotation2d angle =
        new Rotation2d(
                Math.atan((target.getY() - current.getY()) / (target.getX() - current.getX()))
                    - current.getRotation().getRadians())
            .plus(Rotation2d.fromDegrees(180));

    angle = angle.minus(target.getRotation());

    return new DistanceAngle(distance, angle);
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

    angleToDistance.put(16.826, 1.51);
    angleToDistance.put(-5.772, 6.44);
    angleToDistance.put(-1.735, 4.13);
    angleToDistance.put(5.099, 2.55);
  }

  public DistanceAngle getDistanceAngleSpeaker() {
    Pose2d goalPose;
    if (FmsSubsystem.isRedAlliance()) {
      goalPose = RED_SPEAKER;
    } else {
      goalPose = BLUE_SPEAKER;
    }

    Logger.recordOutput("Vision/SpeakerPose", goalPose);

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
  public double getDistanceFromAngle(double angle){
    return angleToDistance.get(angle);
  }

  public void setRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  private boolean isResultValid(FastLimelightResults results) {
    return imu.getRobotAngularVelocity(Timer.getFPGATimestamp() - results.latency()).getDegrees()
            < 20.0
        && results.robotPose().getZ() < Units.feetToMeters(4);
  }

  @Override
  public void robotPeriodic() {
    getSimpleSpeakerBasePose();
    storedResults = getFastResults();
    Logger.recordOutput("Vision/DistanceFromSpeaker", getDistanceAngleSpeaker().distance());
    Logger.recordOutput("Vision/AngleFromSpeaker", getDistanceAngleSpeaker().angle());

    Logger.recordOutput("Vision/DistanceFromFloorSpot", getDistanceAngleFloorShot().distance());
    Logger.recordOutput("Vision/AngleFromFloorSpot", getDistanceAngleFloorShot().angle());
    if (storedResults.isPresent()) {
      var data = storedResults.get();
      Logger.recordOutput("Vision/Latency", data.latency());
      Logger.recordOutput("Vision/DistanceFromTag", data.distanceToTag());
    }
  }

  public Optional<FastLimelightResults> getResults() {
    return storedResults;
  }

  public VisionState getState() {
    if (limelightTimer.hasElapsed(5)) {
      return VisionState.OFFLINE;
    }

    if (storedResults.isPresent()) {
      return VisionState.SEES_TAGS;
    }

    return VisionState.ONLINE_NO_TAGS;
  }
}
