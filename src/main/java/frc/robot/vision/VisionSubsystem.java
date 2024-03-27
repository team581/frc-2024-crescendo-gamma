// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends LifecycleSubsystem {
  private static final boolean SHOOT_TO_SIDE_ENABLED = true;

  private static final double FOV_HORIZONTAL = RobotConfig.get().vision().fovHorz();
  private static final double FOV_VERTICAL = RobotConfig.get().vision().fovVert();

  private static final double principlePixelOffsetX =
      RobotConfig.get().vision().principlePixelOffsetX();
  private static final double principlePixelOffsetY =
      RobotConfig.get().vision().principlePixelOffsetY();

  public static final Pose2d ORIGINAL_RED_SPEAKER =
      new Pose2d(
          Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
  public static final Pose2d ORIGINAL_BLUE_SPEAKER =
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));

  public static final Pose2d RED_FLOOR_SPOT = new Pose2d(15.5, 6.9, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_FLOOR_SPOT = new Pose2d(1, 6.9, Rotation2d.fromDegrees(0));

  public static final Pose3d CAMERA_ON_BOT =
      new Pose3d(
          0,
          Units.inchesToMeters(-1.103),
          Units.inchesToMeters(24.418),
          RobotConfig.get().vision().llAngle());

  public static final Pose3d RED_SPEAKER_DOUBLE_TAG_CENTER =
      new Pose3d(
          Units.inchesToMeters(652.73),
          Units.inchesToMeters(218.42 - 11.125),
          Units.inchesToMeters(57.25),
          new Rotation3d(0, 0, Units.degreesToRadians(180)));
  public static final Pose3d BLUE_SPEAKER_DOUBLE_TAG_CENTER =
      new Pose3d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(218.42 - 11.125),
          Units.inchesToMeters(57.25),
          new Rotation3d(0, 0, Units.degreesToRadians(180)));

  private final Timer limelightTimer = new Timer();
  private double limelightHeartbeat = -1;

  InterpolatingDoubleTreeMap distanceToDev = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap angleToDistance = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap angleToPositionOffset = new InterpolatingDoubleTreeMap();

  private final ImuSubsystem imu;

  public Optional<VisionResult> getVisionResult() {
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    if (Math.abs(estimatePose.pose.getRotation().getDegrees() - imu.getRobotHeading().getDegrees())
        > 5) {
      return Optional.empty();
    }

    for (int i = 0; i < estimatePose.rawFiducials.length; i++) {
      var fiducial = estimatePose.rawFiducials[i];

      if (fiducial.ambiguity > 0.5) {
        return Optional.empty();
      }
    }
    Logger.recordOutput("Vision/MegatagPose",estimatePose.pose);
    return Optional.of(new VisionResult(estimatePose.pose, estimatePose.latency));
  }

  private Optional<DistanceAngle> getTxTyDistanceAngle() {
    // use tx and ty to calculate a DistanceAngle
    // return optional.empty if tv is not present
    if (LimelightHelpers.getTV(getName())){
      double tx = LimelightHelpers.getTX("");
      double ty = LimelightHelpers.getTY("");



    }
    return Optional.empty();
  }

  public static DistanceAngle distanceToTargetPose(Pose2d target, Pose2d current, boolean usingTagDirectly) {
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

    return new DistanceAngle(distance, angle, usingTagDirectly);
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

    angleToDistance.put(-6.264, Units.inchesToMeters(277.5) - 0.12);
    angleToDistance.put(6.316, Units.inchesToMeters(92) - 0.12);
    angleToDistance.put(1.631, Units.inchesToMeters(127.25) - 0.12);
    angleToDistance.put(17.24, Units.inchesToMeters(58.5) - 0.12);
    angleToDistance.put(-2.589, Units.inchesToMeters(175) - 0.12);

    // 96.5
    angleToPositionOffset.put(Rotation2d.fromDegrees(100).getRadians(), 0.4825);
    angleToPositionOffset.put(Rotation2d.fromDegrees(75).getRadians(), 0.4825);
    angleToPositionOffset.put(Rotation2d.fromDegrees(60).getRadians(), 0.0);
    angleToPositionOffset.put(Rotation2d.fromDegrees(40).getRadians(), 0.0);
    angleToPositionOffset.put(Rotation2d.fromDegrees(0.0).getRadians(), 0.0);
    angleToPositionOffset.put(Rotation2d.fromDegrees(-40).getRadians(), 0.0);
    angleToPositionOffset.put(Rotation2d.fromDegrees(-60).getRadians(), 0.0);
    angleToPositionOffset.put(Rotation2d.fromDegrees(-75).getRadians(), -0.4825);
    angleToPositionOffset.put(Rotation2d.fromDegrees(-100).getRadians(), -0.4825);

    limelightTimer.start();
  }

  private Pose2d getSpeaker() {

    if (FmsSubsystem.isRedAlliance()) {
      return ORIGINAL_RED_SPEAKER;
    } else {
      return ORIGINAL_BLUE_SPEAKER;
    }
  }

  public DistanceAngle getDistanceAngleSpeaker() {
    var txTyDistanceAngle = getTxTyDistanceAngle();

    if (txTyDistanceAngle.isPresent()) {
      return txTyDistanceAngle.get();
    }

    // Fallback to using the pose estimator if we can't use tx & ty

    Pose2d goalPose = getSpeaker();

    var adjustedPose = goalPose;

    if (SHOOT_TO_SIDE_ENABLED) {
      var yOffset =
          angleToPositionOffset.get(
              Math.atan(
                  (goalPose.getY() - robotPose.getY()) / (goalPose.getX() - robotPose.getX())));

      if (!FmsSubsystem.isRedAlliance()) {
        yOffset *= -1.0;
      }

      adjustedPose = new Pose2d(goalPose.getX(), goalPose.getY() + yOffset, goalPose.getRotation());
    }
    Logger.recordOutput("Vision/OriginalSpeakerPose", goalPose);
    Logger.recordOutput("Vision/SpeakerPose", adjustedPose);

    return distanceToTargetPose(adjustedPose, robotPose, false);
  }

  public DistanceAngle getDistanceAngleFloorShot() {
    Pose2d goalPose;
    if (FmsSubsystem.isRedAlliance()) {
      goalPose = RED_FLOOR_SPOT;
    } else {
      goalPose = BLUE_FLOOR_SPOT;
    }

    Logger.recordOutput("Vision/FloorSpot", goalPose);

    return distanceToTargetPose(goalPose, robotPose, false);
  }

  public double getStandardDeviation(double distance) {

    return distanceToDev.get(distance);
  }

  public double getDistanceFromAngle(double angle) {
    return angleToDistance.get(angle);
  }

  public Pose3d getAllianceDoubleTagCenterPose() {
    Pose3d goalPose;
    if (FmsSubsystem.isRedAlliance()) {
      goalPose = RED_SPEAKER_DOUBLE_TAG_CENTER;
    } else {
      goalPose = BLUE_SPEAKER_DOUBLE_TAG_CENTER;
    }

    return goalPose;
  }

  public void setRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public Pose2d getUsedRobotPose() {
    return robotPose;
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput(
        "Vision/DistanceFromSpeaker", Units.metersToInches(getDistanceAngleSpeaker().distance()));
    Logger.recordOutput("Vision/AngleFromSpeaker", getDistanceAngleSpeaker().angle().getDegrees());
    Logger.recordOutput("Vision/AngleFromSpeaker", getDistanceAngleSpeaker().angle().getDegrees());
    Logger.recordOutput("Vision/DistanceFromFloorSpot", getDistanceAngleFloorShot().distance());
    Logger.recordOutput("Vision/AngleFromFloorSpot", getDistanceAngleFloorShot().angle());
    Logger.recordOutput("Vision/State", getState());

    // TODO: Figure out the center tag IDs for red & blue speaker, update these to match
    if (FmsSubsystem.isRedAlliance()) {
      LimelightHelpers.setPriorityTagID("", 123);
    } else {
      LimelightHelpers.setPriorityTagID("", 123);
    }

    var newHeartbeat = LimelightHelpers.getLimelightNTDouble("", "hb");

    if (limelightHeartbeat == newHeartbeat) {
      // No new data, Limelight dead?
    } else {
      limelightTimer.restart();
    }

    limelightHeartbeat = newHeartbeat;
  }

  public VisionState getState() {
    if (limelightTimer.hasElapsed(5)) {
      // Heartbeat hasn't updated
      return VisionState.OFFLINE;
    }

    if (LimelightHelpers.getTV("")) {
      return VisionState.SEES_TAGS;
    }

    return VisionState.ONLINE_NO_TAGS;
  }
}
