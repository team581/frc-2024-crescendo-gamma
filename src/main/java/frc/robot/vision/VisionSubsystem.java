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
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    if (estimatePose.tagCount == 0) {
      return Optional.empty();
    }

    for (int i = 0; i < estimatePose.rawFiducials.length; i++) {
      var fiducial = estimatePose.rawFiducials[i];

      if (fiducial.ambiguity > 0.5) {
        return Optional.empty();
      }
    }

    return Optional.of(new VisionResult(estimatePose.pose, estimatePose.timestampSeconds));
  }

  public static DistanceAngle distanceToTargetPose(Pose2d target, Pose2d current) {
    double distance = target.getTranslation().getDistance(current.getTranslation());
    Rotation2d angle =
        Rotation2d.fromRadians(
            Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));

    return new DistanceAngle(distance, angle, false);
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

  public Optional<DistanceAngle> getDistanceAngleTxTy() {
    if (LimelightHelpers.getTA("") == 0.0) {
      return Optional.empty();
    }

    Rotation2d tx = Rotation2d.fromDegrees(LimelightHelpers.getTX(""));
    Rotation2d ty = Rotation2d.fromDegrees(LimelightHelpers.getTY(""));

    // TODO: update to use blue if on blue, since heights may be different at comp
    double verticalDistance = getAllianceDoubleTagCenterPose().getZ() - CAMERA_ON_BOT.getZ();

    double distance =
        verticalDistance / (Math.tan(ty.getRadians() + CAMERA_ON_BOT.getRotation().getY()));
    double now = Timer.getFPGATimestamp();
    double latency =
        (LimelightHelpers.getLatency_Capture("") + LimelightHelpers.getLatency_Pipeline(""))
            / 1000.0;
    double timestampAtCapture = now - latency;

    var robotHeading = imu.getRobotHeading(timestampAtCapture);

    double txToRobotScalar = 1.0;

    Rotation2d angle =
        Rotation2d.fromDegrees(robotHeading.getDegrees() - (tx.getDegrees() * txToRobotScalar));
    return Optional.of(new DistanceAngle(distance, angle, true));
  }

  public DistanceAngle getDistanceAngleSpeaker() {
    var maybeTxTyDistanceAngle = getDistanceAngleTxTy();

    Pose2d goalPose = getSpeaker();

    var adjustedPose = goalPose;

    // T
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
    DistanceAngle distanceToTargetPose = distanceToTargetPose(adjustedPose, robotPose);

    Logger.recordOutput("Vision/OriginalSpeakerPose", goalPose);
    Logger.recordOutput("Vision/SpeakerPose", adjustedPose);

    Logger.recordOutput("Vision/PoseAngle", distanceToTargetPose.targetAngle());
    Logger.recordOutput("Vision/PoseDistance", distanceToTargetPose.distance());

    if (maybeTxTyDistanceAngle.isPresent()) {

      Logger.recordOutput("Vision/TxTyDistance", maybeTxTyDistanceAngle.get().distance());
      Logger.recordOutput("Vision/TXTYAngle", maybeTxTyDistanceAngle.get().targetAngle());

      if (RobotConfig.get().vision().strategy() == VisionStrategy.TX_TY_AND_MEGATAG) {
        return maybeTxTyDistanceAngle.get();
      }
    }

    return distanceToTargetPose;
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

  @Override
  public void robotPeriodic() {
    if (FmsSubsystem.isRedAlliance()) {
      LimelightHelpers.setPriorityTagID("", 4);
    } else {
      LimelightHelpers.setPriorityTagID("", 7);
    }
    Logger.recordOutput("Vision/DistanceFromSpeaker", getDistanceAngleSpeaker().distance());
    Logger.recordOutput(
        "Vision/AngleFromSpeaker", getDistanceAngleSpeaker().targetAngle().getDegrees());
    Logger.recordOutput("Vision/DistanceFromFloorSpot", getDistanceAngleFloorShot().distance());
    Logger.recordOutput("Vision/AngleFromFloorSpot", getDistanceAngleFloorShot().targetAngle());
    Logger.recordOutput("Vision/State", getState());
    Logger.recordOutput("Vision/VisionMethod", RobotConfig.get().vision().strategy());

    var newHeartbeat = LimelightHelpers.getLimelightNTDouble("", "hb");

    if (limelightHeartbeat == newHeartbeat) {
      // No new data, Limelight dead?
    } else {
      limelightTimer.restart();
    }

    limelightHeartbeat = newHeartbeat;

    LimelightHelpers.SetRobotOrientation(
        "",
        imu.getRobotHeading().getDegrees(),
        imu.getRobotAngularVelocity().getDegrees(),
        imu.getPitch().getDegrees(),
        imu.getPitchRate().getDegrees(),
        imu.getRoll().getDegrees(),
        imu.getRollRate().getDegrees());
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
