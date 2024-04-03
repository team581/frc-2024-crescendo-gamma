// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends LifecycleSubsystem {
  private static final boolean CALIBRATION_RIG_ENABLED = true;
  private static final boolean SHOOT_TO_SIDE_ENABLED = true;

  public static final Pose2d ORIGINAL_RED_SPEAKER =
      new Pose2d(
          Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
  public static final Pose2d ORIGINAL_BLUE_SPEAKER =
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));

  public static final Pose2d RED_FLOOR_SPOT = new Pose2d(15.5, 8.0, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_FLOOR_SPOT = new Pose2d(1, 8.0, Rotation2d.fromDegrees(0));

  // TODO: Update this
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

  public static final Pose3d ROBOT_TO_CALIBRATION_TAG_CENTER =
      new Pose3d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(57.128),
          Units.inchesToMeters(-64.75),
          new Rotation3d(0, 0, 0));

  public static void logCalibration() {
    var json = LimelightHelpers.getLatestResults("");

    for (LimelightTarget_Fiducial fiducial : json.targetingResults.targets_Fiducials) {
      var prefix = "Vision/Calibration/Tag" + fiducial.fiducialID + "/";

      Pose3d camPoseRelativeTag = fiducial.getCameraPose_TargetSpace();

      Transform3d camPoseRelativeRobot = ROBOT_TO_CALIBRATION_TAG_CENTER.minus(camPoseRelativeTag);
      DogLog.log(prefix + "camRelativeTag", camPoseRelativeTag);
      DogLog.log(prefix + "camRelativeRobot", camPoseRelativeRobot);
    }
  }

  private final Timer limelightTimer = new Timer();
  private double limelightHeartbeat = -1;

  private final InterpolatingDoubleTreeMap distanceToDev = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap angleToSideShotOffset = new InterpolatingDoubleTreeMap();

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

    angleToSideShotOffset.put(Units.degreesToRadians(100), Units.degreesToRadians(5.0));
    angleToSideShotOffset.put(Units.degreesToRadians(40), Units.degreesToRadians(1.75));
    angleToSideShotOffset.put(Units.degreesToRadians(30), Units.degreesToRadians(0.0));
    angleToSideShotOffset.put(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0));

    limelightTimer.start();
  }

  private static Pose2d getSpeaker() {
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

    double verticalDistance = getAllianceDoubleTagCenterPose().getZ() - CAMERA_ON_BOT.getZ();

    double distance =
        verticalDistance / (Math.tan(ty.getRadians() + CAMERA_ON_BOT.getRotation().getY()));
    double now = Timer.getFPGATimestamp();
    double latency =
        (LimelightHelpers.getLatency_Capture("") + LimelightHelpers.getLatency_Pipeline(""))
            / 1000.0;

    double timestampAtCapture = now - latency;

    var robotHeading = imu.getRobotHeading(timestampAtCapture);

    Rotation2d angle = Rotation2d.fromDegrees(robotHeading.getDegrees() - tx.getDegrees());

    return Optional.of(new DistanceAngle(distance, angle, true));
  }

  public DistanceAngle getDistanceAngleSpeaker() {
    var maybeTxTyDistanceAngle = getDistanceAngleTxTy();

    Pose2d speakerPose = getSpeaker();

    DistanceAngle distanceToTargetPose = distanceToTargetPose(speakerPose, robotPose);

    Logger.recordOutput("Vision/MegaTag2/SpeakerPose", speakerPose);
    Logger.recordOutput("Vision/MegaTag2/WantedRobotAngle", distanceToTargetPose.targetAngle());
    Logger.recordOutput("Vision/MegaTag2/RobotDistance", distanceToTargetPose.distance());

    if (maybeTxTyDistanceAngle.isPresent()) {
      Logger.recordOutput("Vision/TxTy/Distance", maybeTxTyDistanceAngle.get().distance());
      Logger.recordOutput(
          "Vision/TxTy/WantedRobotAngle", maybeTxTyDistanceAngle.get().targetAngle().getDegrees());

      if (RobotConfig.get().vision().strategy() == VisionStrategy.TX_TY_AND_MEGATAG) {
        return adjustForSideShot(maybeTxTyDistanceAngle.get());
      }
    }

    return adjustForSideShot(distanceToTargetPose);
  }

  private DistanceAngle adjustForSideShot(DistanceAngle originalPosition) {
    if (!SHOOT_TO_SIDE_ENABLED) {
      return originalPosition;
    }

    double rawAngleDegrees = originalPosition.targetAngle().getDegrees();
    double angleDegrees = MathUtil.inputModulus(rawAngleDegrees, -180.0, 180.0);

    // if (FmsSubsystem.isRedAlliance()) {
    //   angleDegrees += 180.0;
    // }

    DogLog.log("Vision/ShootToTheSide/InputAngleRelativeToSpeaker", angleDegrees);
    double absoluteOffsetRadians =
        (angleToSideShotOffset.get(Units.degreesToRadians(Math.abs(angleDegrees))));
    double offsetRadians = Math.copySign(absoluteOffsetRadians, angleDegrees);
    Logger.recordOutput("Vision/ShootToTheSide/Offset", Units.radiansToDegrees(offsetRadians));

    var adjustedAngle =
        Rotation2d.fromRadians(originalPosition.targetAngle().getRadians() + offsetRadians);

    DogLog.log(
        "Vision/ShootToTheSide/OriginalShotPose",
        new Pose2d(robotPose.getTranslation(), originalPosition.targetAngle()));
    DogLog.log(
        "Vision/ShootToTheSide/AdjustedResultShotPose",
        new Pose2d(robotPose.getTranslation(), adjustedAngle));

    return new DistanceAngle(
        originalPosition.distance(), adjustedAngle, originalPosition.seesSpeakerTag());
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

  public static Pose3d getAllianceDoubleTagCenterPose() {
    if (FmsSubsystem.isRedAlliance()) {
      return RED_SPEAKER_DOUBLE_TAG_CENTER;
    } else {
      return BLUE_SPEAKER_DOUBLE_TAG_CENTER;
    }
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

    if (CALIBRATION_RIG_ENABLED) {

      logCalibration();
    }
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
