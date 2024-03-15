// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.TimedDataBuffer;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class LocalizationSubsystem extends LifecycleSubsystem {
  private static final double SHOOT_WHILE_MOVE_LOOKAHEAD = 0.2;
  public static final boolean USE_SHOOT_WHILE_MOVE = false;

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private final VisionSubsystem vision;
  private Pose2d savedExpected = new Pose2d();
  private double lastAddedVisionTimestamp = 0;
  private int loops = 0;
  private double vector = 0;

  private final TimedDataBuffer xHistory =
      new TimedDataBuffer(RobotConfig.get().vision().translationHistoryArraySize());
  private final TimedDataBuffer yHistory =
      new TimedDataBuffer(RobotConfig.get().vision().translationHistoryArraySize());
  private final TimedDataBuffer distanceToSavedHistory =
      new TimedDataBuffer(RobotConfig.get().vision().translationHistoryArraySize());

  public LocalizationSubsystem(SwerveSubsystem swerve, ImuSubsystem imu, VisionSubsystem vision) {
    super(SubsystemPriority.LOCALIZATION);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;
    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveSubsystem.KINEMATICS,
            imu.getRobotHeading(),
            swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
            new Pose2d());
    odometry =
        new SwerveDriveOdometry(
            SwerveSubsystem.KINEMATICS,
            imu.getRobotHeading(),
            swerve.getModulePositions().toArray(new SwerveModulePosition[4]));
  }

  @Override
  public void robotPeriodic() {
    SwerveModulePosition[] modulePositions =
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]);
    odometry.update(imu.getRobotHeading(), modulePositions);
    poseEstimator.update(imu.getRobotHeading(), modulePositions);

    var maybeResults = vision.getSimpleSpeakerBaseResults();
    var timestamp = Timer.getFPGATimestamp();

    if (maybeResults.isPresent()) {
      var results = maybeResults.get();
      Pose2d visionPose = results.pose();

      double visionTimestamp = results.totalLatency();

      if (visionTimestamp == lastAddedVisionTimestamp) {
        // Don't add the same vision pose over and over
      } else {
        poseEstimator.addVisionMeasurement(
            visionPose,
            visionTimestamp,
            VecBuilder.fill(
                RobotConfig.get().vision().xyStdDev(),
                RobotConfig.get().vision().xyStdDev(),
                RobotConfig.get().vision().thetaStdDev()));
        lastAddedVisionTimestamp = visionTimestamp;
      }
    }

    Logger.recordOutput("Localization/OdometryPose", getOdometryPose());
    Logger.recordOutput("Localization/SavedExpectedPose", getSavedExpectedPose(false));
    Logger.recordOutput("Localization/EstimatedPose", getPose());
    Logger.recordOutput("Localization/ChangedDirection", changedDirection());
    Logger.recordOutput(
        "Localization/ExpectedPose", getExpectedPose(SHOOT_WHILE_MOVE_LOOKAHEAD, true));
    Logger.recordOutput("Localization/LimelightPose", LimelightHelpers.getBotPose2d_wpiBlue(""));

    xHistory.addData(timestamp, getPose().getX());
    yHistory.addData(timestamp, getPose().getY());
    distanceToSavedHistory.addData(
        timestamp,
        Math.sqrt(
            Math.pow(getPose().getX() - getSavedExpectedPose(false).getX(), 2)
                + Math.pow(getPose().getY() - getSavedExpectedPose(false).getY(), 2)));

    // vision.setRobotPose(getExpectedPose(SHOOT_WHILE_MOVE_LOOKAHEAD, USE_SHOOT_WHILE_MOVE));
    // TODO: Broken, makes the robot spin in circles slowly when shooting
    vision.setRobotPose(getSavedExpectedPose(true));
    loops++;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    resetPose(pose, pose);
  }

  public Pose2d getSavedExpectedPose(boolean reloadLoops) {
    if (USE_SHOOT_WHILE_MOVE) {
      if ((loops >= SHOOT_WHILE_MOVE_LOOKAHEAD * 50.0)
          && !atPose(getPose().getTranslation(), savedExpected.getTranslation())
      // || changedDirection()
      ) {
        savedExpected = getExpectedPose(SHOOT_WHILE_MOVE_LOOKAHEAD, USE_SHOOT_WHILE_MOVE);
        if (reloadLoops) {
          loops = 0;
        }
      }
    } else {
      savedExpected = getPose();
    }
    return savedExpected;
  }

  private static boolean atPose(Translation2d poseOne, Translation2d poseTwo) {
    return poseOne.getDistance(poseTwo) < 0.1;
  }

  public void resetPose(Pose2d estimatedPose, Pose2d odometryPose) {
    imu.setAngle(odometryPose.getRotation());
    poseEstimator.resetPosition(
        estimatedPose.getRotation(),
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
        estimatedPose);
    odometry.resetPosition(
        odometryPose.getRotation(),
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
        odometryPose);
  }

  private void resetGyro(Rotation2d gyroAngle) {
    Pose2d estimatedPose = new Pose2d(getPose().getTranslation(), gyroAngle);
    Pose2d odometryPose = new Pose2d(getOdometryPose().getTranslation(), gyroAngle);
    resetPose(estimatedPose, odometryPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 0 : 180)));
  }

  public boolean changedDirection() {
    var hist = distanceToSavedHistory;
    if (hist.lookupData(0) - hist.lookupData(1) > 0.1) {
      return true;
    }
    return false;
  }

  public Pose2d getExpectedPose(double lookAhead, boolean shootWhileMove) {
    var velocities = swerve.getRobotRelativeSpeeds();
    var angularVelocity = Rotation2d.fromDegrees(imu.getRobotAngularVelocity().getDegrees() * 1.00);
    if (Math.abs(angularVelocity.getDegrees()) < 0.1) {
      angularVelocity = Rotation2d.fromDegrees(0);
    }
    var xDifference =
        imu.getXAcceleration() * Math.pow(lookAhead, 2) / 2
            + velocities.vxMetersPerSecond * lookAhead;
    var yDifference =
        imu.getYAcceleration() * Math.pow(lookAhead, 2) / 2
            + velocities.vyMetersPerSecond * lookAhead;
    var thetaDifference = new Rotation2d(angularVelocity.getRadians() * lookAhead * -1);

    var expectedPose =
        new Pose2d(
            new Translation2d(xDifference + getPose().getX(), yDifference + getPose().getY()),
            thetaDifference.plus(imu.getRobotHeading()));
    boolean movingSlowEnough = false;
    vector = Math.sqrt(Math.pow(xDifference, 2) + Math.pow(yDifference, 2));

    if (vector < 0.05) {
      movingSlowEnough = true;
    } else {
      movingSlowEnough = false;
    }
    Logger.recordOutput("Localization/movingSlowEnough", movingSlowEnough);
    Logger.recordOutput("Localization/xDifference", xDifference);
    Logger.recordOutput("Localization/yDifference", yDifference);
    Logger.recordOutput("Localization/Vector", vector);
    if (!shootWhileMove || movingSlowEnough) {
      return getPose();
    }

    return expectedPose;
  }

  public boolean atSafeJitter() {
    // Get first value of X & Y from history
    double xDifference =
        Math.abs(xHistory.lookupData(-Double.POSITIVE_INFINITY) - getPose().getX());
    double yDifference =
        Math.abs(yHistory.lookupData(-Double.POSITIVE_INFINITY) - getPose().getY());

    ChassisSpeeds speeds = new ChassisSpeeds(xDifference, yDifference, 0);

    // This doesn't check angular velocity, because we trust that to be correct & not have jitter
    // X & Y from pose estimator have the jitter
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < Units.feetToMeters(0.5);
  }
}
