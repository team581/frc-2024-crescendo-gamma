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
  private static final double SHOOT_WHILE_MOVE_LOOKAHEAD = 0.1;
  private static final boolean USE_SHOOT_WHILE_MOVE = false;

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private final VisionSubsystem vision;
  private double lastAddedVisionTimestamp = 0;

  private final TimedDataBuffer xHistory = new TimedDataBuffer(8);
  private final TimedDataBuffer yHistory =
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

    var maybeResults = vision.getResults();
    var timestamp = Timer.getFPGATimestamp();

    if (maybeResults.isPresent()) {
      var results = maybeResults.get();
      Pose2d visionPose = results.robotPose().toPose2d();
      double limelightStandardDeviation = vision.getStandardDeviation(results.latency());

      double visionTimestamp = timestamp - results.latency();

      if (visionTimestamp == lastAddedVisionTimestamp) {
        // Don't add the same vision pose over and over
      } else {
        poseEstimator.addVisionMeasurement(
            visionPose,
            visionTimestamp,
            VecBuilder.fill(
                limelightStandardDeviation,
                limelightStandardDeviation,
                limelightStandardDeviation));
        lastAddedVisionTimestamp = visionTimestamp;
      }
    }

    Logger.recordOutput("Localization/OdometryPose", getOdometryPose());
    Logger.recordOutput("Localization/EstimatedPose", getPose());
    Logger.recordOutput("Localization/LimelightPose", LimelightHelpers.getBotPose2d_wpiBlue(""));

    xHistory.addData(timestamp, getPose().getX());
    yHistory.addData(timestamp, getPose().getY());

    vision.setRobotPose(getExpectedPose(SHOOT_WHILE_MOVE_LOOKAHEAD));
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

  public Pose2d getExpectedPose(double lookAhead) {
    var velocities = swerve.getRobotRelativeSpeeds();
    var angularVelocity = imu.getRobotAngularVelocity();

    var xDifference =
        imu.getXAcceleration() * Math.pow(lookAhead, 2) / 2
            + velocities.vxMetersPerSecond * lookAhead;
    var yDifference =
        imu.getYAcceleration() * Math.pow(lookAhead, 2) / 2
            + velocities.vyMetersPerSecond * lookAhead;
    var thetaDifference = new Rotation2d(angularVelocity.getRadians() * lookAhead);

    var expectedPose =
        new Pose2d(
            new Translation2d(xDifference + getPose().getX(), yDifference + getPose().getY()),
            thetaDifference.plus(imu.getRobotHeading()));

    Logger.recordOutput("Localization/ExpectedPose", expectedPose);

    if (!USE_SHOOT_WHILE_MOVE) {
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
