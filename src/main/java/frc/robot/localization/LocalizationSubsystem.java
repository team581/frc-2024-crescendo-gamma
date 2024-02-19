// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.PoseWithTagID;
import frc.robot.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class LocalizationSubsystem extends LifecycleSubsystem {
  private static final double SHOOT_WHILE_MOVE_LOOKAHEAD = 0.5;
  private static final boolean USE_SHOOT_WHILE_MOVE = true;

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveOdometry odometry;
  private final VisionSubsystem vision;

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
    odometry.update(
        imu.getRobotHeading(), swerve.getModulePositions().toArray(new SwerveModulePosition[4]));
    poseEstimator.update(
        imu.getRobotHeading(), swerve.getModulePositions().toArray(new SwerveModulePosition[4]));

    try {
      var results = vision.getFilteredAprilTags();
      for (PoseWithTagID fiducial : results.tags()) {
        poseEstimator.addVisionMeasurement(fiducial.robotPose().toPose2d(), results.latency());
      }
    } catch (Exception e) {
    }

    Logger.recordOutput("Localization/OdometryPose", getOdometryPose());
    Logger.recordOutput("Localization/EstimatedPose", getPose());
    Logger.recordOutput("Localization/ExpectedPose", getExpectedPose(SHOOT_WHILE_MOVE_LOOKAHEAD));
    Logger.recordOutput("Localization/AccelerationX", imu.getXAcceleration());
    Logger.recordOutput("Localization/AccelerationY", imu.getYAcceleration());
    Logger.recordOutput(
        "Localization/VelocityX", swerve.getRobotRelativeSpeeds().vxMetersPerSecond);
    Logger.recordOutput(
        "Localization/VelocityY", swerve.getRobotRelativeSpeeds().vyMetersPerSecond);
    Logger.recordOutput(
        "Localization/VelocityTheta", swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond);
    Logger.recordOutput("Localization/LimelightPose", LimelightHelpers.getBotPose2d_wpiBlue(""));

    vision.setRobotPose(getExpectedPose(SHOOT_WHILE_MOVE_LOOKAHEAD));
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    imu.setAngle(pose.getRotation());
    poseEstimator.resetPosition(
        pose.getRotation(), swerve.getModulePositions().toArray(new SwerveModulePosition[4]), pose);
    odometry.resetPosition(
        pose.getRotation(), swerve.getModulePositions().toArray(new SwerveModulePosition[4]), pose);
  }

  public void resetGyro(Rotation2d gyroAngle) {
    Pose2d pose = new Pose2d(getPose().getTranslation(), gyroAngle);
    resetPose(pose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 0 : 180)));
  }

  public Pose2d getExpectedPose(double lookAhead) {
    if (!USE_SHOOT_WHILE_MOVE) {
      return getPose();
    }

    var velocities = swerve.getRobotRelativeSpeeds();
    var angularVelocity = imu.getRobotAngularVelocity();

    var xDifference =
        (velocities.vxMetersPerSecond + (imu.getXAcceleration() * lookAhead)) * lookAhead;
    var yDifference =
        (velocities.vyMetersPerSecond + (imu.getYAcceleration() * lookAhead)) * lookAhead;
    var thetaDifference = new Rotation2d(angularVelocity.getRadians() * lookAhead);

    return new Pose2d(
        new Translation2d(xDifference + getPose().getX(), yDifference + getPose().getY()),
        thetaDifference.plus(imu.getRobotHeading()));
  }
}
