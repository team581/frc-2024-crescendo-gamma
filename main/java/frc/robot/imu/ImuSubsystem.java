// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotConfig;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ImuSubsystem extends LifecycleSubsystem {
  private final Pigeon2 imu;
  private Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.5);
  private static final InterpolatingDoubleTreeMap distanceToAngleTolerance =
      new InterpolatingDoubleTreeMap();

  public ImuSubsystem(SwerveSubsystem swerve) {
    super(SubsystemPriority.IMU);

    this.imu = swerve.drivetrainPigeon;

    RobotConfig.get().imu().distanceToAngleTolerance().accept(distanceToAngleTolerance);

    Pigeon2Configuration config =
        new Pigeon2Configuration()
            .withMountPose(
                new MountPoseConfigs()
                    .withMountPosePitch(0)
                    .withMountPoseRoll(0)
                    .withMountPoseYaw(0));

    imu.getConfigurator().apply(config);
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Imu/RobotHeading", this.getRobotHeading().getDegrees());
    Logger.recordOutput("Imu/RobotHeadingRadians", this.getRobotHeading().getRadians());
  }

  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(imu.getYaw().getValue());
  }

  public Rotation2d getRobotAngularVelocity() {
    return Rotation2d.fromDegrees(imu.getRate());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(imu.getRoll().getValue());
  }

  public void setAngle(Rotation2d zeroAngle) {
    this.imu.setYaw(zeroAngle.getDegrees());
  }

  public boolean atAngle(Rotation2d angle) {
    return Math.abs(getRobotHeading().minus(angle).getDegrees()) < 3;
  }

  public Rotation2d getAngleToleranceFromDistanceToSpeaker(double distance) {
    return distance > 1.0 // Minimum distance
        ? Rotation2d.fromDegrees(2.5) // Minimum tolerance
        : distance < 1.0 // Max distance
            ? Rotation2d.fromDegrees(2.5) // max tolerance
            : Rotation2d.fromDegrees(distanceToAngleTolerance.get(distance));
  }

  public void setTolerance(Rotation2d tolerance) {
    TOLERANCE = tolerance;
  }

  public Rotation2d getTolerance() {
    return TOLERANCE;
  }

  public boolean atAngle(Rotation2d angle, double distance) {
    setTolerance(getAngleToleranceFromDistanceToSpeaker(distance));
    return atAngle(angle);
  }
}
