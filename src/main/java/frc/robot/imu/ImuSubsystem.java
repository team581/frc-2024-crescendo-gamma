// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ImuSubsystem extends LifecycleSubsystem {
  private final Pigeon2 imu;
  private final InterpolatingDoubleTreeMap distanceToAngleTolerance =
      new InterpolatingDoubleTreeMap();
  private final TimeInterpolatableBuffer<Double> robotHeadingHistory =
      TimeInterpolatableBuffer.createDoubleBuffer(3);

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
    Rotation2d robotHeading = this.getRobotHeading();
    Logger.recordOutput("Imu/RobotHeading", robotHeading.getDegrees());
    Logger.recordOutput(
        "Imu/RobotHeadingModulo", MathUtil.inputModulus(robotHeading.getDegrees(), 0, 360));
    Logger.recordOutput("Imu/RobotHeadingRadians", robotHeading.getRadians());

    var yaw = this.imu.getYaw();

    double yawOffset = Utils.getCurrentTimeSeconds() - yaw.getTimestamp().getTime();

    robotHeadingHistory.addSample(
        Timer.getFPGATimestamp() - yawOffset, Units.degreesToRadians(yaw.getValueAsDouble()));
  }

  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(imu.getYaw().getValue());
  }

  public Rotation2d getRobotHeading(double timestamp) {
    return Rotation2d.fromRadians(
        robotHeadingHistory.getSample(timestamp).orElseGet(() -> getRobotHeading().getRadians()));
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(imu.getPitch().getValue());
  }

  public Rotation2d getPitchRate() {
    return Rotation2d.fromDegrees(imu.getAngularVelocityYWorld().getValue());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(imu.getRoll().getValue());
  }

  public Rotation2d getRollRate() {
    return Rotation2d.fromDegrees(imu.getAngularVelocityXWorld().getValue());
  }

  public Rotation2d getRobotAngularVelocity() {
    return Rotation2d.fromDegrees(imu.getRate());
  }

  public void setAngle(Rotation2d zeroAngle) {
    this.imu.setYaw(zeroAngle.getDegrees());
  }

  private boolean atAngle(Rotation2d angle, Rotation2d tolerance) {
    return Math.abs(getRobotHeading().minus(angle).getDegrees()) < tolerance.getDegrees();
  }

  public boolean belowVelocityForSpeaker(double distance) {
    return getRobotAngularVelocity().getDegrees() < 10;
  }

  public boolean atAngleForSpeaker(Rotation2d angle, double distance) {
    return atAngle(angle, Rotation2d.fromDegrees(2.5));
  }

  public boolean atAngleForFloorSpot(Rotation2d angle) {
    return atAngle(angle, Rotation2d.fromDegrees(10));
  }

  public double getYAcceleration() {
    return imu.getAccelerationY().getValueAsDouble();
  }

  public double getXAcceleration() {
    return imu.getAccelerationX().getValueAsDouble();
  }
}
