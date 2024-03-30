// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends LifecycleSubsystem {
  private static final double MAX_SPEED_SHOOTING =
      Units.feetToMeters(LocalizationSubsystem.USE_SHOOT_WHILE_MOVE ? 3 : 3);
  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(3);
  private boolean isShooting = false;

  private double leftXDeadband = 0.05;
  private double rightXDeadband = 0.05;
  private double leftYDeadband = 0.05;

  public static final Translation2d FRONT_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontLeft.LocationX, CommandSwerveDrivetrain.FrontLeft.LocationY);
  public static final Translation2d FRONT_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontRight.LocationX,
          CommandSwerveDrivetrain.FrontRight.LocationY);
  public static final Translation2d BACK_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackLeft.LocationX, CommandSwerveDrivetrain.BackLeft.LocationY);
  public static final Translation2d BACK_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackRight.LocationX, CommandSwerveDrivetrain.BackRight.LocationY);
  public static final Translation2d[] MODULE_LOCATIONS =
      new Translation2d[] {
        FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION
      };
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_LOCATIONS);
  private final double TimeConstant = 0.2;
  private final double AccelerationLimit = 3.2;
  private Translation2d previousVelocity = new Translation2d();

  /**
   * Helper for applying current limits to swerve modules, since the CTR swerve builder API doesn't
   * support it.
   */
  private static void applyCurrentLimits(SwerveModule module) {
    module
        .getDriveMotor()
        .getConfigurator()
        .apply(RobotConfig.get().swerve().driveMotorCurrentLimits());
    module
        .getSteerMotor()
        .getConfigurator()
        .apply(RobotConfig.get().swerve().steerMotorCurrentLimits());
    module
        .getDriveMotor()
        .getConfigurator()
        .apply(RobotConfig.get().swerve().driveMotorTorqueCurrentLimits());
  }

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private boolean snapToAngle = false;
  private Rotation2d goalSnapAngle = new Rotation2d();
  private final CommandXboxController controller;

  // My drivetrain
  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();

  public final Pigeon2 drivetrainPigeon = drivetrain.getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);
  private final SwerveModule frontLeft = drivetrain.getModule(0);
  private final SwerveModule frontRight = drivetrain.getModule(1);
  private final SwerveModule backLeft = drivetrain.getModule(2);
  private final SwerveModule backRight = drivetrain.getModule(3);
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private boolean closedLoop = false;

  // TODO: tune
  private final PIDController xPid = new PIDController(0.5, 0, 0);
  private final PIDController yPid = new PIDController(0.5, 0, 0);
  private final PIDController omegaPid = new PIDController(0.5, 0, 0);

  public SwerveSubsystem(CommandXboxController driveController) {
    super(SubsystemPriority.SWERVE);
    this.controller = driveController;

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    driveToAngle.HeadingController = RobotConfig.get().swerve().snapController();
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    applyCurrentLimits(frontLeft);
    applyCurrentLimits(frontRight);
    applyCurrentLimits(backLeft);
    applyCurrentLimits(backRight);
  }

  public List<SwerveModulePosition> getModulePositions() {
    return List.of(
        frontLeft.getPosition(true),
        frontRight.getPosition(true),
        backLeft.getPosition(true),
        backRight.getPosition(true));
  }

  public void setSnapToAngle(Rotation2d angle) {
    goalSnapAngle = angle;
    snapToAngle = true;
  }

  public void disableSnapToAngle() {
    snapToAngle = false;
  }

  public void setFieldRelativeSpeeds(ChassisSpeeds speeds, boolean closedLoop) {
    this.fieldRelativeSpeeds = speeds;
    this.closedLoop = closedLoop;
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds, boolean closedLoop) {
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drivetrain.getState().Pose.getRotation());
    setFieldRelativeSpeeds(fieldRelative, closedLoop);
  }

  public Command driveTeleopCommand() {
    return run(
        () -> {
          if (!DriverStation.isTeleop()) {
            return;
          }
          double leftY =
              -1.0
                  * ControllerHelpers.getExponent(
                      ControllerHelpers.getDeadbanded(controller.getLeftY(), leftYDeadband), 1.5);
          double leftX =
              ControllerHelpers.getExponent(
                  ControllerHelpers.getDeadbanded(controller.getLeftX(), leftXDeadband), 1.5);
          double rightX =
              ControllerHelpers.getExponent(
                  ControllerHelpers.getDeadbanded(controller.getRightX(), rightXDeadband), 2);

          if (RobotConfig.get().swerve().invertRotation()) {
            rightX *= -1.0;
          }

          if (RobotConfig.get().swerve().invertX()) {
            leftX *= -1.0;
          }

          if (RobotConfig.get().swerve().invertY()) {
            leftY *= -1.0;
          }

          if (FmsSubsystem.isRedAlliance()) {
            leftX *= -1.0;
            leftY *= -1.0;
          }

          Logger.recordOutput("Swerve/Controller/FinalLeftX", leftX);
          Logger.recordOutput("Swerve/Controller/FinalRightX", rightX);
          Logger.recordOutput("Swerve/Controller/FinalLeftY", leftY);

          ChassisSpeeds teleopSpeeds =
              new ChassisSpeeds(
                  -1.0 * leftY * MaxSpeed,
                  leftX * MaxSpeed,
                  rightX * TELEOP_MAX_ANGULAR_RATE.getRadians());

          Logger.recordOutput("Swerve/RawTeleopSpeeds", teleopSpeeds);

          // teleopSpeeds = accelerationLimitChassisSpeeds(teleopSpeeds);

          double currentSpeed =
              Math.sqrt(
                  Math.pow(teleopSpeeds.vxMetersPerSecond, 2)
                      + Math.pow(teleopSpeeds.vyMetersPerSecond, 2));
          Logger.recordOutput("Swerve/CurrentSpeed", currentSpeed);
          var scaled = teleopSpeeds.div(currentSpeed / MAX_SPEED_SHOOTING);
          Logger.recordOutput("Swerve/ScaledSpeeds", scaled);
          Logger.recordOutput("Swerve/IsShooting", isShooting);
          Logger.recordOutput("Swerve/GoingToFast", currentSpeed > MAX_SPEED_SHOOTING);
          if (isShooting) {
            if (currentSpeed > MAX_SPEED_SHOOTING) {
              teleopSpeeds =
                  new ChassisSpeeds(
                      scaled.vxMetersPerSecond,
                      scaled.vyMetersPerSecond,
                      teleopSpeeds.omegaRadiansPerSecond);
            }
          }

          Logger.recordOutput("Swerve/UsedTeleopSpeeds", teleopSpeeds);

          setFieldRelativeSpeeds(teleopSpeeds, false);
        });
  }

  private ChassisSpeeds accelerationLimitChassisSpeeds(ChassisSpeeds speeds) {

    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double maxVelocitychange = AccelerationLimit * TimeConstant;

    Translation2d velocityChange = (velocity.minus(previousVelocity));
    double velocityChangeAngle =
        Math.atan2(velocityChange.getY(), velocityChange.getX()); // Radians
    Translation2d limitedVelocityVectorChange = velocityChange;
    Translation2d limitedVelocityVector = velocity;

    double acceleration = velocity.getNorm() - previousVelocity.getNorm();
    Logger.recordOutput("Swerve/Acceleration", acceleration);

    if (velocityChange.getNorm() > maxVelocitychange && acceleration > 0) {
      limitedVelocityVectorChange =
          new Translation2d(maxVelocitychange, new Rotation2d(velocityChangeAngle));
      limitedVelocityVector = previousVelocity.plus(limitedVelocityVectorChange);
    }

    previousVelocity = limitedVelocityVector;

    return new ChassisSpeeds(
        limitedVelocityVector.getX(), limitedVelocityVector.getY(), speeds.omegaRadiansPerSecond);
  }

  public void setShootingMode(boolean value) {
    isShooting = value;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return KINEMATICS.toChassisSpeeds(drivetrain.getState().ModuleStates);
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Swerve/SnapToAngle", snapToAngle);
    Logger.recordOutput("Swerve/SnapToAngleGoal", goalSnapAngle.getDegrees());
    Logger.recordOutput("Swerve/Pose", drivetrain.getState().Pose);
    // TODO: Fix logging SwerveModuleState[] struct array
    DogLog.log("Swerve/ModuleStates/0", drivetrain.getState().ModuleStates[0]);
    DogLog.log("Swerve/ModuleStates/1", drivetrain.getState().ModuleStates[1]);
    DogLog.log("Swerve/ModuleStates/2", drivetrain.getState().ModuleStates[2]);
    DogLog.log("Swerve/ModuleStates/3", drivetrain.getState().ModuleStates[3]);
    Logger.recordOutput("Swerve/ModuleTargets", drivetrain.getState().ModuleTargets);

    Logger.recordOutput(
        "Swerve/FrontLeft/DriveMotor/Temperature",
        frontLeft.getDriveMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/FrontLeft/SteerMotor/Temperature",
        frontLeft.getSteerMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/FrontRight/DriveMotor/Temperature",
        frontRight.getDriveMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/FrontRight/SteerMotor/Temperature",
        frontRight.getSteerMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/BackLeft/DriveMotor/Temperature",
        backLeft.getDriveMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/BackLeft/SteerMotor/Temperature",
        backLeft.getSteerMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/BackRight/DriveMotor/Temperature",
        backRight.getDriveMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/BackRight/SteerMotor/Temperature",
        backRight.getSteerMotor().getDeviceTemp().getValueAsDouble());
    Logger.recordOutput(
        "Swerve/FrontLeft/DriveMotor/StatorCurrent",
        frontLeft.getDriveMotor().getStatorCurrent().getValue());
    Logger.recordOutput(
        "Swerve/FrontRight/DriveMotor/StatorCurrent",
        frontRight.getDriveMotor().getStatorCurrent().getValue());
    Logger.recordOutput(
        "Swerve/BackLeft/DriveMotor/StatorCurrent",
        backLeft.getDriveMotor().getStatorCurrent().getValue());
    Logger.recordOutput(
        "Swerve/BackRight/DriveMotor/StatorCurrent",
        backRight.getDriveMotor().getStatorCurrent().getValue());

    Logger.recordOutput("Swerve/RobotSpeed", getRobotRelativeSpeeds());

    DriveRequestType driveType;

    if (closedLoop) {
      driveType = DriveRequestType.Velocity;
    } else {
      driveType = DriveRequestType.OpenLoopVoltage;
    }

    if (snapToAngle) {
      drivetrain.setControl(
          driveToAngle
              .withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond)
              .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond)
              .withTargetDirection(goalSnapAngle)
              .withDriveRequestType(driveType));
    } else {
      drivetrain.setControl(
          drive
              .withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond)
              .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond)
              .withRotationalRate(fieldRelativeSpeeds.omegaRadiansPerSecond)
              .withDriveRequestType(driveType));
    }
  }

  public boolean snapsEnabled() {
    return snapToAngle;
  }

  public Rotation2d snapAngle() {
    return goalSnapAngle;
  }

  public boolean movingSlowEnoughForSpeakerShot() {
    return movingSlowEnoughForSpeakerShot(getRobotRelativeSpeeds());
  }

  public boolean movingSlowEnoughForSpeakerShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_SPEED_SHOOTING;
  }

  private boolean atLocation(Pose2d target, Pose2d current) {
    // Return true once at location
    double translationTolerance = 0.1;
    double omegaTolerance = 5;
    return Math.abs(current.getX() - target.getX()) <= translationTolerance
        && Math.abs(current.getY() - target.getY()) <= translationTolerance
        && Math.abs(current.getRotation().getDegrees() - target.getRotation().getDegrees())
            <= omegaTolerance;
  }

  public Command driveToPoseCommand(Supplier<Pose2d> targetSupplier, Supplier<Pose2d> currentPose) {
    return run(() -> {
          var target = targetSupplier.get();
          var pose = currentPose.get();
          double vx = xPid.calculate(pose.getX(), target.getX());
          double vy = yPid.calculate(pose.getY(), target.getY());
          double vomega =
              omegaPid.calculate(
                  Rotation2d.fromDegrees(drivetrain.getPigeon2().getYaw().getValueAsDouble())
                      .getRadians(),
                  target.getRotation().getRadians());

          var newSpeeds = new ChassisSpeeds(vx, vy, vomega);
          setFieldRelativeSpeeds(newSpeeds, true);
        })
        .until(
            () -> {
              var target = targetSupplier.get();

              return atLocation(target, currentPose.get());
            });
  }
}
