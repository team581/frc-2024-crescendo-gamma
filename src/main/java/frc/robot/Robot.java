// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Autos;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ClimberSubsystemStub;
import frc.robot.climber.IClimberSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  private final ShoulderSubsystem shoulder =
      new ShoulderSubsystem(
          new TalonFX(
              RobotConfig.get().shoulder().rightMotorID(), RobotConfig.get().canivoreName()),
          new TalonFX(
              RobotConfig.get().shoulder().leftMotorID(), RobotConfig.get().canivoreName()));
  private final ShooterSubsystem shooter =
      new ShooterSubsystem(
          new TalonFX(RobotConfig.get().shooter().leftMotorID(), RobotConfig.get().canivoreName()),
          new TalonFX(
              RobotConfig.get().shooter().rightMotorID(), RobotConfig.get().canivoreName()));
  private final IClimberSubsystem climber =
      RobotConfig.IS_PRACTICE_BOT
          ? new ClimberSubsystemStub()
          : new ClimberSubsystem(
              new TalonFX(
                  RobotConfig.get().climber().mainMotorID(), RobotConfig.get().canivoreName()),
              new TalonFX(
                  RobotConfig.get().climber().followerMotorID(), RobotConfig.get().canivoreName()));
  private final IntakeSubsystem intake =
      new IntakeSubsystem(
          new TalonFX(RobotConfig.get().intake().topMotorID(), RobotConfig.get().canivoreName()),
          new TalonFX(RobotConfig.get().intake().bottomMotorID(), RobotConfig.get().canivoreName()),
          new DigitalInput(RobotConfig.get().intake().sensorID()));
  private final SwerveSubsystem swerve = new SwerveSubsystem(driverController);
  private final ImuSubsystem imu = new ImuSubsystem(swerve);
  private final FmsSubsystem fms = new FmsSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final LocalizationSubsystem localization = new LocalizationSubsystem(swerve, imu, vision);
  private final SnapManager snaps = new SnapManager(swerve, driverController);
  private final RobotManager robotManager =
      new RobotManager(
          shoulder, intake, shooter, localization, vision, climber, swerve, snaps, imu);
  private final RobotCommands actions = new RobotCommands(robotManager);
  private final Autos autos = new Autos(swerve, localization, imu, actions);
  private final LightsSubsystem lightsSubsystem =
      new LightsSubsystem(
          new CANdle(RobotConfig.get().lights().deviceID(), RobotConfig.get().canivoreName()),
          robotManager,
          vision);

  public Robot() {
    // Log to a USB stick
    Logger.addDataReceiver(new WPILOGWriter());
    if (RobotConfig.IS_DEVELOPMENT) {
      // Publish data to NetworkTables
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("RoborioSerialNumber", RobotConfig.SERIAL_NUMBER);
    Logger.recordMetadata("RobotConfig", RobotConfig.get().robotName());
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    Logger.start();

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = autos.getAutoCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.driveTeleopCommand());

    driverController.back().onTrue(localization.getZeroCommand());

    driverController.y().onTrue(snaps.getCommand(() -> SnapManager.getSourceAngle()));
    driverController.x().onTrue(snaps.getCommand(() -> SnapManager.getStageLeftAngle()));
    driverController.a().onTrue(snaps.getCommand(() -> SnapManager.getDownAngle()));
    driverController.b().onTrue(snaps.getCommand(() -> SnapManager.getAmpAngle()));

    driverController
        .leftTrigger()
        .onTrue(actions.intakeFloorCommand())
        .onFalse(actions.stowDownAfterIntakeCommand());
    driverController
        .leftBumper()
        .onTrue(actions.sourceIntakeCommand())
        .onFalse(actions.stowUpAfterIntakeCommand());
    driverController
        .rightTrigger()
        .onTrue(actions.confirmShotCommand())
        .onFalse(actions.stowDownCommand());
    driverController
        .rightBumper()
        .onTrue(actions.outtakeCommand())
        .onFalse(actions.stowDownCommand());

    operatorController.povUp().onTrue(actions.getClimberCommand());

    operatorController
        .y()
        .onTrue(actions.waitSubwooferShotCommand())
        .onFalse(actions.stowDownCommand());
    operatorController.b().onTrue(actions.stowUpCommand());
    operatorController.a().onTrue(actions.stowDownCommand());
    operatorController
        .leftBumper()
        .onTrue(actions.waitForFloorShotCommand())
        .onFalse(actions.stowDownCommand());
    operatorController
        .rightBumper()
        .onTrue(actions.waitForAmpShotCommand())
        .onFalse(actions.stowDownCommand());
    operatorController
        .rightTrigger()
        .onTrue(actions.waitForSpeakerShotCommand())
        .onFalse(actions.stowDownCommand());
    operatorController.back().onTrue(actions.homeCommand());
  }
}
