// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.snaps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.fms.FmsSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SnapManager extends LifecycleSubsystem {
  public static Rotation2d getSourceAngle() {
    return FmsSubsystem.isRedAlliance()
        ? Rotation2d.fromDegrees(60.0)
        : Rotation2d.fromDegrees(180.0 - 60.0);
  }

  public static Rotation2d getStageLeftAngle() {
    return FmsSubsystem.isRedAlliance()
        ? Rotation2d.fromDegrees(-60 )
        : Rotation2d.fromDegrees(-60+ 180);
  }

  public static Rotation2d getStageRightAngle() {
    return FmsSubsystem.isRedAlliance()
        ? Rotation2d.fromDegrees(60.0)
        : Rotation2d.fromDegrees(60.0 + 180.0);
  }

  public static Rotation2d getAmpAngle() {
    return FmsSubsystem.isRedAlliance()
        ? Rotation2d.fromDegrees(-90)
        : Rotation2d.fromDegrees(-90.0);
  }

  private final SwerveSubsystem swerve;
  private Rotation2d angle = new Rotation2d();
  private boolean enabled;
  private final CommandXboxController controller;

  public SnapManager(SwerveSubsystem swerve, CommandXboxController driverController) {
    super(SubsystemPriority.SNAPS);
    this.swerve = swerve;
    this.controller = driverController;

    new Trigger(() -> Math.abs(controller.getRightX()) > 0.075).onTrue(getDisableCommand());
  }

  public void setAngle(Rotation2d angle) {
    this.angle = angle;
  }

  public void setEnabled(boolean value) {
    this.enabled = value;

    if (!value) {
      swerve.disableSnapToAngle();
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("SnapManager/Enabled", enabled);
    Logger.recordOutput("SnapManager/GoalAngle", angle.getDegrees());
  }

  @Override
  public void enabledPeriodic() {
    if (enabled) {
      swerve.setSnapToAngle(angle);
    }
  }

  public Command getCommand(Supplier<Rotation2d> angle) {
    return run(() -> {
          setAngle(angle.get());
          setEnabled(true);
        })
        .withName("SnapCommand");
  }

  public Command getDisableCommand() {
    return runOnce(() -> setEnabled(false)).withName("SnapDisableCommand");
  }

  @Override
  public void teleopInit() {
    // Avoid sudden snaps as soon as you enable
    setEnabled(false);
  }

  public void cancelCurrentCommand() {
    Command command = getCurrentCommand();

    if (command != null) {
      command.cancel();
    }
  }
}
