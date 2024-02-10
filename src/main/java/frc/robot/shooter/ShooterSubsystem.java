// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterSubsystem extends LifecycleSubsystem {
  private static final int IDLE_RPM = 1000;
  private static final double SUBWOOFER_SHOOTING_RPM = 5000;
  private static final double AMP_SHOOTING_RPM = 1000;
  private static final double TOLERANCE_RPM = 100;
  private final TalonFX bottomMotor;
  private final TalonFX topMotor;
  private final LoggedDashboardNumber ntRPM = new LoggedDashboardNumber("Shooter/RPMOverride", -1);
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
  private double speakerDistance = 0;
  private double floorSpotDistance = 0;
  private double goalRPM = 0;
  double usedGoalRPM = 0;

  private static final InterpolatingDoubleTreeMap speakerDistanceToRPM =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap floorSpotDistanceToRPM =
      new InterpolatingDoubleTreeMap();

  private ShooterMode goalMode = ShooterMode.IDLE;
  private final StaticBrake brakeRequest = new StaticBrake();

  public ShooterSubsystem(TalonFX bottomMotor, TalonFX topMotor) {
    super(SubsystemPriority.SHOOTER);

    this.bottomMotor = bottomMotor;
    this.topMotor = topMotor;

    RobotConfig.get().shooter().speakerShotRpms().accept(speakerDistanceToRPM);
    RobotConfig.get().shooter().floorShotRpms().accept(floorSpotDistanceToRPM);

    bottomMotor.getConfigurator().apply(RobotConfig.get().shooter().bottomMotorConfig());
    topMotor.getConfigurator().apply(RobotConfig.get().shooter().topMotorConfig());
  }

  @Override
  public void enabledPeriodic() {
    double overrideRPM = ntRPM.get();

    Logger.recordOutput("Shooter/OverrideRPM", overrideRPM);
    Logger.recordOutput("Shooter/UsedRPM", usedGoalRPM);

    usedGoalRPM = overrideRPM == -1 ? goalRPM : overrideRPM;

    if (goalMode == ShooterMode.INTAKE) {
      bottomMotor.setControl(brakeRequest);
      topMotor.setControl(brakeRequest);
    } else {
      bottomMotor.setControl(velocityRequest.withVelocity(usedGoalRPM / 60));
      topMotor.setControl(velocityRequest.withVelocity(usedGoalRPM / 60));
    }
  }

  @Override
  public void robotPeriodic() {
    if (goalMode == ShooterMode.SPEAKER_SHOT) {
      goalRPM = speakerDistanceToRPM.get(speakerDistance);
    } else if (goalMode == ShooterMode.AMP_SHOT) {
      goalRPM = AMP_SHOOTING_RPM;
    } else if (goalMode == ShooterMode.SUBWOOFER_SHOT) {
      goalRPM = SUBWOOFER_SHOOTING_RPM;
    } else if (goalMode == ShooterMode.FLOOR_SHOT) {
      goalRPM = floorSpotDistanceToRPM.get(floorSpotDistance);
    } else {
      goalRPM = IDLE_RPM;
    }
    Logger.recordOutput("Shooter/DistanceToSpeaker", speakerDistance);
    Logger.recordOutput("Shooter/DistanceToFloorSpot", floorSpotDistance);
    Logger.recordOutput("Shooter/Mode", goalMode);
    Logger.recordOutput("Shooter/GoalRPM", goalRPM);
    Logger.recordOutput("Shooter/TopMotor/Temperature", topMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Shooter/TopMotor/RPM", getRPM(topMotor));
    Logger.recordOutput(
        "Shooter/TopMotor/POutput", topMotor.getClosedLoopProportionalOutput().getValue());
    Logger.recordOutput(
        "Shooter/TopMotor/SupplierCurrent", topMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/TopMotor/StatorCurrent", topMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shooter/TopMotor/Voltage", topMotor.getMotorVoltage().getValue());
    Logger.recordOutput(
        "Shooter/BottomMotor/StatorCurrent", bottomMotor.getStatorCurrent().getValue());
    Logger.recordOutput("Shooter/BottomMotor/Voltage", bottomMotor.getMotorVoltage().getValue());
    Logger.recordOutput("Shooter/BottomMotor/RPM", getRPM(bottomMotor));
    Logger.recordOutput("Shooter/BottomMotor/Temperature", bottomMotor.getDeviceTemp().getValue());
    Logger.recordOutput(
        "Shooter/BottomMotor/POutput", bottomMotor.getClosedLoopProportionalOutput().getValue());
    Logger.recordOutput(
        "Shooter/BottomMotor/SupplierCurrent", bottomMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/AtGoal", atGoal(goalMode));
  }

  public boolean atGoal(ShooterMode mode) {
    if (mode != goalMode) {
      return false;
    }

    if (goalMode == ShooterMode.IDLE) {
      return true;
    }

    if (Math.abs(goalRPM - getRPM(bottomMotor)) < TOLERANCE_RPM
        && Math.abs(goalRPM - getRPM(topMotor)) < TOLERANCE_RPM) {
      return true;
    }

    return false;
  }

  private double getRPM(TalonFX motor) {
    return motor.getVelocity().getValue() * 60.0;
  }

  public void setMode(ShooterMode newMode) {
    goalMode = newMode;
  }

  public void setSpeakerDistance(double distance) {
    speakerDistance = distance;
  }

  public void setFloorSpotDistance(double distance) {
    floorSpotDistance = distance;
  }
}
