package frc.robot.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig;

public class ElevatorPositions {
  public static final double STOWED_UP = 0.0;
  public static final double STOWED_DOWN = 0.0;

  public static final Rotation2d GROUND_INTAKING = RobotConfig.get().elevator().minAngle();
  public static final double SOURCE_INTAKING = 0.0;

  public static final double OUTTAKING = 0.0;

  public static final double FLOOR_SHOT = 0.0;

  public static final double SUBWOOFER_SHOT = 0.0;
  public static final double AMP_SHOT = 0.0;

  public static final double WAITING_CLIMBER_RAISED = 0.0;
  public static final double TRAP_SHOT = 0.0;

  private ElevatorPositions() {}
}
