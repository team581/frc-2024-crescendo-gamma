// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.amp_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class AmpAlignPaths {
  public static final AmpAlignPath RED_AMP =
      new AmpAlignPath(
          new Pose2d(14.66, 7.32, Rotation2d.fromDegrees(-90)), Rotation2d.fromDegrees(-90));
  public static final AmpAlignPath BLUE_AMP =
      new AmpAlignPath(
          new Pose2d(1.8, 7.19, Rotation2d.fromDegrees(-90)), Rotation2d.fromDegrees(-90));

  public static final List<AmpAlignPath> PATHS = List.of(RED_AMP, BLUE_AMP);
}
