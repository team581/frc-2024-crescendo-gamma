// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record PoseWithTagID(int fiducialID, Pose3d robotPose, Pose3d cameraPoseTargetSpace) {}
