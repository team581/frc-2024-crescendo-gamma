// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;

// Target angle should be the angle that the robot needs to be to be looking directly at the target
public record DistanceAngle(double distance, Rotation2d targetAngle, boolean seesSpeakerTag) {}
