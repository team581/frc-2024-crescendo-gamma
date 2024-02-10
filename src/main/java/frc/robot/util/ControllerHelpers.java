// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

public class ControllerHelpers {
  public static double getDeadbanded(double joystickVal, double threshold) {
    double newJoystickVal = joystickVal;
    if (Math.abs(joystickVal) < threshold) {
      newJoystickVal = 0;
    } else {
      newJoystickVal =
          getSign(joystickVal) * (1 / (1 - threshold)) * (Math.abs(joystickVal) - threshold);
    }
    return newJoystickVal;
  }

  public static double getSign(double num) {
    return num >= 0 ? 1 : -1;
  }

  public static double getExponent(double joystickVal, double exponent) {
    return getSign(joystickVal) * Math.abs(Math.pow(Math.abs(joystickVal), exponent));
  }
}
