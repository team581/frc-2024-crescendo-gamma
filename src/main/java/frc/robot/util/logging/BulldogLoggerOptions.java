// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

// TODO: Add support for "extras" (ex. PDH currents)
public record BulldogLoggerOptions(boolean ntPublish, boolean captureNt, boolean captureDs) {
  public BulldogLoggerOptions() {
    // Default options
    this(false, false, true);
  }

  public BulldogLoggerOptions withNtPublish(boolean ntPublish) {
    return new BulldogLoggerOptions(ntPublish, captureNt(), captureDs());
  }

  public BulldogLoggerOptions withCaptureNt(boolean captureNt) {
    return new BulldogLoggerOptions(ntPublish(), captureNt, captureDs());
  }

  public BulldogLoggerOptions withCaptureDs(boolean captureDs) {
    return new BulldogLoggerOptions(ntPublish(), captureNt(), captureDs);
  }
}
