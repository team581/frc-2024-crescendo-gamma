// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

// TODO: Add support for "extras" (ex. PDH currents)
public record BulldogLogOptions(boolean ntPublish, boolean captureNt, boolean captureDs) {
  public BulldogLogOptions() {
    // Default options
    this(false, false, true);
  }

  public BulldogLogOptions withNtPublish(boolean ntPublish) {
    return new BulldogLogOptions(ntPublish, captureNt(), captureDs());
  }

  public BulldogLogOptions withCaptureNt(boolean captureNt) {
    return new BulldogLogOptions(ntPublish(), captureNt, captureDs());
  }

  public BulldogLogOptions withCaptureDs(boolean captureDs) {
    return new BulldogLogOptions(ntPublish(), captureNt(), captureDs);
  }
}
