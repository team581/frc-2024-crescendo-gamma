// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging.advantagekit;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.logging.BulldogLogger;

/**
 * A wrapper around {@link BulldogLogger} that makes it easy to use as a drop-in replacement for
 * AdvantageKit.
 */
public class Logger {
  private static BulldogLogger baseLogger;

  public static void setBaseLogger(BulldogLogger newLogger) {
    baseLogger = newLogger;
  }

  public static void recordOutput(String key, boolean[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, boolean value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, double[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, double value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, float[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, float value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, int[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, int value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, long[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, long value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, String[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, String value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, Enum<?>[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, Enum<?> value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, StructSerializable[] value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordOutput(String key, StructSerializable value) {
    if (baseLogger != null) {
      baseLogger.log(key, value);
    }
  }

  public static void recordMetadata(String key, String value) {
    if (baseLogger != null) {
      baseLogger.log("Metadata/" + key, value);
    }
  }

  public static double getRealTimestamp() {
    return HALUtil.getFPGATime();
  }

  private Logger() {}
}
