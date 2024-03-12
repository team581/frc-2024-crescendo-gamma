// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.junction;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * A logger that is almost fully compatible with the AdvantageKit `Logger` class. It uses WPILib's
 * DataLog to log the data to a file, and streams the data using NetworkTables 4.
 */
public class Logger {
  private static final String OUTPUT_LOG_ENTRY_PREFIX = "/RealOutputs/";
  private static final String METADATA_LOG_ENTRY_PREFIX = "/RealMetadata/";

  private static final Map<String, BooleanLogEntry> booleanLogs = new HashMap<>();
  private static final Map<String, DoubleLogEntry> doubleLogs = new HashMap<>();
  private static final Map<String, FloatLogEntry> floatLogs = new HashMap<>();
  private static final Map<String, IntegerLogEntry> integerLogs = new HashMap<>();
  private static final Map<String, StringLogEntry> stringLogs = new HashMap<>();
  private static final Map<String, BooleanArrayLogEntry> booleanArrayLogs = new HashMap<>();
  private static final Map<String, DoubleArrayLogEntry> doubleArrayLogs = new HashMap<>();
  private static final Map<String, FloatArrayLogEntry> floatArrayLogs = new HashMap<>();
  private static final Map<String, IntegerArrayLogEntry> integerArrayLogs = new HashMap<>();
  private static final Map<String, StringArrayLogEntry> stringArrayLogs = new HashMap<>();
  private static final Map<String, RawLogEntry> rawLogs = new HashMap<>();

  private static final Map<String, BooleanPublisher> booleanPublishers = new HashMap<>();
  private static final Map<String, DoublePublisher> doublePublishers = new HashMap<>();
  private static final Map<String, FloatPublisher> floatPublishers = new HashMap<>();
  private static final Map<String, IntegerPublisher> integerPublishers = new HashMap<>();
  private static final Map<String, StringPublisher> stringPublishers = new HashMap<>();
  private static final Map<String, BooleanArrayPublisher> booleanArrayPublishers = new HashMap<>();
  private static final Map<String, DoubleArrayPublisher> doubleArrayPublishers = new HashMap<>();
  private static final Map<String, FloatArrayPublisher> floatArrayPublishers = new HashMap<>();
  private static final Map<String, IntegerArrayPublisher> integerArrayPublishers = new HashMap<>();
  private static final Map<String, StringArrayPublisher> stringArrayPublishers = new HashMap<>();
  private static final Map<String, RawPublisher> rawPublishers = new HashMap<>();

  private static final Map<String, Struct<?>> structTypeCache = new HashMap<>();

  private static final DataLog log = DataLogManager.getLog();

  private static boolean networkTables = false;
  private static NetworkTable outputTable;
  private static NetworkTable metadataTable;
  private static Map<String, String> allMetadata = new HashMap<>();

  /** Start the logger without streaming to NetworkTables. */
  public static void start() {
    start(false);
  }

  /**
   * Start the logger.
   *
   * @param shouldUseNetworkTables Whether data should be captured & exported from NetworkTables
   */
  public static void start(boolean shouldUseNetworkTables) {
    networkTables = shouldUseNetworkTables;
    if (RobotBase.isSimulation()) {
      // Log to project directory in simulation
      DataLogManager.start();
    } else {
      DataLogManager.start("/U/logs");
    }
    DataLogManager.logNetworkTables(networkTables);
    DriverStation.startDataLog(log);

    if (networkTables) {
      var table = NetworkTableInstance.getDefault().getTable("/AdvantageKit");
      outputTable = table.getSubTable("RealOutputs");
      metadataTable = table.getSubTable("RealMetadata");
    }

    for (Map.Entry<String, String> metadata : allMetadata.entrySet()) {
      String key = metadata.getKey();
      String value = metadata.getValue();

      StringLogEntry entry = new StringLogEntry(log, METADATA_LOG_ENTRY_PREFIX + key);
      entry.append(value);
      entry.finish();

      if (networkTables) {
        StringPublisher publisher = metadataTable.getStringTopic(key).publish();
        publisher.set(value);
      }
    }

    allMetadata = null;
    metadataTable = null;
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, boolean value) {
    booleanLogs
        .computeIfAbsent(key, k -> new BooleanLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(value);
    if (networkTables) {
      booleanPublishers
          .computeIfAbsent(key, k -> outputTable.getBooleanTopic(key).publish())
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, double value) {
    doubleLogs
        .computeIfAbsent(key, k -> new DoubleLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(value);
    if (networkTables) {
      doublePublishers
          .computeIfAbsent(key, k -> outputTable.getDoubleTopic(key).publish())
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, float value) {
    floatLogs
        .computeIfAbsent(key, k -> new FloatLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(value);
    if (networkTables) {
      floatPublishers
          .computeIfAbsent(key, k -> outputTable.getFloatTopic(key).publish())
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, int value) {
    integerLogs
        .computeIfAbsent(key, k -> new IntegerLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(value);
    if (networkTables) {
      integerPublishers
          .computeIfAbsent(key, k -> outputTable.getIntegerTopic(key).publish())
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, String value) {
    stringLogs
        .computeIfAbsent(key, k -> new StringLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(value);
    if (networkTables) {
      stringPublishers
          .computeIfAbsent(key, k -> outputTable.getStringTopic(key).publish())
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, boolean[] values) {
    booleanArrayLogs
        .computeIfAbsent(key, k -> new BooleanArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(values);
    if (networkTables) {
      booleanArrayPublishers
          .computeIfAbsent(key, k -> outputTable.getBooleanArrayTopic(key).publish())
          .set(values);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, double[] values) {
    doubleArrayLogs
        .computeIfAbsent(key, k -> new DoubleArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(values);
    if (networkTables) {
      doubleArrayPublishers
          .computeIfAbsent(key, k -> outputTable.getDoubleArrayTopic(key).publish())
          .set(values);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, float[] values) {
    floatArrayLogs
        .computeIfAbsent(key, k -> new FloatArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(values);
    if (networkTables) {
      floatArrayPublishers
          .computeIfAbsent(key, k -> outputTable.getFloatArrayTopic(key).publish())
          .set(values);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, long[] values) {
    integerArrayLogs
        .computeIfAbsent(key, k -> new IntegerArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(values);
    if (networkTables) {
      integerArrayPublishers
          .computeIfAbsent(key, k -> outputTable.getIntegerArrayTopic(key).publish())
          .set(values);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, int[] values) {
    long[] longs = Arrays.stream(values).asLongStream().toArray();
    integerArrayLogs
        .computeIfAbsent(key, k -> new IntegerArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(longs);
    if (networkTables) {
      integerArrayPublishers
          .computeIfAbsent(key, k -> outputTable.getIntegerArrayTopic(key).publish())
          .set(longs);
    }
  }

  @SuppressWarnings("resource")
  public static void recordOutput(String key, String[] value) {
    stringArrayLogs
        .computeIfAbsent(key, k -> new StringArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
        .append(value);
    if (networkTables) {
      stringArrayPublishers
          .computeIfAbsent(key, k -> outputTable.getStringArrayTopic(key).publish())
          .set(value);
    }
  }

  public static void recordOutput(String key, Enum<?> value) {
    recordOutput(key, value.name());
  }

  @SuppressWarnings({"unchecked", "resource"})
  public static <T extends WPISerializable> void recordOutput(String key, T value) {
    var maybeStruct = findStructType(value.getClass());

    if (maybeStruct.isPresent()) {
      Struct<T> struct = (Struct<T>) maybeStruct.get();
      ByteBuffer bb = ByteBuffer.allocate(struct.getSize());
      struct.pack(bb, value);
      rawLogs
          .computeIfAbsent(key, k -> new RawLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(bb.array());
      if (networkTables) {
        rawPublishers
            .computeIfAbsent(key, k -> outputTable.getRawTopic(key).publish(struct.getTypeString()))
            .set(bb.array());
      }
    }
  }

  @SuppressWarnings({"unchecked", "resource"})
  public static <T extends WPISerializable> void recordOutput(String key, T... value) {
    var maybeStruct = findStructType(value.getClass().getComponentType());

    if (maybeStruct.isPresent()) {
      Struct<T> struct = (Struct<T>) maybeStruct.get();
      ByteBuffer bb = ByteBuffer.allocate(struct.getSize() * value.length);
      for (T v : value) {
        struct.pack(bb, v);
      }
      rawLogs
          .computeIfAbsent(key, k -> new RawLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(bb.array());
      if (networkTables) {
        rawPublishers
            .computeIfAbsent(key, k -> outputTable.getRawTopic(key).publish(struct.getTypeString()))
            .set(bb.array());
      }
    }
  }

  public static void recordMetadata(String key, String value) {
    allMetadata.put(key, value == null ? "" : value);
  }

  public static double getRealTimestamp() {
    // Seconds to microseconds
    return Timer.getFPGATimestamp() * 1e6;
  }

  private static Optional<Struct<?>> findStructType(Class<?> classObj) {
    if (!structTypeCache.containsKey(classObj.getName())) {
      structTypeCache.put(classObj.getName(), null);
      Field field = null;
      try {
        field = classObj.getDeclaredField("struct");
      } catch (NoSuchFieldException | SecurityException e) {
      }
      if (field != null) {
        try {
          structTypeCache.put(classObj.getName(), (Struct<?>) field.get(null));
        } catch (IllegalArgumentException | IllegalAccessException e) {
        }
      }
    }

    Optional<Struct<?>> struct = Optional.ofNullable(structTypeCache.get(classObj.getName()));

    if (struct.isPresent()) {
      log.addSchema(struct.get());
    }

    return struct;
  }

  private Logger() {}
}
