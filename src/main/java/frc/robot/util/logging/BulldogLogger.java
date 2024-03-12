// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

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
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.ProtobufLogEntry;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/** A logger based on WPILib's {@link DataLogManager} */
public class BulldogLogger {
  private static final String LOG_TABLE = "/Robot";
  private static final PubSubOption PUB_SUB_OPTIONS = PubSubOption.sendAll(true);

  private static BulldogLogger instance;

  public static synchronized BulldogLogger getInstance(DataLog log) {
    if (instance == null) {
      instance = new BulldogLogger(log, new BulldogLoggerOptions());
    }

    return instance;
  }

  public static synchronized BulldogLogger getInstance(DataLog log, BulldogLoggerOptions options) {
    if (instance == null) {
      instance = new BulldogLogger(log, options);
    }

    return instance;
  }

  public static synchronized BulldogLogger getInstance(BulldogLoggerOptions options) {
    if (instance == null) {
      instance = new BulldogLogger(options);
    }

    return instance;
  }

  /** Get an instance of the logger with the default options. */
  public static synchronized BulldogLogger getInstance() {
    return getInstance(new BulldogLoggerOptions());
  }

  private static String prefixKey(String key) {
    return LOG_TABLE + "/" + key;
  }

  private final Map<String, BooleanArrayLogEntry> booleanArrayLogs = new HashMap<>();
  private final Map<String, BooleanLogEntry> booleanLogs = new HashMap<>();
  private final Map<String, DoubleArrayLogEntry> doubleArrayLogs = new HashMap<>();
  private final Map<String, DoubleLogEntry> doubleLogs = new HashMap<>();
  private final Map<String, FloatArrayLogEntry> floatArrayLogs = new HashMap<>();
  private final Map<String, FloatLogEntry> floatLogs = new HashMap<>();
  private final Map<String, IntegerArrayLogEntry> integerArrayLogs = new HashMap<>();
  private final Map<String, IntegerLogEntry> integerLogs = new HashMap<>();
  private final Map<String, ProtobufLogEntry<?>> protobufLogs = new HashMap<>();
  private final Map<String, RawLogEntry> rawLogs = new HashMap<>();
  private final Map<String, StringArrayLogEntry> stringArrayLogs = new HashMap<>();
  private final Map<String, StringLogEntry> stringLogs = new HashMap<>();
  private final Map<String, StructArrayLogEntry<?>> structArrayLogs = new HashMap<>();
  private final Map<String, StructLogEntry<?>> structLogs = new HashMap<>();

  private final Map<String, BooleanArrayPublisher> booleanArrayPublishers = new HashMap<>();
  private final Map<String, BooleanPublisher> booleanPublishers = new HashMap<>();
  private final Map<String, DoubleArrayPublisher> doubleArrayPublishers = new HashMap<>();
  private final Map<String, DoublePublisher> doublePublishers = new HashMap<>();
  private final Map<String, FloatArrayPublisher> floatArrayPublishers = new HashMap<>();
  private final Map<String, FloatPublisher> floatPublishers = new HashMap<>();
  private final Map<String, IntegerArrayPublisher> integerArrayPublishers = new HashMap<>();
  private final Map<String, IntegerPublisher> integerPublishers = new HashMap<>();
  private final Map<String, ProtobufPublisher<?>> protobufPublishers = new HashMap<>();
  private final Map<String, RawPublisher> rawPublishers = new HashMap<>();
  private final Map<String, StringArrayPublisher> stringArrayPublishers = new HashMap<>();
  private final Map<String, StringPublisher> stringPublishers = new HashMap<>();
  private final Map<String, StructArrayPublisher<?>> structArrayPublishers = new HashMap<>();
  private final Map<String, StructPublisher<?>> structPublishers = new HashMap<>();

  private final DataLog log;
  private final BulldogLoggerOptions options;

  private final Map<String, Struct<?>> structTypeCache = new HashMap<>();

  private NetworkTable logTable;

  @SuppressWarnings("resource")
  public void log(String key, boolean[] value) {
    booleanArrayLogs
        .computeIfAbsent(key, k -> new BooleanArrayLogEntry(log, prefixKey(k)))
        .append(value);
    if (options.ntPublish()) {
      booleanArrayPublishers
          .computeIfAbsent(key, k -> logTable.getBooleanArrayTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public void log(String key, boolean value) {
    booleanLogs.computeIfAbsent(key, k -> new BooleanLogEntry(log, prefixKey(k))).append(value);
    if (options.ntPublish()) {
      booleanPublishers
          .computeIfAbsent(key, k -> logTable.getBooleanTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public void log(String key, double[] value) {
    doubleArrayLogs
        .computeIfAbsent(key, k -> new DoubleArrayLogEntry(log, prefixKey(k)))
        .append(value);
    if (options.ntPublish()) {
      doubleArrayPublishers
          .computeIfAbsent(key, k -> logTable.getDoubleArrayTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public void log(String key, double value) {
    doubleLogs.computeIfAbsent(key, k -> new DoubleLogEntry(log, prefixKey(k))).append(value);
    if (options.ntPublish()) {
      doublePublishers
          .computeIfAbsent(key, k -> logTable.getDoubleTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public void log(String key, float[] value) {
    floatArrayLogs
        .computeIfAbsent(key, k -> new FloatArrayLogEntry(log, prefixKey(k)))
        .append(value);
    if (options.ntPublish()) {
      floatArrayPublishers
          .computeIfAbsent(key, k -> logTable.getFloatArrayTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public void log(String key, float value) {
    floatLogs.computeIfAbsent(key, k -> new FloatLogEntry(log, prefixKey(k))).append(value);
    if (options.ntPublish()) {
      floatPublishers
          .computeIfAbsent(key, k -> logTable.getFloatTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  public void log(String key, int[] value) {
    log(key, value);
  }

  @SuppressWarnings("resource")
  public void log(String key, long[] value) {
    integerArrayLogs
        .computeIfAbsent(key, k -> new IntegerArrayLogEntry(log, prefixKey(k)))
        .append(value);
    if (options.ntPublish()) {
      integerArrayPublishers
          .computeIfAbsent(key, k -> logTable.getIntegerArrayTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  @SuppressWarnings("resource")
  public void log(String key, int value) {
    integerLogs.computeIfAbsent(key, k -> new IntegerLogEntry(log, prefixKey(k))).append(value);
    if (options.ntPublish()) {
      integerPublishers
          .computeIfAbsent(key, k -> logTable.getIntegerTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  // TODO: Protobuf logs

  // TODO: Raw logs

  @SuppressWarnings("resource")
  public void log(String key, String[] value) {
    if (value == null) {
      return;
    }
    stringArrayLogs
        .computeIfAbsent(key, k -> new StringArrayLogEntry(log, prefixKey(k)))
        .append(value);
    if (options.ntPublish()) {
      stringArrayPublishers
          .computeIfAbsent(key, k -> logTable.getStringArrayTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  public void log(String key, Enum<?>[] value) {
    if (value == null) {
      return;
    }
    // Convert enum array to string array
    var stringArray = new String[value.length];

    for (int i = 0; i < value.length; i++) {
      stringArray[i] = value[i].name();
    }

    log(key, stringArray);
  }

  @SuppressWarnings("resource")
  public void log(String key, String value) {
    if (value == null) {
      return;
    }
    stringLogs.computeIfAbsent(key, k -> new StringLogEntry(log, prefixKey(k))).append(value);
    if (options.ntPublish()) {
      stringPublishers
          .computeIfAbsent(key, k -> logTable.getStringTopic(k).publish(PUB_SUB_OPTIONS))
          .set(value);
    }
  }

  public void log(String key, Enum<?> value) {
    if (value == null) {
      return;
    }
    log(key, value.name());
  }

  @SuppressWarnings("resource")
  public <T> void log(String key, Struct<T> struct, T[] value) {
    if (struct == null || value == null) {
      return;
    }
    @SuppressWarnings("unchecked")
    var entry =
        (StructArrayLogEntry<T>)
            structArrayLogs.computeIfAbsent(
                key, k -> StructArrayLogEntry.create(log, prefixKey(k), struct));

    entry.append(value);
    if (options.ntPublish()) {
      @SuppressWarnings("unchecked")
      var publisher =
          (StructArrayPublisher<T>)
              structArrayPublishers.computeIfAbsent(
                  key, k -> logTable.getStructArrayTopic(k, struct).publish(PUB_SUB_OPTIONS));
      publisher.set(value);
    }
  }

  public <T extends StructSerializable> void log(String key, T[] value) {
    if (value == null) {
      return;
    }
    var maybeStruct = findStructType(value.getClass());

    if (maybeStruct.isPresent()) {
      @SuppressWarnings("unchecked")
      var struct = (Struct<T>) maybeStruct.get();
      log(key, struct, value);
    }
  }

  @SuppressWarnings("resource")
  public <T> void log(String key, Struct<T> struct, T value) {
    if (struct == null || value == null) {
      return;
    }
    @SuppressWarnings("unchecked")
    var entry =
        (StructLogEntry<T>)
            structLogs.computeIfAbsent(key, k -> StructLogEntry.create(log, prefixKey(k), struct));

    entry.append(value);
    if (options.ntPublish()) {
      @SuppressWarnings("unchecked")
      var publisher =
          (StructPublisher<T>)
              structPublishers.computeIfAbsent(
                  key, k -> logTable.getStructTopic(k, struct).publish(PUB_SUB_OPTIONS));
      publisher.set(value);
    }
  }

  public <T extends StructSerializable> void log(String key, T value) {
    if (value == null) {
      return;
    }
    var maybeStruct = findStructType(value.getClass());

    if (maybeStruct.isPresent()) {
      @SuppressWarnings("unchecked")
      var struct = (Struct<T>) maybeStruct.get();
      log(key, struct, value);
    }
  }

  private BulldogLogger(BulldogLoggerOptions options) {
    this(DataLogManager.getLog(), options);
  }

  private BulldogLogger(DataLog log, BulldogLoggerOptions options) {
    this.log = log;
    this.options = options;

    if (options.ntPublish()) {
      logTable = NetworkTableInstance.getDefault().getTable(LOG_TABLE);
    }

    DataLogManager.logNetworkTables(options.captureNt());

    if (options.captureDs()) {
      DriverStation.startDataLog(log);
    }
  }

  private Optional<Struct<?>> findStructType(Class<?> classObj) {
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
}
