// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package dev.doglog;

import dev.doglog.loggers.DataLogLogger;
import dev.doglog.loggers.DogLogLogger;
import dev.doglog.loggers.NetworkTablesLogger;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/** A logger based on WPILib's {@link DataLogManager} */
// TODO: Add static methods for logging, probably want to move a bunch of this logic to a new Logger
// class
public class DogLog extends DogLogLogger {
  private static final String LOG_TABLE = "/Robot";

  private static DogLog instance;

  public static synchronized DogLog getInstance(DataLog log) {
    if (instance == null) {
      instance = new DogLog(log, new DogLogOptions());
    }

    return instance;
  }

  public static synchronized DogLog getInstance(DataLog log, DogLogOptions options) {
    if (instance == null) {
      instance = new DogLog(log, options);
    }

    return instance;
  }

  public static synchronized DogLog getInstance(DogLogOptions options) {
    if (instance == null) {
      instance = new DogLog(options);
    }

    return instance;
  }

  /** Get an instance of the logger with the default options. */
  public static synchronized DogLog getInstance() {
    return getInstance(new DogLogOptions());
  }

  private DogLog(DogLogOptions options) {
    this(DataLogManager.getLog(), options);
  }

  private DogLog(DataLog log, DogLogOptions options) {
    super(
        new DataLogLogger(log, LOG_TABLE),
        options.ntPublish() ? new NetworkTablesLogger(LOG_TABLE) : null);

    DataLogManager.logNetworkTables(options.captureNt());

    if (options.captureDs()) {
      DriverStation.startDataLog(log);
    }
  }
}
