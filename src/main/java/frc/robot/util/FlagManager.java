// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import dev.doglog.DogLog;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

public class FlagManager<T extends Enum<T>> {
  private final String loggerCategory;

  private final Set<T> checked;

  public FlagManager(String loggerCategory, Class<T> flag) {
    this.loggerCategory = loggerCategory;
    this.checked = EnumSet.noneOf(flag);
  }

  public void log() {
    DogLog.log(
        loggerCategory + "/Flags",
        checked.stream().map(value -> value.toString()).toArray(String[]::new));
  }

  public void check(T flag) {
    checked.add(flag);
  }

  public List<T> getChecked() {
    return new ArrayList<>(checked);
  }

  public void clear() {
    checked.clear();
  }
}
