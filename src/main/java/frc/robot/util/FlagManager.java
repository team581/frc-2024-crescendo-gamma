// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.util.logging.advantagekit.Logger;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

public class FlagManager<T extends Enum<T>> {
  private final String loggerCategory;
  private final EnumSet<T> allMembers;

  private final Set<T> checked;

  public FlagManager(String loggerCategory, Class<T> flag) {
    this.loggerCategory = loggerCategory;
    this.allMembers = EnumSet.allOf(flag);
    this.checked = EnumSet.noneOf(flag);
  }

  public void log() {
    for (T flag : allMembers) {
      Logger.recordOutput(loggerCategory + "/Flags/" + flag.toString(), checked.contains(flag));
    }
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
