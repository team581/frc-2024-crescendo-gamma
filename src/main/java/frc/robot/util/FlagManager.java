// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class FlagManager<T extends Enum<T>> {
  private final Class<T> flag;
  private final EnumSet<T> allMembers;

  private final Set<T> checked;

  public FlagManager(Class<T> flag) {
    this.flag = flag;
    this.allMembers = EnumSet.allOf(flag);
    this.checked = EnumSet.noneOf(flag);
  }

  public void log() {
    for (T flag : allMembers) {
      Logger.recordOutput("RobotManager/Flags/" + flag.toString(), checked.contains(flag));
    }
  }

  public void check(T flag) {
    checked.add(flag);
  }

  public List<T> getChecked() {
    return new ArrayList<T>(checked);
  }

  public void clear() {
    checked.clear();
  }
}