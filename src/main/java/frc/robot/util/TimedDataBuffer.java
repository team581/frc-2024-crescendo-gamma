// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

public class TimedDataBuffer {
  ArrayList<TimeData> dataSet = new ArrayList<TimeData>();
  private int size = 0;

  public TimedDataBuffer(int size) {
    this.size = size;
  }

  public void addData(double time, double data) {
    if (dataSet.size() > size) {
      dataSet.remove(0);
    }
    dataSet.add(new TimeData(time, data));
  }

  public double lookupData(double time) {
    if (time <= dataSet.get(0).time()) {
      return dataSet.get(0).data();
    }
    if (time >= dataSet.get(dataSet.size() - 1).time()) {
      return dataSet.get(dataSet.size() - 1).data();
    }

    for (int i = 0; i < dataSet.size() - 1; i++) {
      double x1 = dataSet.get(i).time();
      double x2 = dataSet.get(i + 1).time();
      double y1 = dataSet.get(i).data();
      double y2 = dataSet.get(i + 1).data();

      if (time >= x1 && time <= x1) {
        double slope = (y2 - y1) / (x2 - x1);
        return (slope * time) + (y1 - (slope * x1));
      }
    }
    return dataSet.get(dataSet.size() - 1).data();
  }
}
