// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),

  ONE_PIECE("Red 1 Piece", ""),
  TWO_PIECE_2("Red 2 Piece 2", ""),
  THREE_PIECE_1_2("Red 3 Piece 1-2", ""),
  FOUR_PIECE_6_4("Red 4 Piece 6-4", ""),
  FIVE_PIECE_1_4("Red 5 Piece 1-4", ""),
  FIVE_PIECE_2_5("Red 5 Piece 2-5", ""),
  FIVE_PIECE_3_6("Red 5 Piece 3-6", ""),
  SIX_PIECE_1_5("Red 6 Piece 1-5", ""),
  SIX_PIECE_1_5_TEST("Red 6 Piece 1-5 Test", "");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}
