// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),

  // FOUR_PIECE_6_4("Red 4 Piece 6-4", "Blue 4 Piece 6-4"),
  // FIVE_PIECE_2_5("Red 5 Piece 2-5", "Blue 5 Piece 2-5"),
  // FIVE_PIECE_3_6("Red 5 Piece 3-6", "Blue 5 Piece 3-6"),
  FOUR_PIECE_SOURCE_8_6("Red 4 Piece Source 8-6", "Blue 4 Piece Source 8-6"),
  // FIVE_PIECE_RVL_8_5("Red 5 Piece RVL 8-5", "Blue 5 Piece RVL 8-6"),
  // FIVE_PIECE_AMP_4_2("Red 5 Piece 4-2", "Blue 5 Piece 4-2"),
  SIX_PIECE_SOURCE_1_5("Red 6 Piece Source 1-5", "Blue 6 Piece Source 1-5");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}
