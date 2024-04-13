// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),

  FOUR_PIECE_SOURCE_8_6_DYN("Red 4 Piece Source 8-6", "Blue 4 Piece Source 8-6"),
  FOUR_PIECE_SOURCE_7_8_DYN("Red 4 Piece Source 7,6,8", "Blue 4 Piece Source 7,6,8"),
  SNAKE_FOUR_DYN("Red Right Snake, 4", ""),
  SNAKE_FIVE_DYN("Red Right Snake, 5", ""),
  SIX_PIECE_2_5_DYN("Red 6 Piece 2,1,3-5", "Blue 6 Piece 2,1,3-5"),
  SIX_PIECE_2_6_4("Red 6 Piece 6,4", "Blue 6 Piece 6,4"),
  SIX_PIECE_2_6_DYN("Red 6 Piece 2,1,3,5,6", "Blue 6 Piece 2,1,3,5,6");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}
