// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AutoChooser {
  private final SendableChooser<AutoSelection> chooser = new SendableChooser<>();
  private final Set<String> brokenAutoNames = new HashSet<>();
  private Optional<Command> cachedCommand = Optional.empty();
  private String cachedAutoName = "";

  public AutoChooser() {
    SmartDashboard.putData("Autos/SelectedAuto", chooser);
    chooser.setDefaultOption(AutoSelection.DO_NOTHING.toString(), AutoSelection.DO_NOTHING);

    for (AutoSelection selection : EnumSet.allOf(AutoSelection.class)) {
      chooser.addOption(selection.toString(), selection);
    }
  }

  public Command getAutoCommand() {
    var selection = getAutoSelection();

    String autoName = FmsSubsystem.isRedAlliance() ? selection.redAutoName : selection.blueAutoName;


    // When the name of the auto changes (either from changing alliance color or from making a new selection), clear out the cached command
    if (autoName.equals(cachedAutoName)) {
      cachedCommand = Optional.empty();
    }

    cachedAutoName = autoName;

    Logger.recordOutput(
        "Autos/BrokenAutoNames", brokenAutoNames.toArray(new String[brokenAutoNames.size()]));
    Logger.recordOutput("Autos/SelectedAutoName", selection);
    Logger.recordOutput("Autos/SelectedAutoFileName", autoName.equals("") ? "(none)" : autoName);

    if (cachedCommand.isPresent()) {
      return cachedCommand.get();
    }

    if (autoName.equals("")) {
      var command = Commands.print("No auto path provided, doing nothing");
      cachedCommand = Optional.of(command);
      return command;
    }

    try {
      var command =
          AutoBuilder.buildAuto(autoName)
              .withName("Auto__" + selection.toString() + "__" + autoName);

      cachedCommand = Optional.of(command);
      return command;
    } catch (Exception e) {
      var alliance = FmsSubsystem.isRedAlliance() ? "red" : "blue";
      brokenAutoNames.add(selection.toString() + " - " + alliance + ": " + autoName);
    }

    var command = Commands.print("Auto failed to load, check Autos/BrokenAutoNames");
    cachedCommand = Optional.of(command);
    return command;
  }

  private AutoSelection getAutoSelection() {
    var selected = Optional.ofNullable(chooser.getSelected());

    return selected.orElse(AutoSelection.DO_NOTHING);
  }
}
