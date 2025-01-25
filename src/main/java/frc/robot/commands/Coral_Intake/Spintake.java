// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral_Intake;

import frc.robot.subsystems.Coral_Intake;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Spintake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Coral_Intake subsystem;


  public Spintake(Coral_Intake subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setSpeed(0); //TODO
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
