// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class RunAlgaeIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake algaeIntake;
  private final CommandPS5Controller controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param algaeIntake The subsystem used by this command.
   */
  public RunAlgaeIntake(AlgaeIntake algaeIntake, CommandPS5Controller controller) {
    this.algaeIntake = algaeIntake;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {algaeIntake.runIntake();}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !controller.L2().getAsBoolean();
  }
}
