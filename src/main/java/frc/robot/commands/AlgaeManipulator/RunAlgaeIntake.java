// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class RunAlgaeIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeManipulator algaeManipulator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param algaeManipulator The subsystem used by this command.
   */
  public RunAlgaeIntake(AlgaeManipulator algaeManipulator) {
    this.algaeManipulator = algaeManipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeManipulator.runIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeManipulator.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
