// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

/** An example command that uses an example subsystem. */
public class IntakeDeactuate extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake coralIntake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param coralIntake The subsystem used by this command.
   */
  public IntakeDeactuate(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralIntake.deactuate1();
    coralIntake.deactuate2();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralIntake.getIsFinished();
  }
}
