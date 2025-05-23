// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Manipulator;

/** An example command that uses an example subsystem. */
public class LimitedCoralIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake coralIntake;
  private final Manipulator manip;

  /**
   * Creates a new ExampleCommand.
   *
   * @param coralIntake The subsystem used by this command.
   */
  public LimitedCoralIntake(CoralIntake coralIntake, Manipulator manip) {
    this.coralIntake = coralIntake;
    this.manip = manip;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {coralIntake.runIntake();}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manip.getFirstBeam();
  }
}
