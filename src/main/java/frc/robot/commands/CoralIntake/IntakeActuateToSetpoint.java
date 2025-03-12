// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CoralIntake;

/** An example command that uses an example subsystem. */
public class IntakeActuateToSetpoint extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralIntake coralIntake;
  private final CommandXboxController operatorController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param coralIntake The subsystem used by this command.
   */
  public IntakeActuateToSetpoint(CoralIntake coralIntake, CommandXboxController operatorController) {
    this.coralIntake = coralIntake;
    this.operatorController = operatorController;
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
    double setpoint = -operatorController.getRightY();
    if(setpoint > 0) {
      coralIntake.gotoSetpoint1(setpoint);
      coralIntake.gotoSetpoint2(setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
