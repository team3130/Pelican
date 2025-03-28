// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/** An example command that uses an example subsystem. */
public class ToggleAlgaeActuation extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake algaeIntake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param algaeIntake The subsystem used by this command.
   */
  public ToggleAlgaeActuation(AlgaeIntake algaeIntake) {
    this.algaeIntake = algaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (algaeIntake.getActuationPosition() == 0) {
      algaeIntake.actuateIntake();
    } else {
      algaeIntake.deactuateIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.stopActuation();
    algaeIntake.setActuated(!algaeIntake.getActuated());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(algaeIntake.getActuationPosition() == 0) {
      return algaeIntake.getSwitch();
    } else {
      return algaeIntake.getActuationPosition() >= algaeIntake.getSetpoint();
    }
  }
}
