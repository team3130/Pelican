// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Manipulator;

/** An example command that uses an example subsystem. */
public class GoToL4 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Manipulator manip;
  private boolean runnable;

  /**
   * Creates a new ExampleCommand.
   *
   * @param elevator The subsystem used by this command.
   */
  public GoToL4(Elevator elevator, Manipulator manip) {
    this.elevator = elevator;
    this.manip = manip;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(elevator.isAtMinPosition()) {
      elevator.setRunnable(!manip.getFirstBeam() && !manip.getSecondBeam());
    } else {
      elevator.setRunnable(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevator.isRunnable()) {
      if (elevator.isZeroed()) {
        elevator.goToL4();
      } else {
        elevator.goToHome();
        elevator.goToL4();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.brokeTopLimitSwitch();
  }
}
