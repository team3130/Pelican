// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorAutoHome extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorAutoHome(Elevator elevator) {
    elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setAtL1(false);
    elevator.setAtL2(false);
    elevator.setAtL3(false);
    elevator.setAtL4(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.moveElevatorAtSpeed(elevator.smartLift(elevator.getHomeSetpoint()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
    if (elevator.isAtSetpointWithDeadband()){
      elevator.setAtHome(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!elevator.getHasZeroed() || elevator.isAtSetpoint());
  }
}
