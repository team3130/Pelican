// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;

/** An example command that uses an example subsystem. */
public class GoToL2Basic extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final LEDs LED;

  /**
   * Creates a new ExampleCommand.
   *
   * @param elevator The subsystem used by this command.
   */
  public GoToL2Basic(Elevator elevator, LEDs LED) {
    this.elevator = elevator;
    this.LED = LED;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevator.getPosition() > elevator.getL2()) {
      elevator.goDown();
    }else if(elevator.getPosition() < elevator.getL2()) {
      elevator.goUp();
    }
    //LED.setLEDstateElevator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return elevator.getPosition() < elevator.getL2() + 1 && elevator.getPosition() > elevator.getL2() - 1;
  }
}
