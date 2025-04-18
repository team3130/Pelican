// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;

/** An example command that uses an example subsystem. */
public class GoToHome extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final LEDs LEDs;

  /**
   * Creates a new ExampleCommand.
   *
   * @param elevator The subsystem used by this command.
   */
  public GoToHome(Elevator elevator, LEDs LEDs) {
    this.elevator = elevator;
    this.LEDs = LEDs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.goToHome();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //LEDs.setLEDstateElevator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.brokeBottomLimitSwitch() || elevator.brokeTopLimitSwitch();
  }
}
