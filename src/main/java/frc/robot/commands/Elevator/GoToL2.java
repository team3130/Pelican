// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.LEDs;

/** An example command that uses an example subsystem. */
public class GoToL2 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Manipulator manip;
  private final LEDs LED;
  private final AlgaeIntake algaeIntake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param elevator The subsystem used by this command.
   */
  public GoToL2(Elevator elevator, Manipulator manip, LEDs LED, AlgaeIntake algaeIntake) {
    this.elevator = elevator;
    this.LED = LED;
    this.manip = manip;
    this.algaeIntake = algaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(algaeIntake.getAlgaeMode()) {
      elevator.goToSetpoint(elevator.getL2()+2);
    } else {
      elevator.goToL2();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
