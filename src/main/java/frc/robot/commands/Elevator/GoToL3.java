// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class GoToL3 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final LEDs LED;
  private final Manipulator manip;
  private final AlgaeIntake algaeIntake;
  private boolean runnable;

  /**
   * Creates a new ExampleCommand.
   *
   * @param elevator The subsystem used by this command.
   */
  public GoToL3(Elevator elevator, Manipulator manip, LEDs LED, AlgaeIntake algaeIntake) {
    this.elevator = elevator;
    this.manip = manip;
    this.LED = LED;
    this.algaeIntake = algaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!algaeIntake.getAlgaeMode()) {
      elevator.goToL3();
    } else {
      elevator.goToHome();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //LED.setLEDstateElevator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(algaeIntake.getAlgaeMode()) {
      elevator.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!algaeIntake.getAlgaeMode()) {
      return true;
    } else {
      return elevator.brokeBottomLimitSwitch() || elevator.brokeTopLimitSwitch();
    }
  }
}
