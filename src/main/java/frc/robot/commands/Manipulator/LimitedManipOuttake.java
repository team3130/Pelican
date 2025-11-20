// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

import java.util.Objects;

/** An example command that uses an example subsystem. */
public class LimitedManipOuttake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator manip;
  private final Elevator elevator;
  private final AlgaeIntake algaeIntake;
  private final LEDs LED;
  private final Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param manip The subsystem used by this command.
   */
  public LimitedManipOuttake(Manipulator manip, Elevator elevator, LEDs LED, AlgaeIntake algaeIntake) {
    this.manip = manip;
    this.elevator = elevator;
    this.LED = LED;
    this.algaeIntake = algaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.isAtL1() || elevator.isAtL2() || elevator.isAtL3() || elevator.isAtL4()) {
      manip.runManip();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manip.stopManip();
    timer.stop();
    manip.setIsOuttaking(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (manip.getSecondBeam() && manip.getFirstBeam()) || timer.hasElapsed(0.5);
  }
}
