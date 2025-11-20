// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

import java.awt.*;

/** An example command that uses an example subsystem. */
public class LimitedManipIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator manip;
  private final Elevator elevator;
  private final AlgaeIntake algaeIntake;
  private final CommandPS5Controller controller;
  private final LEDs LED;

  /**
   * Creates a new ExampleCommand.
   *
   * @param manip The subsystem used by this command.
   */
  public LimitedManipIntake(Manipulator manip, Elevator elevator, LEDs LED, AlgaeIntake algaeIntake, CommandPS5Controller controller) {
    this.manip = manip;
    this.elevator = elevator;
    this.LED = LED;
    this.algaeIntake = algaeIntake;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevator.isAtMinPosition()) {
      manip.runManip();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.isAtMinPosition()) {
      if (!manip.getFirstBeam() && !manip.getSecondBeam()) {
        manip.manipAtSpeed(0.4);
      } else {
        manip.runManip();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manip.stopManip();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manip.getFirstBeam() && !manip.getSecondBeam();
  }
}
