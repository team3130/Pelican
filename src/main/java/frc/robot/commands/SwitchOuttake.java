// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.AlgaeIntake.RunAlgaeOuttake;
import frc.robot.commands.Manipulator.LimitedManipOuttake;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwitchOuttake extends Command {
  private final AlgaeIntake algaeIntake;
  private final Manipulator manip;
  private final Elevator elevator;
  private final LEDs leds;
  private final CommandPS5Controller controller;
  /** Creates a new SwitchOuttake. */
  public SwitchOuttake(AlgaeIntake algaeIntake, Manipulator manip, Elevator elevator, LEDs leds, CommandPS5Controller controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeIntake = algaeIntake;
    this.manip = manip;
    this.elevator = elevator;
    this.leds = leds;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(algaeIntake.getAlgaeMode()) {
      new RunAlgaeOuttake(algaeIntake, controller).schedule();
    } else {
      new LimitedManipOuttake(manip, elevator, leds, algaeIntake).schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
