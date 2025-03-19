// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDs;

/** An example command that uses an example subsystem. */
public class GoToExtended extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber climber;
  private final LEDs LED;

  /**
   * Creates a new ExampleCommand.
   *
   * @param climber The subsystem used by this command.
   */
  public GoToExtended(Climber climber, LEDs LED) {
    this.climber = climber;
    this.LED = LED;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(climber.isZeroed()) {
      climber.climbUp();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LED.setLEDstateClimber();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getPosition() >= climber.getExtendedPos();
  }
}
