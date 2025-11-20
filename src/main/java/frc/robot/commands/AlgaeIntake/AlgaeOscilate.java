// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/** An example command that uses an example subsystem. */
public class AlgaeOscilate extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake algaeIntake;
  private double currentPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param algaeIntake The subsystem used by this command.
   */
  public AlgaeOscilate(AlgaeIntake algaeIntake) {
    this.algaeIntake = algaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPosition = algaeIntake.getPosition() + 1;
    System.out.println(algaeIntake.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntake.intakeSetpoint(currentPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
