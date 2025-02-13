// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.AlgaeIntake.ActuateAlgaeIntake;
import frc.robot.commands.AlgaeIntake.RunAlgaeIntake;
import frc.robot.commands.AlgaeIntake.RunAlgaeOuttake;
import frc.robot.commands.Autos;
import frc.robot.commands.Chassis.ResetOdometryForward;
import frc.robot.commands.CoralIntake.UnlimitedCoralOuttake;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Manipulator.OneSwitchLimitedManipIntake;
import frc.robot.commands.Manipulator.OneSwitchLimitedManipOuttake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Chassis chassis = new Chassis();
  private final Manipulator manip = new Manipulator();
  private final Elevator elevator = new Elevator();
  private final CoralIntake coralIntake = new CoralIntake();
  private final Climber climber = new Climber();
  private final AlgaeIntake algaeIntake = new AlgaeIntake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.R2().whileTrue(new OneSwitchLimitedManipOuttake(manip, elevator));
    driverController.L2().whileTrue(new OneSwitchLimitedManipIntake(manip, elevator));

    driverController.L1().whileTrue(new GoToMinPosition(elevator)); //loading position
    driverController.R1().whileTrue(new GoToL4(elevator));
    driverController.povDown().whileTrue(new GoToL3(elevator));
    driverController.circle().whileTrue(new GoToL2(elevator));
    driverController.triangle().whileTrue(new GoToL1(elevator));

    driverController.cross().whileTrue(new ActuateAlgaeIntake(algaeIntake));
    driverController.R3().whileTrue(new RunAlgaeIntake(algaeIntake));
    driverController.L3().whileTrue(new RunAlgaeOuttake(algaeIntake));

    driverController.povUp().whileTrue(new ResetOdometryForward(chassis));

    driverController.povLeft().whileTrue(new UnlimitedCoralOuttake(coralIntake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
