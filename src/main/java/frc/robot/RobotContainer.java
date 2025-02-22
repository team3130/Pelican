// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeIntake.*;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralIntake.LimitedCoralIntake;
import frc.robot.commands.CoralIntake.UnlimitedCoralIntake;
import frc.robot.commands.CoralIntake.UnlimitedCoralOuttake;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Manipulator.*;
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
  public final SlewRateLimiter driveLimiter = new SlewRateLimiter(0.5, -1, 0);
  public final SlewRateLimiter steerLimiter = new SlewRateLimiter(1);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Manipulator manip;
  private final Elevator elevator;
  private final CoralIntake coralIntake;
  private final AlgaeIntake algaeIntake;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(Constants.Swerve.maxSpeed * 0.09).withRotationalDeadband(Constants.Swerve.maxAngularRate * 0.09) // Add a 10% deadband
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(Constants.Swerve.maxSpeed);

  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final CommandSwerveDrivetrain driveTrain = frc.robot.TunerConstants.createDrivetrain();

  private final CommandPS5Controller driverController = new CommandPS5Controller(0);

  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    manip = new Manipulator();
    elevator = new Elevator();
    coralIntake = new CoralIntake();
    algaeIntake = new AlgaeIntake();

    NamedCommands.registerCommand("Limited Manip Intake", new LimitedManipIntake(manip, elevator));
    NamedCommands.registerCommand("UnLimited Manip Outtake", new LimitedManipOuttake(manip, elevator));

    NamedCommands.registerCommand("Go Min Position", new GoToMinPosition(elevator));
    NamedCommands.registerCommand("Go L4", new GoToL4(elevator));
    NamedCommands.registerCommand("Go L3", new GoToL3(elevator));
    NamedCommands.registerCommand("Go L2", new GoToL2(elevator));
    NamedCommands.registerCommand("Go L1", new GoToL1(elevator));
    NamedCommands.registerCommand("Go L4 Basic", new GoToL4Basic(elevator));

    NamedCommands.registerCommand("Toggle Algae Intake", new ActuateAlgaeIntake(algaeIntake));
    NamedCommands.registerCommand("Run Algae Intake", new RunAlgaeIntake(algaeIntake));
    NamedCommands.registerCommand("Run Algae Outtake", new RunAlgaeOuttake(algaeIntake));

    NamedCommands.registerCommand("Limited Coral Intake", new LimitedCoralIntake(coralIntake, manip));
    NamedCommands.registerCommand("UnLimited Coral Outtake", new UnlimitedCoralOuttake(coralIntake));

    // Configure the trigger bindings
    configureBindings();
    exportSmartDashboardData();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    driverController.R2().whileTrue(new UnlimitedRunManip(manip, elevator));
    driverController.L2().whileTrue(new UnlimitedReverseRunManip(manip, elevator));

    //driverController.R2().whileTrue(new UnlimitedRunManip(manip));

    //driverController.L3().onTrue(new GoToMinPosition(elevator)); //loading position
    driverController.R3().whileTrue(new GoToL4Basic(elevator));
    driverController.povDown().whileTrue(new GoToL3Basic(elevator));
    driverController.circle().whileTrue(new GoToL2Basic(elevator));
    //driverController.triangle().onTrue(new GoToL1(elevator));

    driverController.L1().whileTrue(new GoDown(elevator));
    driverController.R1().whileTrue(new GoUp(elevator));

    //driverController.cross().whileTrue(new ToggleAlgaeActuation(algaeIntake));
    //driverController.R3().whileTrue(new RunAlgaeIntake(algaeIntake));
    operatorController.y().whileTrue(new ActuateAlgaeIntake(algaeIntake));
    operatorController.x().whileTrue(new DeactuateAlgaeIntake(algaeIntake));
    //driverController.L1().whileTrue(new RunAlgaeOuttake(algaeIntake));

    driverController.povLeft().whileTrue(new UnlimitedCoralOuttake(coralIntake));
    driverController.R2().whileTrue(new UnlimitedCoralIntake(coralIntake));

    operatorController.a().whileTrue(new GoToHome(elevator));
    operatorController.b().whileTrue(new GoToL2Basic(elevator));
    operatorController.x().whileTrue(new GoToL3Basic(elevator));
    operatorController.y().whileTrue(new GoToL4Basic(elevator));
    operatorController.povDown().whileTrue(new GoToL1Basic(elevator));

    operatorController.rightBumper().whileTrue(new ActuateAlgaeIntake(algaeIntake));
    operatorController.rightBumper().whileTrue(new RunAlgaeIntake(algaeIntake));
    operatorController.leftBumper().whileTrue(new DeactuateAlgaeIntake(algaeIntake));
    operatorController.povLeft().whileTrue(new RunAlgaeOuttake(algaeIntake));

    operatorController.rightTrigger().whileTrue(new UnlimitedRunManip(manip, elevator));
    operatorController.leftTrigger().whileTrue(new UnlimitedReverseRunManip(manip, elevator));
    operatorController.rightTrigger().whileTrue(new UnlimitedCoralIntake(coralIntake));

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    driveTrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            driveTrain.applyRequest(this::modularDriveRequest));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    operatorController.back().and(operatorController.y()).whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    operatorController.back().and(operatorController.x()).whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    operatorController.start().and(operatorController.y()).whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    operatorController.start().and(operatorController.x()).whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driverController.povUp().onTrue(driveTrain.runOnce(() -> driveTrain.seedFieldCentric()));

    driveTrain.registerTelemetry(logger::telemeterize);
  }

  public void exportSmartDashboardData() {
    SmartDashboard.putData(manip);
    SmartDashboard.putData(coralIntake);
    SmartDashboard.putData(algaeIntake);
    SmartDashboard.putData(elevator);
  }

  public Command pick() {
    return autoChooser.getSelected();
  }

  public Command elevatorHome() {return new GoToHome(elevator);}
  public Command algaeActuationHome() {return new AlgaeActuationGoHome(algaeIntake);}

  public SwerveRequest modularDriveRequest() {
    double maxSpeed = 0;
    if(elevator.brokeBottomLimitSwitch()) {
      maxSpeed = Constants.Swerve.maxSpeed;
    } else if(elevator.brokeTopLimitSwitch()) {
      maxSpeed = Constants.Swerve.maxSpeedFullExtended;
    } else {
      maxSpeed = Constants.Swerve.maxSpeedPartiallyExtended;
    }
    double xAxis = -driverController.getLeftY() * Math.abs(driverController.getLeftY()) * maxSpeed;
    double yAxis = -driverController.getLeftX() * Math.abs(driverController.getLeftX()) * maxSpeed;
    double rotation = -driverController.getRightX() * Math.abs(driverController.getRightX()) * maxSpeed;
    return drive.withVelocityX(xAxis).withVelocityY(yAxis).withRotationalRate(rotation);
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
