// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeIntake.*;
import frc.robot.commands.Autos;
import frc.robot.commands.Camera.UpdateOdoFromVision;
import frc.robot.commands.Chassis.*;
import frc.robot.commands.Climber.*;
import frc.robot.commands.CoralIntake.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Elevator.GoToHome;
import frc.robot.commands.Manipulator.*;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.Command;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Timer timer = new Timer();
  public final MySlewRateLimiter driveLimiter = new MySlewRateLimiter(2, -5, 0);

  public final MySlewRateLimiter thetaLimiter;
  private boolean isAngleReal = false;
  private final double deadband = 0.05 * Constants.Swerve.maxSpeed;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Manipulator manip;
  private final Elevator elevator;
  private final CoralIntake coralIntake;
  private final AlgaeIntake algaeIntake;
  private final Climber climber;
  private final LEDs LED;
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(Constants.Swerve.maxSpeed * 0.05).withRotationalDeadband(Constants.Swerve.maxAngularRate * 0.09) // Add a 10% deadband
  
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity); // Use velocity control for drive motors
  private final Camera camera;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(Constants.Swerve.maxSpeed);

  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final CommandSwerveDrivetrain driveTrain = TunerConstants.createDrivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);

  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    thetaLimiter = new MySlewRateLimiter(0);
    manip = new Manipulator();
    elevator = new Elevator();
    coralIntake = new CoralIntake();
    algaeIntake = new AlgaeIntake();
    climber = new Climber();
    camera = new Camera(driveTrain);
    LED = new LEDs(elevator, manip, climber, camera, driveTrain);

    NamedCommands.registerCommand("Limited Manip Intake", new LimitedManipIntake(manip, elevator, LED));
    NamedCommands.registerCommand("Auton Limited Manip Intake", new AutonLimitedManipIntake(manip, elevator, LED));
    //only use command below at tulsa regional, fix for second break beam not working
    NamedCommands.registerCommand("Auton One Switch Limited Manip Intake", new AutonOneSwitchLimitedManipIntake(manip, elevator, LED));
    NamedCommands.registerCommand("Limited Manip Intake Reverse", new LimitedManipIntakeReverse(manip, LED));
    NamedCommands.registerCommand("Limited Manip Outtake", new LimitedManipOuttake(manip, elevator, LED));
    NamedCommands.registerCommand("Unlimited Run Manip", new UnlimitedRunManip(manip, elevator));

    NamedCommands.registerCommand("Go Home", new GoToHome(elevator, LED));
    NamedCommands.registerCommand("Go Min Position", new GoToMinPosition(elevator, LED));
    NamedCommands.registerCommand("Go L4", new GoToL4(elevator, manip, LED));
    NamedCommands.registerCommand("Go L3", new GoToL3(elevator, manip, LED));
    NamedCommands.registerCommand("Go L2", new GoToL2(elevator, manip, LED));
    NamedCommands.registerCommand("Go L1", new GoToL1(elevator, manip, LED));
    NamedCommands.registerCommand("Go Home", new GoToHome(elevator, LED));
    NamedCommands.registerCommand("Go L4 Basic", new GoToL4Basic(elevator, LED));
    NamedCommands.registerCommand("Go L3 Basic", new GoToL3Basic(elevator, LED));

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

    sendAutonChoosers();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link CommandPS4Controller
   * PS4} controllers or {@link CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //driverController.R2().whileTrue(new UnlimitedRunManip(manip, elevator));
    driverController.L3().whileTrue(new UnlimitedReverseRunManip(manip, elevator));
    //driverController.R2().whileTrue(new OneSwitchLimitedManipIntake(manip, elevator));
    driverController.L2().onTrue(new SequentialCommandGroup(
            new LimitedManipIntake(manip, elevator, LED),
            new LimitedManipIntakeReverse(manip, LED)
    ));
    driverController.R2().whileTrue(new LimitedManipIntakeOuttake(manip, elevator, LED));

    //driverController.R2().whileTrue(new UnlimitedCoralIntake(coralIntake));

    driverController.R3().whileTrue(new GoUp(elevator, LED));
    //driverController.L1().whileTrue(new GoDown(elevator));
    driverController.L1().onTrue(new GoToMinPosition(elevator, LED)); //loading position
    driverController.R1().onTrue(new GoToL4(elevator, manip, LED));
    driverController.square().onTrue(new GoToL3(elevator, manip, LED));
    driverController.cross().onTrue(new GoToL2(elevator, manip, LED));
    //driverController.triangle().onTrue(new GoToL1(elevator, manip, LED));
    driverController.povDown().onTrue(new GoToHome(elevator, LED));

    //driverController.square().whileTrue(new BasicClimberUp(climber));
    //driverController.triangle().whileTrue(new BasicClimberDown(climber));

    //driverController.triangle().whileTrue(new UpdateOdoFromVision(driveTrain, camera, logger));
    //driverController.square().whileTrue(new UpdateOdoFromPose(driveTrain, camera));
    //camera.setDefaultCommand(new UpdateSmartDashFromVisionOnly(driveTrain, camera, logger));
    //camera.setDefaultCommand(new UpdateOdoFromVision(driveTrain, camera, logger));

    //driverController.square().whileTrue(new TopALeftFolllowPath(driveTrain));
    //driverController.triangle().whileTrue(new TopARightFolllowPath(driveTrain));2
    driverController.axisMagnitudeGreaterThan(PS5Controller.Axis.kRightY.value, 0.7).whileTrue(new OneDimensionalTrajectoryDrive(driveTrain, this, driverController, logger));
    //driverController.povRight().whileTrue(new FollowClosestPath(driveTrain, driverController));
    //driverController.povRight().whileTrue(new DriveAtVelocity(driveTrain, drive));

    //driverController.povLeft().whileTrue(new UnlimitedCoralOuttake(coralIntake));
    //driverController.R2().whileTrue(new UnlimitedCoralIntake(coralIntake));
    //driverController.circle().onTrue(new IntakeActuate(coralIntake));
    driverController.povLeft().onTrue(new IntakeDeactuate(coralIntake));
    //driverController.circle().onTrue(new SequentialCommandGroup(new IntakeActuate(coralIntake), new GoToExtended(climber)));
    //coralIntake.setDefaultCommand(new IntakeActuateToSetpoint(coralIntake, operatorController));

    driverController.triangle().whileTrue(new BasicClimberDown(climber, LED));
    driverController.povRight().whileTrue(new BasicClimberUp(climber, LED));

    //operatorController.a().whileTrue(new GoToHome(elevator));
    //operatorController.b().whileTrue(new GoToL2Basic(elevator));
    //operatorController.x().whileTrue(new GoToL3Basic(elevator));
    //operatorController.y().whileTrue(new GoToL4Basic(elevator));
    //operatorController.povDown().whileTrue(new GoToL1Basic(elevator));

    //operatorController.rightBumper().whileTrue(new ActuateAlgaeIntake(algaeIntake));
    //operatorController.rightBumper().whileTrue(new RunAlgaeIntake(algaeIntake));
    //operatorController.leftBumper().whileTrue(new DeactuateAlgaeIntake(algaeIntake));
    //operatorController.povLeft().whileTrue(new RunAlgaeOuttake(algaeIntake));

    operatorController.a().whileTrue(new IntakeActuate(coralIntake));
    operatorController.povLeft().whileTrue(new BasicClimberUp(climber, LED));
    if(Constants.debugMode) {
      operatorController.b().whileTrue(new IntakeActuateToSetpoint(coralIntake, 0));
    }


    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    driveTrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            driveTrain.applyRequest(() -> {
              ChassisSpeeds chassisSpeed = accelLimitVectorDrive(getHIDspeedsMPS());
              return drive.withVelocityX(chassisSpeed.vxMetersPerSecond)
                      .withVelocityY(chassisSpeed.vyMetersPerSecond)
                      .withRotationalRate(chassisSpeed.omegaRadiansPerSecond);
            }));


            //driveTrain.applyRequest(this::accelLimitVectorDrive)

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
    SmartDashboard.putData(climber);
    SmartDashboard.putData(thetaLimiter);
    SmartDashboard.putData(camera);

    SmartDashboard.putData(logger.getField());
  }

  public Command pick() {
    return autoChooser.getSelected();
  }

  public void setElevatorZeroed(boolean value) {elevator.setZeroed(value);}
  public Command elevatorHome() {return new GoToHome(elevator, LED);}
  public Command algaeActuationHome() {return new AlgaeActuationGoHome(algaeIntake);}
  public Command climberHome() {return new ZeroClimber(climber, LED);}
  public Command intakeDeactuate() {return new IntakeDeactuate(coralIntake);}
  public void basicVisionResetOdo() {
    camera.getVisionOdometry(logger);
  }
  public void visionResetOdo() {
    if(driveTrain.getState().Speeds.vxMetersPerSecond < 0.05 && driveTrain.getState().Speeds.vyMetersPerSecond < 0.05) {
      if(timer.isRunning()) {
        if (timer.hasElapsed(0.1)) {
          camera.getVisionOdometry(logger);
        }
      } else {
        timer.start();
      }
    } else {
      timer.stop();
      timer.reset();
    }
  }

  public void sendAutonChoosers() {
    SendableChooser<Command> pathChooser1 = PathChooser.buildAndSendCoralChooser("Coral 1", manip, elevator, LED);
    SendableChooser<Command> pathChooser2 = PathChooser.buildAndSendCoralChooser("Coral 2", manip, elevator, LED);
    SendableChooser<Command> pathChooser3 = PathChooser.buildAndSendCoralChooser("Coral 3", manip, elevator, LED);
    SendableChooser<Command> stationChooser1 = PathChooser.buildAndSendStationChooser("Station 1", manip, elevator, LED);
    SendableChooser<Command> stationChooser2 = PathChooser.buildAndSendStationChooser("Station 2", manip, elevator, LED);
    SendableChooser<Command> stationChooser3 = PathChooser.buildAndSendStationChooser("Station 3", manip, elevator, LED);

    SmartDashboard.putData("Coral 1 Path", pathChooser1);
    SmartDashboard.putData("Coral 2 Path", pathChooser2);
    SmartDashboard.putData("Coral 3 Path", pathChooser3);
    SmartDashboard.putData("Station 1 Path", stationChooser1);
    SmartDashboard.putData("Station 2 Path", stationChooser2);
    SmartDashboard.putData("Station 3 Path", stationChooser3);
  }

  public SequentialCommandGroup configureAuton() {
    return PathChooser.buildAutoCommand(elevator, LED);
  }

  public double getModularSpeed() {
    if(elevator.brokeBottomLimitSwitch()) {
      return Constants.Swerve.maxSpeed;
    } else if(elevator.brokeTopLimitSwitch()) {
      return Constants.Swerve.maxSpeedFullExtended;
    } else {
      return Constants.Swerve.maxSpeedPartiallyExtended;
    }
  }

  public double getElevatorRealPercent() {
    return getElevatorPercentSpeed() / Constants.Swerve.maxSpeed;
  }

  public double getElevatorPercentSpeed() {
    double maxSpeed = Constants.Swerve.maxSpeed;
    double minSpeed = 1;
    double range = maxSpeed - minSpeed;
    double untranslatedSpeed = (elevator.getPosition() / elevator.getMaxPosition()) * range;
    double realSpeed = maxSpeed - untranslatedSpeed;
    return realSpeed;
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

  public SwerveRequest driveRequest() {
    double xAxis = -driverController.getLeftY();
    double yAxis = -driverController.getLeftX();
    return drive.withVelocityX(xAxis * Constants.Swerve.maxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(yAxis * Constants.Swerve.maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * Constants.Swerve.maxAngularRate); // Drive counterclockwise with negative X (left)
  }

  public boolean isWithinDeadband(Translation2d vector) {
    return (vector.getNorm() <= deadband);
  }

  public ChassisSpeeds getHIDspeedsMPS() {
    double xAxis = -driverController.getLeftY();
    double yAxis = -driverController.getLeftX();
    double rotation = -driverController.getRightX();
    xAxis = MathUtil.applyDeadband(xAxis, Constants.Swerve.kDeadband);
    yAxis = MathUtil.applyDeadband(yAxis, Constants.Swerve.kDeadband);
    rotation = MathUtil.applyDeadband(rotation, Constants.Swerve.kDeadband);
    xAxis *= Math.abs(xAxis) * Constants.Swerve.maxSpeed * getElevatorRealPercent();
    yAxis *= Math.abs(yAxis) * Constants.Swerve.maxSpeed * getElevatorRealPercent();
    rotation *= Math.abs(rotation) * Constants.Swerve.maxAngularRate * getElevatorRealPercent();
    return new ChassisSpeeds(xAxis, yAxis, rotation);
  }

  public ChassisSpeeds accelLimitVectorDrive(ChassisSpeeds desiredSpeed) {
    double xAxis = desiredSpeed.vxMetersPerSecond;
    double yAxis = desiredSpeed.vyMetersPerSecond;
    double rotation = desiredSpeed.omegaRadiansPerSecond;
    Translation2d vector = new Translation2d(xAxis, yAxis);
    if(isAngleReal) { //if angle is real, then we were moving 20 ms ago
      if(vector.getNorm() > 0.001){ //if the norm is significant, we continue to move
        double delta = thetaLimiter.getDelta(vector.getAngle().getRadians());
        double cos = Math.cos(delta);
         if(cos > 0){ //positive cos means keep moving (turn angle is small)
           var mag = vector.getNorm() * cos;
           driveLimiter.setPositiveRateLimit(driveLimiter.getLinearPositiveRateLimit(driveLimiter.lastValue()));
           mag = driveLimiter.calculate(mag);
           double thetaLimiterConstant = 10;
           double limit = thetaLimiterConstant /mag;
           thetaLimiter.updateValues(limit, -limit);
           var theta = thetaLimiter.angleCalculate(vector.getAngle().getRadians());
           Translation2d newVector = new Translation2d(mag, new Rotation2d(theta));
           return new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation);
         }
      }
      //here we continue if we are decelerating, either small mag or big turn.
      thetaLimiter.reset(thetaLimiter.lastValue());
      driveLimiter.setPositiveRateLimit(driveLimiter.getLinearPositiveRateLimit(driveLimiter.lastValue()));
      var newMag = driveLimiter.calculate(0);
      Rotation2d angle = new Rotation2d(thetaLimiter.lastValue());
      Translation2d newVector = new Translation2d(newMag, angle);
      if(newMag < 0.001){ // we have stopped moving
        isAngleReal = false;
      }
      return new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation);
    }
    else { //if angle is not real, then we were standing still 20 ms ago
      if(vector.getNorm() < 0.001){ //if the norm is still tiny, then keep idling
        driveLimiter.reset(0);
        return new ChassisSpeeds(0,0, rotation);
      }
      else { //if the norm is significant, start driving
        isAngleReal = true;
        thetaLimiter.reset(vector.getAngle().getRadians());
        driveLimiter.setPositiveRateLimit(driveLimiter.getLinearPositiveRateLimit(driveLimiter.lastValue()));
        var mag = driveLimiter.calculate(vector.getNorm());
        Translation2d newVector = new Translation2d(mag, vector.getAngle());
        return new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation);
      }
    }
  }
}
