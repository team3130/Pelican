package frc.robot.commands.Chassis;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.MySlewRateLimiter;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class OneDimensionalTrajectoryDrive extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final SwerveRequest.FieldCentric drive;
    private final CommandPS5Controller driverController;
    private final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
            Constants.Swerve.maxAngularRate / (2 * Math.PI),
            Constants.Swerve.maxAngularRate/Math.PI);
    private final ProfiledPIDController turningController = new ProfiledPIDController(4, 0, 0, rotationConstraints);
    private final MySlewRateLimiter turningLimiter;
    private final Telemetry logger;
    private Pose2d targetPose = new Pose2d(3, 3, Rotation2d.kZero);
    private boolean runnable = false;
    private final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    //left to right, top to bottom for blue/ red is rotated so it seems weird here
    private final Pose3d[] blueCoralTagPoses = {field.getTagPose(19).get(), field.getTagPose(20).get(),
            field.getTagPose(18).get(), field.getTagPose(21).get(),
            field.getTagPose(17).get(), field.getTagPose(22).get()};
    private final Pose3d[] redCoralTagPoses = {field.getTagPose(6).get(), field.getTagPose(8).get(),
            field.getTagPose(10).get(), field.getTagPose(7).get(),
            field.getTagPose(9).get(), field.getTagPose(11).get()};

    public OneDimensionalTrajectoryDrive(CommandSwerveDrivetrain commandSwerveDrivetrain, SwerveRequest.FieldCentric drive,
                                         CommandPS5Controller driverController, Telemetry logger) {
        this.driveTrain = commandSwerveDrivetrain;
        this.drive = drive;
        this.driverController = driverController;
        this.logger = logger;
        turningLimiter = new MySlewRateLimiter(0.25, -0.25, driveTrain.getStatePose().getRotation().getRotations());
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
        turningController.enableContinuousInput(-.5, .5);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        boolean onBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        if(onBlue) {
            double lowestDistance = 1000;
            for(int i = 0; i < blueCoralTagPoses.length; i++) {
                Pose2d currentPose = blueCoralTagPoses[i].toPose2d();
                double x = currentPose.getX() - driveTrain.getStatePose().getX();
                double y = currentPose.getY() - driveTrain.getStatePose().getY();
                double distance = Math.sqrt((x * x) + (y * y));
                if(distance < lowestDistance) {
                    lowestDistance = distance;
                    targetPose = currentPose;
                }
            }
        } else {
            double lowestDistance = 1000;
            for(int i = 0; i < redCoralTagPoses.length; i++) {
                Pose2d currentPose = redCoralTagPoses[i].toPose2d();
                double x = currentPose.getX() - driveTrain.getStatePose().getX();
                double y = currentPose.getY() - driveTrain.getStatePose().getY();
                double distance = Math.sqrt((x * x) + (y * y));
                if(distance < lowestDistance) {
                    lowestDistance = distance;
                    targetPose = currentPose;
                }
            }
        }
        targetPose = targetPose.plus(new Transform2d(new Translation2d(.8, Rotation2d.kZero), Rotation2d.k180deg)); //.8 is in meters
        double deadband = 0.7;
        double joystickChoice = -driverController.getRightY();
        if(joystickChoice > deadband || joystickChoice < -deadband) {
            if (joystickChoice > 0) {
                //0.1651 is in meters and is equivalent to 6.5 inches
                targetPose = targetPose.plus(new Transform2d(new Translation2d(0.1651, Rotation2d.kCW_90deg), Rotation2d.kZero));
            }
            else {
                targetPose = targetPose.plus(new Transform2d(new Translation2d(0.1651, Rotation2d.kCCW_90deg), Rotation2d.kZero));
            }
            runnable = true;
        }
        logger.updateTarget(targetPose);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if(runnable) {
            Translation2d approach = driveTrain.produceOneDimensionalTrajectory(targetPose);
            approach = approach.div(approach.getNorm());
            Translation2d joystick = new Translation2d(driverController.getLeftX(), driverController.getLeftY());
            double magnitude = (-joystick.getY() * approach.getX()) + (-joystick.getX() * approach.getY()); //x and y should be flipped for field oriented
            magnitude *= Constants.Swerve.maxSpeed;
            double rotation = turningController.calculate(driveTrain.getStatePose().getRotation().getRotations(), targetPose.getRotation().getRotations());
            driveTrain.setControl(
                    drive.withVelocityX(approach.getX() * magnitude)
                            .withVelocityY(approach.getY() * magnitude)
                            .withRotationalRate(rotation)
            );
        }
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        //AutoBuilder.buildAuto(chosenPathName).schedule();
    }
}
