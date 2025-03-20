package frc.robot.commands.Chassis;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class OneDimensionalTrajectoryDrive extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final double minLogicDistance = 1.5;
    private final double normalCorrectionSpeed = 2;
    private final double tangentJoystickMultiplier = 2;
    private final RobotContainer robotContainer;
    private final CommandPS5Controller driverController;
    private final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
            Constants.Swerve.maxAngularRate,
            Constants.Swerve.maxAngularRate);
    private final ProfiledPIDController turningController = new ProfiledPIDController(12, 0, 0, rotationConstraints);
    private final Telemetry logger;
    private Pose2d targetPose = new Pose2d(3, 3, Rotation2d.kZero);
    private boolean runnable = false;
    private final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    boolean onBlue = true;
    boolean isAtPP = false;

    //left to right, top to bottom for blue/ red is rotated so it seems weird here
    private final Pose3d[] blueCoralTagPoses = {field.getTagPose(19).get(), field.getTagPose(20).get(),
            field.getTagPose(18).get(), field.getTagPose(21).get(),
            field.getTagPose(17).get(), field.getTagPose(22).get()};
    private final Pose3d[] redCoralTagPoses = {field.getTagPose(6).get(), field.getTagPose(8).get(),
            field.getTagPose(10).get(), field.getTagPose(7).get(),
            field.getTagPose(9).get(), field.getTagPose(11).get()};

    public OneDimensionalTrajectoryDrive(CommandSwerveDrivetrain commandSwerveDrivetrain, RobotContainer robotContainer,
                                         CommandPS5Controller driverController, Telemetry logger) {
        this.driveTrain = commandSwerveDrivetrain;
        this.robotContainer = robotContainer;
        this.driverController = driverController;
        this.logger = logger;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
        turningController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        isAtPP = false;
        var alliance = DriverStation.getAlliance();
        alliance.ifPresent(value -> onBlue = value == DriverStation.Alliance.Blue);
        turningController.reset(driveTrain.getStatePose().getRotation().getRadians());
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
            for (Pose3d redCoralTagPose : redCoralTagPoses) {
                Pose2d currentPose = redCoralTagPose.toPose2d();
                double x = currentPose.getX() - driveTrain.getStatePose().getX();
                double y = currentPose.getY() - driveTrain.getStatePose().getY();
                double distance = Math.sqrt((x * x) + (y * y));
                if (distance < lowestDistance) {
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
        double diffX = targetPose.getX() - driveTrain.getStatePose().getX();
        double diffY = targetPose.getY() - driveTrain.getStatePose().getY();
        double distance = Math.sqrt((diffX * diffX) + (diffY * diffY));
        double joystickY = driverController.getLeftY();
        if(runnable) {
            ChassisSpeeds chassisSpeeds = robotContainer.getHIDspeedsMPS();
            double xAxis = chassisSpeeds.vxMetersPerSecond;
            double yAxis = chassisSpeeds.vyMetersPerSecond;
            Translation2d vector = new Translation2d(xAxis, yAxis);
            double magnitude = vector.getNorm();

            Translation2d approach;
            if (minLogicDistance > distance) {
                Translation2d robotToTarget = new Translation2d(diffX, diffY);
                Translation2d unitTangent = new Translation2d(1, targetPose.getRotation());
                Translation2d tangent = unitTangent.times((unitTangent.getX())*robotToTarget.getX() + (unitTangent.getY())*robotToTarget.getY());
                Translation2d normal = robotToTarget.minus(tangent);
                approach = (unitTangent.times(tangentJoystickMultiplier*joystickY));
                if((targetPose.getX() < 4.3434) || (targetPose.getX() > 13.0302)) {
                    approach = approach.times(-1);
                }
                approach = approach.plus(normal.times(normalCorrectionSpeed));
            } else {
                approach = driveTrain.produceOneDimensionalTrajectory(targetPose);
                approach = approach.times(-magnitude*joystickY);
                if((targetPose.getX() > 4.3434) && (targetPose.getX() < 13.0302)) {
                    approach = approach.times(-1);
                }
            }

            if (!onBlue) {
                approach = approach.rotateBy(Rotation2d.k180deg);
            }
            double rotation = turningController.calculate(driveTrain.getStatePose().getRotation().getRadians(),
                    targetPose.getRotation().getRadians());
            ChassisSpeeds desiredDrive = new ChassisSpeeds(approach.getX(), approach.getY(), rotation);
            ChassisSpeeds limitedDesiredDrive = robotContainer.accelLimitVectorDrive(desiredDrive);
            driveTrain.setControl(robotContainer.drive.withVelocityX(limitedDesiredDrive.vxMetersPerSecond).
                    withVelocityY(limitedDesiredDrive.vyMetersPerSecond).
                    withRotationalRate(limitedDesiredDrive.omegaRadiansPerSecond));
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
     * recommended to use * {@link InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
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
