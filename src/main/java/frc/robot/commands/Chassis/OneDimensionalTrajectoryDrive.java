package frc.robot.commands.Chassis;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.MySlewRateLimiter;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class OneDimensionalTrajectoryDrive extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private double prevVal;
    private double prevTime;
    private final double minLogicDistanceTangent = 100;
    private final double minLogicDistanceNormal = 100;
    private final double normSpeed = 2;
    private final double normAcceleration = 2.5;
    private final double tolerance = 0.005;
    private final ProfiledPIDController pidController = new ProfiledPIDController(Constants.Swerve.translationPID[0], Constants.Swerve.translationPID[1], Constants.Swerve.translationPID[2],
            new TrapezoidProfile.Constraints(normSpeed, normAcceleration));
    private final double tangentJoystickMultiplier = Math.sqrt(Constants.Swerve.maxSpeed*Constants.Swerve.maxSpeed - normSpeed*normSpeed);
    private final RobotContainer robotContainer;
    private final CommandPS5Controller driverController;
    private final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
            Constants.Swerve.maxAngularRate,
            Constants.Swerve.maxAngularRate); //todo make angular accel real because these should not be the same number
    private final ProfiledPIDController turningController = new ProfiledPIDController(12, 0, 0, rotationConstraints);
    private final Telemetry logger;
    private Pose2d targetPose = new Pose2d(3, 3, Rotation2d.kZero);
    private boolean runnable = false;
    private final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    boolean onBlue = true;
    boolean useMinLogicDistance = false;

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

    public Translation2d getRobotToTarget(Translation2d vector) {
        double magnitude = vector.getNorm();
        double diffX = targetPose.getX() - driveTrain.getStatePose().getX();
        double diffY = targetPose.getY() - driveTrain.getStatePose().getY();
        double distance = Math.sqrt((diffX * diffX) + (diffY * diffY));
        return new Translation2d(diffX, diffY);
    }

    public Translation2d getUnitTangent() {
        return new Translation2d(1, targetPose.getRotation());
    }

    public Translation2d getNormal(Translation2d vector) {
        Translation2d robotToTarget = getRobotToTarget(vector);
        Translation2d unitTangent = getUnitTangent();
        Translation2d tangent = unitTangent.times((unitTangent.getX())*robotToTarget.getX() + (unitTangent.getY())*robotToTarget.getY());
        return robotToTarget.minus(tangent);
    }

    public double getNormalCurrentSpeed(Translation2d vector) {
        Translation2d normal = getNormal(vector);
        double normalSpeed = normal.getX()*driveTrain.getState().Speeds.vxMetersPerSecond + normal.getY()*driveTrain.getState().Speeds.vyMetersPerSecond;
        return Math.abs(normalSpeed);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        double delta = input - this.prevVal; //input is desired speed
        this.prevVal += MathUtil.clamp(delta, -40 * elapsedTime, 10 * elapsedTime);
        this.prevTime = currentTime;
        return this.prevVal;
    }
    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */

    @Override
    public void initialize() {
        useMinLogicDistance = false;
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
            ChassisSpeeds chassisSpeeds = robotContainer.getHIDspeedsMPS();
            double xAxis = chassisSpeeds.vxMetersPerSecond;
            double yAxis = chassisSpeeds.vyMetersPerSecond;
            Translation2d vector = new Translation2d(xAxis, yAxis);
            Translation2d normalPosDirection = new Translation2d(-getUnitTangent().getY(), getUnitTangent().getX());
            double sign;
            if((getNormal(vector).getX()*normalPosDirection.getX() + getNormal(vector).getY()*normalPosDirection.getY()) > 0) {
                sign = 1;
            } else {
                sign = -1;
            }
            pidController.reset(sign*getNormal(vector).getNorm()); //todo is it right to be getting norm on the distance getter here?
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
            ChassisSpeeds chassisSpeeds = robotContainer.getHIDspeedsMPS();
            double xAxis = chassisSpeeds.vxMetersPerSecond;
            double yAxis = chassisSpeeds.vyMetersPerSecond;
            Translation2d vector = new Translation2d(xAxis, yAxis);
            Translation2d unitTangent = getUnitTangent();
            Translation2d normal = getNormal(vector);
            Translation2d normalPosDirection = new Translation2d(-getUnitTangent().getY(), getUnitTangent().getX());
            double sign;
            if((getNormal(vector).getX()*normalPosDirection.getX() + getNormal(vector).getY()*normalPosDirection.getY()) > 0) {
                sign = 1;
            } else {
                sign = -1;
            }

            Translation2d approach;
            if (true) {
                useMinLogicDistance = true;
                double dotMultiplier = vector.getX()*unitTangent.getX() + vector.getY()*unitTangent.getY();
                approach = (unitTangent.times(calculate(Math.abs(tangentJoystickMultiplier*dotMultiplier))));
                if(tangentJoystickMultiplier*dotMultiplier < 0) {
                    approach = approach.times(-1);
                }
                if (!onBlue) {
                    approach = approach.rotateBy(Rotation2d.k180deg);
                }
                double mag = pidController.calculate(sign*normal.getNorm(), 0);
                if(normal.getNorm() > tolerance) {
                    approach = approach.plus(normalPosDirection.times(-mag));
                }
            } else {
                approach = driveTrain.produceOneDimensionalTrajectory(targetPose);
                approach = approach.times(vector.getNorm());
            }
            if (!onBlue) {
                approach = approach.rotateBy(Rotation2d.k180deg);
            }
            double rotation = turningController.calculate(driveTrain.getStatePose().getRotation().getRadians(),
                    targetPose.getRotation().getRadians());
            ChassisSpeeds desiredDrive = new ChassisSpeeds(
                    approach.getX() * robotContainer.getElevatorRealPercent(),
                    approach.getY() * robotContainer.getElevatorRealPercent(),
                    rotation);
            ChassisSpeeds limitedDesiredDrive = robotContainer.accelLimitVectorDrive(desiredDrive) ;
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
