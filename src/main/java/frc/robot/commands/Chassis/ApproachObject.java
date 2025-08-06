package frc.robot.commands.Chassis;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import java.util.Arrays;


public class ApproachObject extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    public final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
           .withDeadband(Constants.Swerve.maxSpeed * 0.05).withRotationalDeadband(Constants.Swerve.maxAngularRate * 0.09) // Add a 10% deadband
           .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final Camera camera;
    private Translation2d[] vectors;

    public ApproachObject(CommandSwerveDrivetrain driveTrain, Camera camera) {
        this.driveTrain = driveTrain;
        this.camera = camera;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize(){

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        MatOfPoint2f objectData = camera.getObjectData();
        vectors = camera.computeHomography(objectData);
        System.out.println("Get Object Data: ");
        System.out.println(objectData.dump());
        System.out.println("Compute Homography Test With [400, 600]: ");
        System.out.println(camera.computeHomography(new MatOfPoint2f(new Point(400, 600)))[0]);
        System.out.println("Vectors: ");
        System.out.println(vectors[0]);
        driveTrain.applyRequest(() -> {
            ChassisSpeeds speeds = driveTrain.accelLimitVectorDrive(new ChassisSpeeds(vectors[0].getX(), vectors[0].getY(), 0));
            return drive.withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond);
        });
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
        driveTrain.applyRequest(() -> {
            return drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0);
        });
    }
}
