
package frc.robot.commands.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.MySlewRateLimiter;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public  class TeleopDrive extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    public final MySlewRateLimiter driveLimiter = new MySlewRateLimiter(0.5, -2, 0);
    public final MySlewRateLimiter thetaLimiter = new MySlewRateLimiter(0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.maxSpeed * 0.09).withRotationalDeadband(Constants.Swerve.maxAngularRate * 0.09) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final CommandPS5Controller controller;

 public TeleopDrive(CommandSwerveDrivetrain commandSwerveDrivetrain, CommandPS5Controller controller)
    {
          this.commandSwerveDrivetrain = commandSwerveDrivetrain;
          this.controller = controller;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.commandSwerveDrivetrain);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        commandSwerveDrivetrain.setControl(accelLimitVectorDrive(controller));
    }
    
    @Override
    public boolean isFinished()
    {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        
    }

    public SwerveRequest accelLimitVectorDrive(CommandPS5Controller driverController) {
        double xAxis = -driverController.getLeftY() * Math.abs(driverController.getLeftY()) * Constants.Swerve.maxSpeed;
        double yAxis = -driverController.getLeftX() * Math.abs(driverController.getLeftX()) * Constants.Swerve.maxSpeed;
        double rotation = -driverController.getRightX() * Constants.Swerve.maxAngularRate;
        double deadband = 0.09 * Constants.Swerve.maxSpeed;
        if(-deadband <= xAxis && xAxis <= deadband && -deadband <= yAxis && yAxis <= deadband) {
            thetaLimiter.reset(0);
            driveLimiter.reset(0);
            return drive.withVelocityX(xAxis).withVelocityY(yAxis).withRotationalRate(rotation);
        } else {
            Translation2d vector = new Translation2d(xAxis, yAxis);

            double theta  = thetaLimiter.getDelta(vector.getAngle().getRadians());;
            double mag = driveLimiter.calculate(vector.getNorm() * Math.cos(theta));
            mag = Math.max(mag, 0);
            if(driveLimiter.lastValue() == 0) {
                vector = new Translation2d(mag, vector.getAngle());
                thetaLimiter.reset(vector.getAngle().getRadians());
                return drive.withVelocityX(vector.getX()).withVelocityY(vector.getY()).withRotationalRate(rotation);
            }
            if(Math.cos(theta) <= 0) {
                vector = new Translation2d(mag, new Rotation2d(thetaLimiter.lastValue()));
                thetaLimiter.reset(thetaLimiter.lastValue());
                return drive.withVelocityX(vector.getX()).withVelocityY(vector.getY()).withRotationalRate(rotation);
            }
            if(mag < 4/Math.PI * thetaLimiter.getElapsedTime()) {
                vector = new Translation2d(mag, vector.getAngle());
                thetaLimiter.reset(vector.getAngle().getRadians());
                return drive.withVelocityX(vector.getX()).withVelocityY(vector.getY()).withRotationalRate(rotation);
            }

            double limit = 4 / mag;
            thetaLimiter.updateValues(limit, -limit);
            Rotation2d angle = new Rotation2d(thetaLimiter.angleCalculate(vector.getAngle().getRadians())); //calculate method with -pi to pi bounds
            vector = new Translation2d(mag, angle);
            return drive.withVelocityX(vector.getX()).withVelocityY(vector.getY()).withRotationalRate(rotation);
        }
    }
}
