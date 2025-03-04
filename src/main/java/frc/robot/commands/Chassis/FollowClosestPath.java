package frc.robot.commands.Chassis;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.json.simple.parser.ParseException;

import java.io.IOException;


public class FollowClosestPath extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final CommandPS5Controller driverController;
    private final String[][] pathNames = {
            {"TopALeftFollow", "TopARightFollow"},
            {"TopBLeftFollow", "TopBRightFollow"},
            {"MiddleALeftFollow", "MiddleARightFollow"},
            {"MiddleBLeftFollow", "MiddleBRightFollow"},
            {"BottomALeftFollow", "BottomARightFollow"},
            {"BottomBLeftFollow", "BottomBRightFollow"}
    };
    private final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    //left to right, top to bottom
    private final double[] blueCoralTagNums = {19, 20, 18, 21, 17, 22};
    private final double[] redCoralTagNums = {9, 8, 10, 7, 11, 6};
    private final Pose3d[] blueCoralTagPoses = {field.getTagPose(19).get(), field.getTagPose(20).get(),
            field.getTagPose(18).get(), field.getTagPose(21).get(),
            field.getTagPose(17).get(), field.getTagPose(22).get()};
    private final Pose3d[] redCoralTagPoses = {field.getTagPose(9).get(), field.getTagPose(8).get(),
            field.getTagPose(10).get(), field.getTagPose(7).get(),
            field.getTagPose(11).get(), field.getTagPose(6).get()};

    private boolean onBlue = false;

    public FollowClosestPath(CommandSwerveDrivetrain commandSwerveDrivetrain, CommandPS5Controller driverController) {
        this.driveTrain = commandSwerveDrivetrain;
        this.driverController = driverController;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            onBlue = true;
        } else {
            onBlue = false;
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        int sideChosen = -1;
        if(onBlue) {
            double lowestDistance = 1000;
            for(int i = 0; i < blueCoralTagPoses.length; i++) {
                Pose3d tagPose = blueCoralTagPoses[i];
                double x = tagPose.getX() - driveTrain.getStatePose().getX();
                double y = tagPose.getY() - driveTrain.getStatePose().getY();
                double distance = Math.sqrt((x * x) + (y * y));
                if(distance < lowestDistance) {
                    lowestDistance = distance;
                    sideChosen = i;
                }
            }
        } else {
            double lowestDistance = 1000;
            for(int i = 0; i < redCoralTagPoses.length; i++) {
                Pose3d tagPose = redCoralTagPoses[i];
                double x = tagPose.getX() - driveTrain.getStatePose().getX();
                double y = tagPose.getY() - driveTrain.getStatePose().getY();
                double distance = Math.sqrt((x * x) + (y * y));
                if(distance < lowestDistance) {
                    lowestDistance = distance;
                    sideChosen = i;
                }
            }
        }
        int leftOrRight = -1;
        double upperDeadband = 0.7;
        double lowerDeadband = 0.3;
        double joystickChoice = driverController.getRightY();
        if(joystickChoice > upperDeadband || joystickChoice < lowerDeadband) {
            if (joystickChoice > upperDeadband) {
                leftOrRight = 0;
            } else {
                leftOrRight = 1;
            }
            try {
                driveTrain.produceTrajectory(pathNames[sideChosen][leftOrRight]);
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
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

    }
}
