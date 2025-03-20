package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class Camera implements Sendable, Subsystem {
    private final PhotonCamera camera = new PhotonCamera("3130Camera");
    private final Transform3d robotToCamera = new Transform3d(0.34925, 0.27305, 0.34290, new Rotation3d(Math.PI,0,-0.005));
    private final String fieldName = Filesystem.getDeployDirectory().getPath() + "/2025-ERRshop-field.json";
    private final PhotonPoseEstimator photonPoseEstimator;
    private EstimatedRobotPose odoState;
    public Camera() {
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        /*
        try{
            aprilTagFieldLayout = new AprilTagFieldLayout(fieldName);
            System.out.println(fieldName);
            Alert alert = new Alert(fieldName, Alert.AlertType.kInfo);
            alert.set(true);
        } catch(Exception e) {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
            System.out.println("Fix ur Field ngl");
            System.out.println(fieldName);
        }
         */
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);
    }

    public void getVisionOdometry(CommandSwerveDrivetrain drivetrain, Telemetry logger) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            boolean inRange = false;
            double highestAmbiguity = 0;
            boolean reefId = false;
            for (PhotonTrackedTarget target: result.getTargets()) {
                double xSquared = target.getBestCameraToTarget().getX() * target.getBestCameraToTarget().getX();
                double ySquared = target.getBestCameraToTarget().getY() * target.getBestCameraToTarget().getY();
                double distance = Math.sqrt(xSquared + ySquared);
                if(target.getPoseAmbiguity() > highestAmbiguity) {
                    highestAmbiguity = target.getPoseAmbiguity();
                }
                if(distance < 4){
                    inRange = true;
                }
                int[] ids = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
                for(int id: ids) {
                    if(id == target.fiducialId) {
                        reefId = true;
                    }
                }
            }
            if(inRange && highestAmbiguity < 0.1 && reefId) {
                photonPoseEstimator.setReferencePose(drivetrain.getState().Pose);
                Optional<EstimatedRobotPose> optionalOdoState = photonPoseEstimator.update(result);
                if (optionalOdoState.isPresent()) {
                    odoState = optionalOdoState.get();
                    var newPose = odoState.estimatedPose.toPose2d();
                    logger.updateVision(newPose);
                    drivetrain.addVisionMeasurement(odoState.estimatedPose.toPose2d(), odoState.timestampSeconds);
                }
            }
        }
    }

    public double getXOdoState() {
        if(odoState != null) {
            return odoState.estimatedPose.getX();
        } else {
            return 0;
        }
    }
    public double getYOdoState() {
        if(odoState != null) {
            return odoState.estimatedPose.getY();
        } else {
            return 0;
        }
    }
    public double getZOdoState() {
        if(odoState != null) {
            return odoState.estimatedPose.getZ();
        } else {
            return 0;
        }
    }
    public double getRotationDegreesOdoState() {
        if(odoState != null) {
            return odoState.estimatedPose.getRotation().toRotation2d().getDegrees();
        } else {
            return 0;
        }
    }
    public String getOdoStateQuaternion() {
        if(odoState != null) {
            return odoState.estimatedPose.getRotation().getQuaternion().toString();
        } else {
            return "null";
        }
    }

    public void getFakeResult(CommandSwerveDrivetrain driveTrain) {
        Pose2d fakePose = new Pose2d(2, 2, new Rotation2d(0));
        driveTrain.addVisionMeasurement(fakePose, MathSharedStore.getTimestamp());
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Vision");
        builder.addDoubleProperty("Odo State X", this::getXOdoState, null);
        builder.addDoubleProperty("Odo State Y", this::getYOdoState, null);
        builder.addDoubleProperty("Odo State Z", this::getZOdoState, null);
        builder.addDoubleProperty("Odo State Rotation", this::getRotationDegreesOdoState, null);
        builder.addStringProperty("Odo State Quaternion", this::getOdoStateQuaternion, null);
    }
}