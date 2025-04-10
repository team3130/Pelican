package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class Camera implements Sendable, Subsystem {
    private final CommandSwerveDrivetrain driveTrain;
    private final PhotonCamera camera = new PhotonCamera("3130Camera");
    private final Transform3d robotToCamera = new Transform3d(0.287, 0.275, 0.395, new Rotation3d(2.926,0.208,-0.2814));
    private final String fieldName = Filesystem.getDeployDirectory().getPath() + "/2025-ERRshop-field.json";
    //private final Vector<N3> visionStdDeviations = VecBuilder.fill(0.25, 0.25, 1);
    private final PhotonPoseEstimator photonPoseEstimator;
    private EstimatedRobotPose odoState;
    private boolean updated = false;
    public Camera(CommandSwerveDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
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
        //driveTrain.setVisionMeasurementStdDevs(visionStdDeviations);
    }

    public void getVisionOdometry(Telemetry logger) {
        //Matrix<N3, N1> scaledVisionStdDeviations = visionStdDeviations;
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            boolean inRange = false;
            double highestAmbiguity = 0;
            for (PhotonTrackedTarget target: result.getTargets()) {
                double xSquared = target.getBestCameraToTarget().getX() * target.getBestCameraToTarget().getX();
                double ySquared = target.getBestCameraToTarget().getY() * target.getBestCameraToTarget().getY();
                double distance = Math.sqrt(xSquared + ySquared);
                if(target.getPoseAmbiguity() > highestAmbiguity) {
                    highestAmbiguity = target.getPoseAmbiguity();
                }
                if(distance < 2){
                    inRange = true;
                } else {
                    inRange = false;
                }
                //scaledVisionStdDeviations = visionStdDeviations.times(1 + distance);
            }
            if(DriverStation.isDSAttached() && DriverStation.isDisabled()) {
                inRange = true;
            }

            Optional<EstimatedRobotPose> optionalOdoState = photonPoseEstimator.update(result);
            if (optionalOdoState.isPresent()) {
                odoState = optionalOdoState.get();

                if (odoState.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                    photonPoseEstimator.setReferencePose(driveTrain.getState().Pose);
                    var newPose = odoState.estimatedPose.toPose2d();
                    logger.updateVision(newPose);
                    driveTrain.addVisionMeasurement(
                            odoState.estimatedPose.toPose2d(),
                            odoState.timestampSeconds
                            //scaledVisionStdDeviations
                    );
                    updated = true;

                } else {
                    if (inRange && highestAmbiguity < 0.2) {
                        photonPoseEstimator.setReferencePose(driveTrain.getState().Pose);
                        var newPose = odoState.estimatedPose.toPose2d();
                        logger.updateVision(newPose);
                        driveTrain.addVisionMeasurement(
                                odoState.estimatedPose.toPose2d(),
                                odoState.timestampSeconds
                                //scaledVisionStdDeviations
                        );
                        updated = true;
                    } else {
                        updated = false;
                    }
                }
            } else {
                updated = false;
            }
        }
    }

    public boolean getHasTarget() {
        return updated;
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
        builder.addBooleanProperty("Has Target", this::getHasTarget, null);
        builder.addDoubleProperty("Odo State X", this::getXOdoState, null);
        builder.addDoubleProperty("Odo State Y", this::getYOdoState, null);
        builder.addDoubleProperty("Odo State Z", this::getZOdoState, null);
        builder.addDoubleProperty("Odo State Rotation", this::getRotationDegreesOdoState, null);
        builder.addStringProperty("Odo State Quaternion", this::getOdoStateQuaternion, null);
    }
}