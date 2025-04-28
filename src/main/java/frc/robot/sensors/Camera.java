package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final CommandSwerveDrivetrain driveTrain;
    private final PhotonCamera cameraLeft = new PhotonCamera("3130Camera");
    private final PhotonCamera cameraRight = new PhotonCamera("3130CameraRight");
    private final Transform3d robotToCameraLeft = new Transform3d(0.287, 0.275, 0.395, new Rotation3d(2.926,0.208,-0.2814+0.0268));
    private final Transform3d robotToCameraRight = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    private final String fieldName = Filesystem.getDeployDirectory().getPath() + "/2025-ERRshop-field.json";
    //private final Vector<N3> visionStdDeviations = VecBuilder.fill(0.25, 0.25, 1);
    private final PhotonPoseEstimator photonPoseEstimatorLeft;
    private final PhotonPoseEstimator photonPoseEstimatorRight;
    private EstimatedRobotPose odoStateLeft;
    private EstimatedRobotPose odoStateRight;
    private boolean updatedLeft = false;
    private boolean updatedRight = false;
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
        photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCameraLeft);
        photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCameraRight);
        //driveTrain.setVisionMeasurementStdDevs(visionStdDeviations);
    }
    public void getVisionOdometry(Telemetry logger) {
        getVisionOdometryLeft(logger);
        getVisionOdometryRight(logger);
    }

    public void getVisionOdometryLeft(Telemetry logger) {
        //Matrix<N3, N1> scaledVisionStdDeviations = visionStdDeviations;
        List<PhotonPipelineResult> results = cameraLeft.getAllUnreadResults();
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
                if(distance < 2) {
                    inRange = true;
                } else {
                    inRange = false;
                }
                //scaledVisionStdDeviations = visionStdDeviations.times(1 + distance);
            }
            if(DriverStation.isDSAttached() && DriverStation.isDisabled()) {
                inRange = true;
            }

            Optional<EstimatedRobotPose> optionalOdoState = photonPoseEstimatorLeft.update(result);
            if (optionalOdoState.isPresent()) {
                odoStateLeft = optionalOdoState.get();

                if (odoStateLeft.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                    photonPoseEstimatorLeft.setReferencePose(driveTrain.getState().Pose);
                    var newPose = odoStateLeft.estimatedPose.toPose2d();
                    logger.updateVision(newPose);
                    driveTrain.addVisionMeasurement(
                            odoStateLeft.estimatedPose.toPose2d(),
                            odoStateLeft.timestampSeconds
                            //scaledVisionStdDeviations
                    );
                    updatedLeft = true;

                } else {
                    if (inRange && highestAmbiguity < 0.2) {
                        photonPoseEstimatorLeft.setReferencePose(driveTrain.getState().Pose);
                        var newPose = odoStateLeft.estimatedPose.toPose2d();
                        logger.updateVision(newPose);
                        driveTrain.addVisionMeasurement(
                                odoStateLeft.estimatedPose.toPose2d(),
                                odoStateLeft.timestampSeconds
                                //scaledVisionStdDeviations
                        );
                        updatedLeft = true;
                    } else {
                        updatedLeft = false;
                    }
                }
            } else {
                updatedLeft = false;
            }
        }
    }

    public void getVisionOdometryRight(Telemetry logger) {
        //Matrix<N3, N1> scaledVisionStdDeviations = visionStdDeviations;
        List<PhotonPipelineResult> results = cameraRight.getAllUnreadResults();
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
                if(distance < 2) {
                    inRange = true;
                } else {
                    inRange = false;
                }
                //scaledVisionStdDeviations = visionStdDeviations.times(1 + distance);
            }
            if(DriverStation.isDSAttached() && DriverStation.isDisabled()) {
                inRange = true;
            }

            Optional<EstimatedRobotPose> optionalOdoState = photonPoseEstimatorRight.update(result);
            if (optionalOdoState.isPresent()) {
                odoStateRight = optionalOdoState.get();

                if (odoStateRight.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                    photonPoseEstimatorRight.setReferencePose(driveTrain.getState().Pose);
                    var newPose = odoStateRight.estimatedPose.toPose2d();
                    logger.updateVision(newPose);
                    driveTrain.addVisionMeasurement(
                            odoStateRight.estimatedPose.toPose2d(),
                            odoStateRight.timestampSeconds
                            //scaledVisionStdDeviations
                    );
                    updatedRight = true;

                } else {
                    if (inRange && highestAmbiguity < 0.2) {
                        photonPoseEstimatorLeft.setReferencePose(driveTrain.getState().Pose);
                        var newPose = odoStateLeft.estimatedPose.toPose2d();
                        logger.updateVision(newPose);
                        driveTrain.addVisionMeasurement(
                                odoStateLeft.estimatedPose.toPose2d(),
                                odoStateLeft.timestampSeconds
                                //scaledVisionStdDeviations
                        );
                        updatedRight = true;
                    } else {
                        updatedRight = false;
                    }
                }
            } else {
                updatedRight = false;
            }
        }
    }

    public boolean getHasTarget() {
        return updatedLeft || updatedRight;
    }

    public double getXOdoState() {
        if(odoStateLeft != null) {
            return odoStateLeft.estimatedPose.getX();
        } else {
            return 0;
        }
    }
    public double getYOdoState() {
        if(odoStateLeft != null) {
            return odoStateLeft.estimatedPose.getY();
        } else {
            return 0;
        }
    }
    public double getZOdoState() {
        if(odoStateLeft != null) {
            return odoStateLeft.estimatedPose.getZ();
        } else {
            return 0;
        }
    }
    public double getRotationDegreesOdoState() {
        if(odoStateLeft != null) {
            return odoStateLeft.estimatedPose.getRotation().toRotation2d().getDegrees();
        } else {
            return 0;
        }
    }
    public String getOdoStateQuaternion() {
        if(odoStateLeft != null) {
            return odoStateLeft.estimatedPose.getRotation().getQuaternion().toString();
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