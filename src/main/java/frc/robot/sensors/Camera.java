package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.OpenCvLoader;
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
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Core;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Optional;
import org.opencv.core.CvType;
import org.photonvision.targeting.TargetCorner;

public class Camera implements Sendable, Subsystem {
    int hTeCalibrationIndex = 0;
    private double[] xOdo = new double[100];
    private double[] yOdo = new double[100];
    private double timeOfPrevMeasurement = 0;
    private ArrayList<Mat> tTcTranslations = {};
    private ArrayList<Mat> gTbTranslations = {};
    private Mat hTeTranslation;
    private ArrayList<Mat> tTcRotations = {};
    private ArrayList<Mat> gTbRotations = {};
    private Mat hTeRotation;
    private final CommandSwerveDrivetrain driveTrain;
    private final PhotonCamera camera = new PhotonCamera("3130Camera");
    private final Transform3d robotToCamera = new Transform3d(0.287, 0.275, 0.395, new Rotation3d(3.0042,0.2186,-0.2814+0.0268-0.0642));
    private final String fieldName = Filesystem.getDeployDirectory().getPath() + "/2025-ERRshop-field.json";
    //private final Vector<N3> visionStdDeviations = VecBuilder.fill(0.25, 0.25, 1);
    private final PhotonPoseEstimator photonPoseEstimator;
    private EstimatedRobotPose odoState;
    private final Mat matrix;
    double[] values = {-0.0001270542819333261, 0.0008182770377802222, 0.1862827299336119,
        0.0006939972013693427, 0.0001741107045706584, -0.5076447771203513,
        0.0006253018433804395, -0.002952410871268828, 1};
    private boolean updated = false;
    public Camera(CommandSwerveDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        OpenCvLoader.forceStaticLoad();
        matrix = new Mat(3, 3, CvType.CV_64F);
        matrix.put(0, 0, values);
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
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        //driveTrain.setVisionMeasurementStdDevs(visionStdDeviations);
    }

    public void getVisionOdometry(Telemetry logger) {
        //Matrix<N3, N1> scaledVisionStdDeviations = visionStdDeviations;
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            boolean inRange = false;
            double highestAmbiguity = 0;
            for (PhotonTrackedTarget target : result.getTargets()) {
                double xSquared = target.getBestCameraToTarget().getX() * target.getBestCameraToTarget().getX();
                double ySquared = target.getBestCameraToTarget().getY() * target.getBestCameraToTarget().getY();
                double distance = Math.sqrt(xSquared + ySquared);
                if (target.getPoseAmbiguity() > highestAmbiguity) {
                    highestAmbiguity = target.getPoseAmbiguity();
                }
                if (distance < 2) {
                    inRange = true;
                } else {
                    inRange = false;
                }
                //scaledVisionStdDeviations = visionStdDeviations.times(1 + distance);
            }
            if (DriverStation.isDSAttached() && DriverStation.isDisabled()) {
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

    public Translation2d[] computeHomography(MatOfPoint2f imagePoints) {
        MatOfPoint2f worldPoints = new MatOfPoint2f();
        if(imagePoints.empty()) {
            Translation2d[] empty = new Translation2d[1];
            empty[0] = new Translation2d(0, 0);
            return empty;
        }
        Core.perspectiveTransform(imagePoints, worldPoints, matrix);
        Translation2d[] points = new Translation2d[worldPoints.rows()];

        int i = 0;
        for(Point p: worldPoints.toList()) {
            points[i] = new Translation2d(p.x, p.y);
            i++;
        }

        return points;
    }

    public void handEyeMeasurements() {
        for(int i = 98; i >= 0; i--) {
            xOdo[i + 1] = xOdo[i];
            yOdo[i + 1] = yOdo[i];
        }
        xOdo[0] = getXOdoState();
        yOdo[0] = getYOdoState();

        if((Math.hypot(xOdo[0] - xOdo[99], yOdo[0] - yOdo[99]) <= 0.01) && (MathSharedStore.getTimestamp() - timeOfPrevMeasurement > 5000)) {
            timeOfPrevMeasurement = MathSharedStore.getTimestamp();
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    Transform3d camToTarget = target.getBestCameraToTarget();
                    Mat vec1 = new Mat(3, 1, CvType.CV_64F);
                    vec1.put(0, 0, -camToTarget.getX(), -camToTarget.getY(), -camToTarget.getZ());
                    tTcTranslations.add(vec1);
                    Mat mat1 = new Mat(3, 3, CvType.CV_64F);
                    Matrix<N3, N3> mat = camToTarget.getRotation().toMatrix();
                    for (int row = 0; row < 3; row++) {
                        for (int col = 0; col < 3; col++) {
                            mat1.put(row, col, mat.get(row, col));
                        }
                    }
                    tTcRotations.add(mat1);
                }
            }
            Mat vec2 = new Mat(3, 1, CvType.CV_64F);
            vec2.put(0, 0, -getXOdoState(), -getYOdoState(), 0);
            gTbTranslations.add(vec2);
            Mat mat2 = new Mat(3, 3, CvType.CV_64F);
            Matrix<N3, N3> mat = (new Rotation3d(new Rotation2d(getRotationDegreesOdoState()))).toMatrix();
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    mat2.put(row, col, mat.get(row, col));
                }
            }
            gTbRotations.add(mat2);
            hTeCalibrationIndex++;
        }
    }

    public void handEyeCalibration() {
        Calib3d.calibrateHandEye(gTbRotations, gTbTranslations, tTcRotations, tTcTranslations, hTeRotation, hTeTranslation);
    }

    public MatOfPoint2f getObjectData() {
        camera.setPipelineIndex(1);
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results == null) return new MatOfPoint2f();
        double xTotal = 0;
        double yTotal = 0;
        double index = 0;
        for (PhotonPipelineResult result : results) {
            for (PhotonTrackedTarget target: result.getTargets()) {
                List<TargetCorner> corners = target.getMinAreaRectCorners();
                double xSum = 0.0;
                double ySum = 0.0;

                for (TargetCorner corner : corners) {
                    xSum += corner.x;
                    ySum += corner.y;
                }

                xTotal += xSum / corners.size();
                yTotal += ySum / corners.size();

                index++;
            }
        }
        if(index == 0) {
            return null;
        }
        return new MatOfPoint2f(new Point(xTotal / index, yTotal / index)); //average of all points
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