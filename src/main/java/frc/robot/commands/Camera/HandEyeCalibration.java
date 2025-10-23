
package frc.robot.commands.Camera;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;


public  class HandEyeCalibration extends Command
{
    private final Camera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private double time = Math.pow(10, 8);
    private final double[] xOdo = new double[50];
    private final double[] yOdo = new double[50];
    private double timeOfPrevMeasurement = 0;
    private ArrayList<Mat> tTcTranslations = new ArrayList<>();
    private ArrayList<Mat> gTbTranslations = new ArrayList<>();
    private Mat hTeTranslation = new Mat(3, 3, CvType.CV_64F);
    private ArrayList<Mat> tTcRotations = new ArrayList<>();
    private ArrayList<Mat> gTbRotations = new ArrayList<>();
    private Mat hTeRotation = new Mat(3, 3, CvType.CV_64F);
    private boolean gotPhotonMeasurement = false;

 public HandEyeCalibration(Camera camera, CommandSwerveDrivetrain drivetrain)
    {
          this.camera = camera;
          this.drivetrain = drivetrain;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.camera);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        List<PhotonPipelineResult> results = camera.getResults();
        for(int i = 48; i >= 0; i--) {
            xOdo[i + 1] = xOdo[i];
            yOdo[i + 1] = yOdo[i];
        }
        Pose2d statePose = drivetrain.getStatePose();
        xOdo[0] = statePose.getX();
        yOdo[0] = statePose.getY();

        if(isSlow() && (timeSincePrevMeasurement() > 0.5)) {
            gotPhotonMeasurement = false;
            timeOfPrevMeasurement = MathSharedStore.getTimestamp();
            photonVisionMeasurement(results);
            if(gotPhotonMeasurement) {
                odometryMeasurement();
                time = Math.pow(10, 9);
            }
        }
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
        if(!gTbRotations.isEmpty()) {
            Calib3d.calibrateHandEye(gTbRotations, gTbTranslations, tTcRotations, tTcTranslations, hTeRotation, hTeTranslation);
            System.out.println("Hand to Eye Translation Matrix =\n" + hTeTranslation.dump());
            System.out.println("Hand to Eye Rotation Matrix =\n" + hTeRotation.dump());
            /*
            double[] entries = new double[9];
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {

                }
            }
            Matrix<N3, N3> matrix = new Matrix<>(new Nat.N3(), 3);
            Rotation3d rotation = new Rotation3d();
            System.out.println("Pitch");
        } else {
            System.out.println("Matrices are Empty");
            System.out.println(MathSharedStore.getTimestamp());
        }
        */
        tTcTranslations = new ArrayList<>();
        gTbTranslations = new ArrayList<>();
        hTeTranslation = new Mat(3, 1, CvType.CV_64F);
        tTcRotations = new ArrayList<>();
        gTbRotations = new ArrayList<>();
        hTeRotation = new Mat(3, 3, CvType.CV_64F);
    }

    public double timeSincePrevMeasurement() {
        return MathSharedStore.getTimestamp() - timeOfPrevMeasurement;
    }

    public boolean isSlow() {
        return Math.hypot(xOdo[0] - xOdo[49], yOdo[0] - yOdo[49]) <= 0.005;
    }

    public void photonVisionMeasurement(List<PhotonPipelineResult> results) {
        time = Math.min(MathSharedStore.getTimestamp(), time);
        if(results.get(results.size() - 1).getTimestampSeconds() > time) {
            return;
        }
        if(results.isEmpty()) {
            return;
        }
        for (PhotonTrackedTarget target : results.get(results.size() - 1).getTargets()) {
            gotPhotonMeasurement = true;
            Transform3d camToTarget = target.getBestCameraToTarget();
            Mat vec1 = new Mat(3, 1, CvType.CV_64F);
            vec1.put(0, 0, camToTarget.getX(), camToTarget.getY(), camToTarget.getZ());
            tTcTranslations.add(vec1);
            System.out.println("Added Vision Vec " + vec1.dump());
            Mat mat1 = new Mat(3, 3, CvType.CV_64F);
            Matrix<N3, N3> mat = camToTarget.getRotation().toMatrix();
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    mat1.put(row, col, mat.get(row, col));
                }
            }
            tTcRotations.add(mat1);
            System.out.println("Added Vision Mat " + mat1.dump());
        }
    }

    public void odometryMeasurement() {
        Pose2d statePose = drivetrain.getStatePose();
        Mat vec2 = new Mat(3, 1, CvType.CV_64F);
        vec2.put(0, 0, statePose.getX(), statePose.getY(), 0);
        gTbTranslations.add(vec2);
        System.out.println("Added Odometry Vec " + vec2.dump());
        Mat mat2 = new Mat(3, 3, CvType.CV_64F);
        Matrix<N3, N3> mat = (new Rotation3d(statePose.getRotation())).toMatrix();
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                mat2.put(row, col, mat.get(row, col));
            }
        }
        gTbRotations.add(mat2);
        System.out.println("Added Odometry Mat " + mat2.dump());
    }
}
