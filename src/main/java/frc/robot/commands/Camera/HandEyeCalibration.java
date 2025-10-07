
package frc.robot.commands.Camera;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Camera;
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
    private final double[] xOdo = new double[100];
    private final double[] yOdo = new double[100];
    private double timeOfPrevMeasurement = 0;
    private ArrayList<Mat> tTcTranslations = new ArrayList<>();
    private ArrayList<Mat> gTbTranslations = new ArrayList<>();
    private Mat hTeTranslation = new Mat(3, 3, CvType.CV_64F);
    private ArrayList<Mat> tTcRotations = new ArrayList<>();
    private ArrayList<Mat> gTbRotations = new ArrayList<>();
    private Mat hTeRotation = new Mat(3, 3, CvType.CV_64F);
    private boolean gotPhotonMeasurement = false;

 public HandEyeCalibration(Camera camera)
    {
          this.camera = camera;
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
        for(int i = 98; i >= 0; i--) {
            xOdo[i + 1] = xOdo[i];
            yOdo[i + 1] = yOdo[i];
        }
        xOdo[0] = camera.getXOdoState();
        yOdo[0] = camera.getYOdoState();

        if(isSlow() && (timeSincePrevMeasurement() > 5)) {
            gotPhotonMeasurement = false;
            timeOfPrevMeasurement = MathSharedStore.getTimestamp();
            photonVisionMeasurement();
            if(gotPhotonMeasurement) {
                odometryMeasurement();
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
        } else {
            System.out.println("Matrices are Empty");
            System.out.println(MathSharedStore.getTimestamp());
        }

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
        return Math.hypot(xOdo[0] - xOdo[99], yOdo[0] - yOdo[99]) <= 1;
    }

    public void photonVisionMeasurement() {
        List<PhotonPipelineResult> results = camera.getResults();
        for (PhotonTrackedTarget target : results.get(results.size() - 1).getTargets()) {
            gotPhotonMeasurement = true;
            Transform3d camToTarget = target.getBestCameraToTarget();
            Mat vec1 = new Mat(3, 1, CvType.CV_64F);
            vec1.put(0, 0, -camToTarget.getX(), -camToTarget.getY(), -camToTarget.getZ());
            tTcTranslations.add(vec1);
            System.out.println("Added Vision Vec " + vec1.dump());
            Mat mat1 = new Mat(3, 3, CvType.CV_64F);
            Matrix<N3, N3> mat = camToTarget.getRotation().toMatrix().inv();
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
        Mat vec2 = new Mat(3, 1, CvType.CV_64F);
        vec2.put(0, 0, -camera.getXOdoState(), -camera.getYOdoState(), 0);
        gTbTranslations.add(vec2);
        System.out.println("Added Odometry Vec " + vec2.dump());
        Mat mat2 = new Mat(3, 3, CvType.CV_64F);
        Matrix<N3, N3> mat = (new Rotation3d(new Rotation2d(camera.getRotationDegreesOdoState()))).toMatrix().inv();
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                mat2.put(row, col, mat.get(row, col));
            }
        }
        gTbRotations.add(mat2);
        System.out.println("Added Odometry Mat " + mat2.dump());
    }
}
