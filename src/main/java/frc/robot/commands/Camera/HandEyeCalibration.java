
package frc.robot.commands.Camera;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;


public  class HandEyeCalibration extends Command
{
    private final Camera camera;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final double[] xOdo = new double[100];
    private final double[] yOdo = new double[100];
    private double timeOfPrevMeasurement = 0;

 public HandEyeCalibration(Camera camera, CommandSwerveDrivetrain commandSwerveDrivetrain)
    {
          this.camera = camera;
          this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.camera, this.commandSwerveDrivetrain);
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

        if(isSlow() && (timeSincePrevMeasurement() > 5000)) {
            timeOfPrevMeasurement = MathSharedStore.getTimestamp();
            photonVisionMeasurement();
            odometryMeasurement();
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
        
    }

    public double timeSincePrevMeasurement() {
        return MathSharedStore.getTimestamp() - timeOfPrevMeasurement;
    }

    public boolean isSlow() {
        return Math.hypot(xOdo[0] - xOdo[99], yOdo[0] - yOdo[99]) <= 0.01;
    }

    public void photonVisionMeasurement() {
        List<PhotonPipelineResult> results = camera.getResults();
        for (PhotonTrackedTarget target : results.get(results.size() - 1).getTargets()) {
            Transform3d camToTarget = target.getBestCameraToTarget();
            Mat vec1 = new Mat(3, 1, CvType.CV_64F);
            vec1.put(0, 0, -camToTarget.getX(), -camToTarget.getY(), -camToTarget.getZ());
            camera.addPhotonTranslation(vec1);
            Mat mat1 = new Mat(3, 3, CvType.CV_64F);
            Matrix<N3, N3> mat = camToTarget.getRotation().toMatrix().inv();
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    mat1.put(row, col, mat.get(row, col));
                }
            }
            camera.addPhotonRotation(mat1);
        }
    }

    public void odometryMeasurement() {
        Mat vec2 = new Mat(3, 1, CvType.CV_64F);
        vec2.put(0, 0, -camera.getXOdoState(), -camera.getYOdoState(), 0);
        camera.addOdometryTranslation(vec2);
        Mat mat2 = new Mat(3, 3, CvType.CV_64F);
        Matrix<N3, N3> mat = (new Rotation3d(new Rotation2d(camera.getRotationDegreesOdoState()))).toMatrix().inv();
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                mat2.put(row, col, mat.get(row, col));
            }
        }
        camera.addOdometryTranslation(mat2);
    }
}
