package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

public class test {

    public static void main(String[] args) {
        Point[] imgpoints = {new Point(Math.tan(-0.35), Math.tan(-1.25)), new Point(Math.tan(4.27), Math.tan(-6.85)),
                new Point(Math.tan(-3.52), Math.tan(6.65)), new Point(Math.tan(-0.55), Math.tan(-6.61))};

        Point[] wrldpoints = {new Point(81, -11), new Point(55, -11), new Point(107, -3), new Point(94, -30)};

        Mat homographyMatrix = homographyMatrix(imgpoints, wrldpoints, 5);

        System.out.println(homographyMatrix.dump());
        System.out.println("hi");
    }

    public static Mat homographyMatrix(Point[] imagePointArray, Point[] worldPointArray, double threshold) {
        MatOfPoint2f imagePoints = new MatOfPoint2f(imagePointArray);
        MatOfPoint2f worldPoints = new MatOfPoint2f(worldPointArray);

        return Calib3d.findHomography(imagePoints, worldPoints, Calib3d.RANSAC, threshold);
    }


    public static Translation2d[] computeHomography(MatOfPoint2f imagePoints, Mat homography) {
        MatOfPoint2f worldPoints = new MatOfPoint2f();
        Core.perspectiveTransform(imagePoints, worldPoints, homography);
        Translation2d[] points = new Translation2d[worldPoints.rows()];

        int i = 0;
        for (Point p : worldPoints.toList()) {
            points[i] = new Translation2d(p.x, p.y);
            i++;
        }

        return points;
    }
}
