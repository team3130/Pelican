// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Limelight extends SubsystemBase {
  private final CommandSwerveDrivetrain driveTrain;
  private final Transform3d robotToCamera = new Transform3d();
  private final Vector<N3> visionStdDeviations = VecBuilder.fill(0, 0, 99999);
  private LimelightHelpers.PoseEstimate estimatedPose;
  boolean updated = false;

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
  }

  public void updateOdometry() {
    estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    driveTrain.setVisionMeasurementStdDevs(visionStdDeviations);
    driveTrain.addVisionMeasurement(estimatedPose.pose, estimatedPose.timestampSeconds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
