package frc.robot;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Arrays;

public class PowerLimiter implements Sendable {
    private double prevTime;
    private Translation2d prevState;
    public static final double linearDeadband = 0.05;
    public static final double angularDeadband = Math.PI/24;
    public static final double mpsScalar = 1;
    public static final double rpsScalar = Math.PI;
    public static final double massConstant = 65;
    public static final double RIConstant = 20;
    public double maxLinearEnergyConstant = 1;
    public double maxRotationalEnergyConstant = 1;
    public double maxCentripetalAcceleration = 0.1;

    public PowerLimiter(Translation2d joyStick){
        prevState = joyStick;
        prevTime = MathSharedStore.getTimestamp();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        SendableRegistry.addLW(this, name, name);
    }

    // Remove println statements
    public Translation2d calculateLinear(Translation2d desiredStateLinear) {
        RobotContainer robotContainer = new RobotContainer();
        ChassisSpeeds chassisSpeeds = robotContainer.getDriveTrain().getRobotRelativeSpeeds();
        double[] currentLinearSpeeds = {chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond};
        double[] desiredLinearSpeeds = {desiredStateLinear.getX() * mpsScalar, desiredStateLinear.getY() * mpsScalar};
        if(Math.abs(currentLinearSpeeds[0]) < linearDeadband) {
            currentLinearSpeeds[0] = 0;
        }
        if(Math.abs(currentLinearSpeeds[1]) < linearDeadband) {
            currentLinearSpeeds[1] = 0;
        }

        if((desiredLinearSpeeds[0] != 0) && (currentLinearSpeeds[0] != 0) && (currentLinearSpeeds[0]/Math.abs(currentLinearSpeeds[0]) != desiredLinearSpeeds[0]/Math.abs(desiredLinearSpeeds[0]))) {
            desiredLinearSpeeds[0] = 0;
        }
        if((desiredLinearSpeeds[1] != 0) && (currentLinearSpeeds[1] != 0) && (currentLinearSpeeds[1]/Math.abs(currentLinearSpeeds[1]) != desiredLinearSpeeds[1]/Math.abs(desiredLinearSpeeds[1]))) {
            desiredLinearSpeeds[1] = 0;
        }

        double projScalar = (currentLinearSpeeds[0]*desiredLinearSpeeds[0] + currentLinearSpeeds[1]*desiredLinearSpeeds[1])/(currentLinearSpeeds[0]*currentLinearSpeeds[0] + currentLinearSpeeds[1]*currentLinearSpeeds[1]);
        double[] projDesiredOnCurrent = {projScalar*currentLinearSpeeds[0], projScalar*currentLinearSpeeds[1]};
        double[] normDesiredOnCurrent = {desiredLinearSpeeds[0] - projDesiredOnCurrent[0],  desiredLinearSpeeds[1] - projDesiredOnCurrent[1]};
        System.out.println("\n" + "Proj: " + Arrays.toString(projDesiredOnCurrent));
        System.out.println("Norm: " + Arrays.toString(normDesiredOnCurrent));
        double linearEnergyChange = massConstant/2 * (Math.pow(projDesiredOnCurrent[0], 2) + Math.pow(projDesiredOnCurrent[1], 2) - Math.pow(currentLinearSpeeds[0], 2) - Math.pow(currentLinearSpeeds[1], 2));
        double absProj = Math.sqrt(Math.pow(projDesiredOnCurrent[0], 2) + Math.pow(projDesiredOnCurrent[1], 2));
        double absNorm = Math.sqrt(Math.pow(normDesiredOnCurrent[0], 2) + Math.pow(normDesiredOnCurrent[1], 2));

        if(projDesiredOnCurrent[0] > currentLinearSpeeds[0]) {
            if(linearEnergyChange > maxLinearEnergyConstant) {
                double maxAbsProj = Math.sqrt(maxLinearEnergyConstant*2/massConstant + (Math.pow(currentLinearSpeeds[0], 2) + Math.pow(currentLinearSpeeds[1], 2)));
                projDesiredOnCurrent[0] *= maxAbsProj/absProj;
                projDesiredOnCurrent[1] *= maxAbsProj/absProj;
                normDesiredOnCurrent[0] *= maxAbsProj/absProj;
                normDesiredOnCurrent[1] *= maxAbsProj/absProj;
            }
            System.out.println("\n" + "New Proj: " + Arrays.toString(projDesiredOnCurrent));
            System.out.println("\n" + "New Norm: " + Arrays.toString(normDesiredOnCurrent));
        }

        if(absNorm > maxCentripetalAcceleration) {
            normDesiredOnCurrent[0] *= maxCentripetalAcceleration/absNorm;
            normDesiredOnCurrent[1] *= maxCentripetalAcceleration/absNorm;
            System.out.println("\n" + "New Norm: " + Arrays.toString(normDesiredOnCurrent));
        }

        // Scale down
        return new Translation2d((projDesiredOnCurrent[0] + normDesiredOnCurrent[0])/mpsScalar, (projDesiredOnCurrent[1] + normDesiredOnCurrent[1])/mpsScalar);
    }

    // Remove println statements
    public Translation2d calculateRotational(Translation2d desiredStateRotational) {
        RobotContainer robotContainer = new RobotContainer();
        ChassisSpeeds chassisSpeeds = robotContainer.getDriveTrain().getRobotRelativeSpeeds();
        double currentAngularSpeed = chassisSpeeds.omegaRadiansPerSecond;
        double desiredAngularSpeed = Math.cos(desiredStateRotational.getAngle().getRadians()) * rpsScalar;
        System.out.println("\n" + "Desired: " + desiredAngularSpeed);
        if(Math.abs(currentAngularSpeed) < angularDeadband) {
            currentAngularSpeed = 0;
        }

        if((desiredAngularSpeed != 0) && (currentAngularSpeed != 0) && (currentAngularSpeed/Math.abs(currentAngularSpeed) != desiredAngularSpeed/Math.abs(desiredAngularSpeed))) {
            desiredAngularSpeed = 0;
        }

        double rotationalEnergyChange = RIConstant/2 * (Math.pow(desiredAngularSpeed, 2) - Math.pow(currentAngularSpeed, 2));
        if(rotationalEnergyChange > maxRotationalEnergyConstant) {
            desiredAngularSpeed = Math.sqrt(maxRotationalEnergyConstant*2/RIConstant + (Math.pow(currentAngularSpeed, 2) + Math.pow(currentAngularSpeed, 2)));
            System.out.println("\n" + "New Desired: " + desiredAngularSpeed);
        }

        return new Translation2d(1, new Rotation2d(Math.acos(desiredAngularSpeed/rpsScalar)));
    }

    public double getMaxLinearEnergyConstant() {
        return maxLinearEnergyConstant;
    }

    public void setMaxLinearEnergyConstant(double maxLinearEnergyConstant) {
        this.maxLinearEnergyConstant = maxLinearEnergyConstant;
    }

    public double getMaxRotationalEnergyConstant() {
        return maxRotationalEnergyConstant;
    }

    public void setMaxRotationalEnergyConstant(double maxRotationalEnergyConstant) {
        this.maxRotationalEnergyConstant = maxRotationalEnergyConstant;
    }

    public double getMaxCentripetalAcceleration() {
        return maxCentripetalAcceleration;
    }

    public void setMaxCentripetalAcceleration(double maxCentripetalAcceleration) {
        this.maxCentripetalAcceleration = maxCentripetalAcceleration;
    }
    @Override public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PowerLimiter");
        builder.addDoubleProperty("linearEnergyConstant", this::getMaxLinearEnergyConstant, this::setMaxLinearEnergyConstant);
        builder.addDoubleProperty("rotationalEnergyConstant", this::getMaxRotationalEnergyConstant, this::setMaxRotationalEnergyConstant);
        builder.addDoubleProperty("posMagLimit", this::getMaxCentripetalAcceleration, this::setMaxCentripetalAcceleration);
    }
}
