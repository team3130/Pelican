package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Constants;


// Swerve module that reflects the actual swerve modules
public class SwerveModule implements Sendable {
    private final TalonFX steerMotor; // the steering motor
    private final TalonFX driveMotor; // the driving motor
    private final CANcoder absoluteEncoder; // the can encoder attached to the shaft
    private final PIDController turningPidController; // PID controller for steering
    private final double absoluteEncoderOffset; // the absolute encoder offset from where 0 is to where it thinks it is
    private final int side; // the side that the bot is on
    final VoltageOut steerMotorVoltRequest = new VoltageOut(0);
    final VoltageOut driveMotorVoltRequest = new VoltageOut(0);
    final TalonFXConfiguration steerTalonConfig = new TalonFXConfiguration();
    final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();

    private final VelocityDutyCycle driveVeloRequest;
    private Slot0Configs driveConfigs;
    private double drive_kA = 0;
    private double drive_kV = 0;
    private double drive_kP = 0;
    private double drive_kI = 0;
    private double drive_kD = 0;

    private final MotionMagicDutyCycle steerMotionRequest;
    private Slot0Configs steerConfigs;
    private double steer_kP = 0;
    private double steer_kI = 0;
    private double steer_kD = 0;

    private double targetStateRotiations = 0;


    //private double steeringVoltage = 4d;
    //private double drivingVoltage = 10d;

    /**
     * Initializes a swerve module and its motors.
     * Initializes the steering PID controller.
     * @param side is reflective in {@link Constants}
     */
    public SwerveModule(int side) {
        steerMotor = new TalonFX(Constants.Swerve.turningID[side]);
        driveMotor = new TalonFX(Constants.Swerve.spinningID[side]);

        absoluteEncoder = new CANcoder(Constants.Swerve.CANCoders[side]);
        turningPidController = new PIDController(Constants.Swerve.kP_Swerve[side], Constants.Swerve.kI_Swerve[side], Constants.Swerve.kD_Swerve[side]);

        steerTalonConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive); //set config to clockwise
        steerTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerTalonConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicAcceleration(40)
                .withMotionMagicCruiseVelocity(10)
                .withMotionMagicJerk(0);
        if(side == 0 || side == 1) {
            steerTalonConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConversions.frontSteerGearRatio;
        } else {
            steerTalonConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConversions.backSteerGearRatio;
        }
        steerMotor.getConfigurator().apply(steerTalonConfig); // config factory default + my settings
        steerMotor.setNeutralMode(NeutralModeValue.Brake); // Brake mode
        //steerMotor.setInverted(true);

        driveTalonConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive); //set config to counter-clockwise
        driveMotor.getConfigurator().apply(driveTalonConfig);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        //driveMotor.setInverted(false);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
        turningPidController.setTolerance(0.0025, 0.05); // at position tolerance

        absoluteEncoderOffset = Constants.SwerveEncoderOffsets.kCANCoderOffsets[side];
        this.side =side;

        driveConfigs = new Slot0Configs()
                .withKA(drive_kA).withKV(drive_kV).withKP(drive_kP).withKI(drive_kI).withKD(drive_kD);
        driveMotor.getConfigurator().apply(driveConfigs);
        driveVeloRequest = new VelocityDutyCycle(0).withSlot(0);

        steerConfigs = new Slot0Configs()
                .withKP(steer_kP).withKI(steer_kI).withKD(steer_kD);
        steerMotor.getConfigurator().apply(steerConfigs);
        steerMotionRequest = new MotionMagicDutyCycle(0).withSlot(0);

        driveMotor.getConfigurator().apply((new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentLimit(40).withSupplyCurrentLowerTime(0)), 0.01);

        steerMotor.getConfigurator().apply((new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentLimit(20).withSupplyCurrentLowerTime(0)), 0.01);

        driveMotor.getConfigurator().apply((new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).
                withStatorCurrentLimit(40).withSupplyCurrentLowerTime(0)), 0.01);

        steerMotor.getConfigurator().apply((new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).
                withStatorCurrentLimit(20).withSupplyCurrentLowerTime(0)), 0.01);


        resetEncoders();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        name += " " + side;
        SendableRegistry.addLW(this, name, name);
    }

    // returns the amount of distance the drive motor has travelled in meters
    public double getDrivePosition() {
        if (side == 0 || side == 1) {
            return driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.frontDriveRotToMeters;
        } else {
            return driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.backDriveRotToMeters;
        }
    }

    // returns the position of the steering motor radians
    public Rotation2d getTurningPosition() {
        // return steerMotor.getPosition().getValue() * Constants.Conversions.SteerRotToRads;
        if (side == 0 || side == 1) {
            return new Rotation2d(steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.frontSteerRotToRads);
        } else {
            return new Rotation2d(steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.backSteerRotToRads);
        }
    }

    public double getTurningPositionRadians() {
        if (side == 0 || side == 1) {
            return steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.frontSteerRotToRads;
        } else {
            return steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.backSteerRotToRads;
        }
    }

    // gets the velocity of the drive motor in m/s
    public double getDriveVelocity() {
        if (side == 0 || side == 1) {
            return driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.frontDriveRotToMeters * 10d;
        } else {
            return driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.backDriveRotToMeters * 10d;
        }
    }

    // gets the speed at which the steering motor turns in radians per second
    public double getTurningVelocity() {
        if (side == 0 || side == 1) {
            return steerMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.frontSteerRotToRads * 10d;
        } else {
            return steerMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.backSteerRotToRads * 10d;
        }
    }

    // gets the position of the steering wheel according to the absolute encoders
    public double getAbsoluteEncoderRots() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void updatePValue(double p) {
        turningPidController.setP(p);
    }
    public void updateDValue(double d) {
        turningPidController.setD(d);
    }

    /**
     * Resets the relative encoders according the absolute encoder involving the offset
     */
    public void resetEncoders() {
        steerMotor.setPosition(getAbsoluteEncoderRots() - absoluteEncoderOffset);
    }

    /**
     * Whether the wheels are zeroed or not
     * @return custom at set-point logic for the PID controller
     */
    public boolean wheelsZeroed() {
        Rotation2d pos = getTurningPosition();
        return (pos.getDegrees() > 355 || pos.getDegrees() < 5) && getTurningVelocity() < 0.05;
    }

    /**
     * @return the current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurningPosition());
    }

    //Method runs just with a speed of 1
    public void basicRun(double speed) {
        driveMotor.setControl(driveMotorVoltRequest.withOutput(speed));
    }

    // Default stop method to stop the motors
    public void stop() {
        steerMotor.setControl(steerMotorVoltRequest.withOutput(0));
        driveMotor.setControl(driveMotorVoltRequest.withOutput(0));
    }

    /**
     * Set the desired swerve module state
     * @param state the state to set the swerve modules to
     */
    public void setTeleopDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state.optimize(getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage(Constants.Swerve.maxDriveVoltage * (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(Constants.Swerve.maxSteerVoltage * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    public void setAutonDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state.optimize(getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage((10d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(4d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    public void setTeleopVeloDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        updateDrivePID(); //just for testing
        updateSteerPID(); //just for testing

        // max turn is 90 degrees optimization
        state.optimize(Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble()));
        // got to velocity
        driveMotor.setControl(driveVeloRequest.withVelocity(state.speedMetersPerSecond));
        // go to position
        targetStateRotiations = state.angle.getRotations(); //just for testing
        steerMotor.setControl(steerMotionRequest.withPosition(targetStateRotiations));
    }

    /**
     * Turns the motors to an angle
     * @param setpoint in radians
     */
    public void turnToAngle(double setpoint) {
        //steerMotor.setVoltage(12d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), setpoint));
        steerMotor.setControl(steerMotionRequest.withPosition(setpoint));
    }


    /**
     * Get the position of the swerve module
     * @return gets with turning position and velocity
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningPosition());
    }

    /**
     * Whether the pid controller is at the set point.
     * @return whether pid is done
     */
    public boolean PIDisDone() {
        return turningPidController.atSetpoint();
    }

    public void setPValue(double newP) {
        turningPidController.setP(newP);
    }
    public void setDValue(double newD) {
        turningPidController.setD(newD);
    }
    public void setIValue(double newI) {
        turningPidController.setI(newI);
    }

    public double getPValue() {
        return turningPidController.getP();
    }
    public double getIValue() {
        return turningPidController.getI();
    }
    public double getDValue() {
        return turningPidController.getD();
    }

    /*
    public double getSteeringVoltage() { return steeringVoltage; }
    public double getDrivingVoltage() { return drivingVoltage; }
    public void setSteeringVoltage(double volt) { this.steeringVoltage = volt; }
    public void setDrivingVoltage(double volt){this.drivingVoltage=volt;}


     */

    /**
     * The string representation of the swerve module
     * @return "Swerve module side: " + sideNumber: int
     */
    /*public String toString() {
        return "Swerve module side: " + (side+1);
    }*/
    public int getRealSide(){
        int real = side + 1;
        return real;
    }
    public double getSteerRotations(){
        return steerMotor.getPosition().getValueAsDouble();
    }


    public double getSwerveSteeringVoltage() {
        return Constants.Swerve.maxSteerVoltage;
    }
    public void setSwerveSteeringVoltage(double lol) {
        Constants.Swerve.maxSteerVoltage = lol;
    }
    public double getSwerveDrivingVoltage() {
        return Constants.Swerve.maxDriveVoltage;
    }
    public void getSwerveDrivingVoltage(double lol) {
        Constants.Swerve.maxDriveVoltage = lol;
    }

    public double getModuleSteeringSupplyCurrent() { return steerMotor.getSupplyCurrent().getValueAsDouble();}
    public double getModuleDrivingSupplyCurrent() { return driveMotor.getSupplyCurrent().getValueAsDouble();}

    public void updateDrivePID() {
        driveConfigs.kA = drive_kA;
        driveConfigs.kV = drive_kV;
        driveConfigs.kP = drive_kP;
        driveConfigs.kI = drive_kI;
        driveConfigs.kD = drive_kD;
        driveMotor.getConfigurator().apply(driveConfigs);
    }
    public double getDrive_kA() {return drive_kA;}
    public double getDrive_kV() {return drive_kV;}
    public double getDrive_kP() {return drive_kP;}
    public double getDrive_kI() {return drive_kI;}
    public double getDrive_kD() {return drive_kD;}
    public void setDrive_kA(double value) {drive_kA = value;}
    public void setDrive_kV(double value) {drive_kV = value;}
    public void setDrive_kP(double value) {drive_kP = value;}
    public void setDrive_kI(double value) {drive_kI = value;}
    public void setDrive_kD(double value) {drive_kD = value;}

    public void updateSteerPID() {
        steerConfigs.kP = steer_kP;
        steerConfigs.kI = steer_kI;
        steerConfigs.kD = steer_kD;
        steerMotor.getConfigurator().apply(steerConfigs);
    }
    public double getSteer_kP() {return steer_kP;}
    public double getSteer_kI() {return steer_kI;}
    public double getSteer_kD() {return steer_kD;}
    public void setSteer_kP(double value) {steer_kP = value;}
    public void setSteer_kI(double value) {steer_kI = value;}
    public void setSteer_kD(double value) {steer_kD = value;}

    public double getTargetStateRotiations() {return targetStateRotiations;}

    /**
     * Builds the sendable for shuffleboard
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Swerve Module " + (getRealSide()));
            // builder.addDoubleProperty("Drive velocity", this::getDriveVelocity, null);
            builder.addDoubleProperty("Steer position", this::getSteerRotations, null);
            builder.addDoubleProperty("Turning Position Radians", this::getTurningPositionRadians, null);
            builder.addDoubleProperty("Drive position", this::getDrivePosition, null);
            builder.addDoubleProperty("Absolute encoder position", this::getAbsoluteEncoderRots, null);
            builder.addDoubleProperty("Constant Steering voltage", this::getSwerveSteeringVoltage, this::setSwerveSteeringVoltage);
            builder.addDoubleProperty("Constant Driving voltage", this::getSwerveDrivingVoltage, this::getSwerveDrivingVoltage);

            builder.addDoubleProperty("Steering Module Current Supply", this::getModuleSteeringSupplyCurrent, null);
            builder.addDoubleProperty("Driving Module Current Supply", this::getModuleDrivingSupplyCurrent, null);

            //    builder.addDoubleProperty("Steering Voltage", this::getSteeringVoltage, this::setSteeringVoltage);
            // builder.addDoubleProperty("Driving voltage", this:: getDrivingVoltage, this::setDrivingVoltage);
/*        builder.addDoubleProperty("Steer velocity", this::getTurningVelocity, null);
        builder.addDoubleProperty("Steer relative", this::getRelativePositionDegrees, null);
        */
            builder.addDoubleProperty("Swerve P " + getRealSide(), this::getPValue, this::setPValue);
            builder.addDoubleProperty("Swerve I " + getRealSide(), this::getIValue, this::setIValue);
            builder.addDoubleProperty("Swerve D " + getRealSide(), this::getDValue, this::setDValue);

            builder.addDoubleProperty("Velo Drive A " + getRealSide(), this::getDrive_kA, this::setDrive_kA);
            builder.addDoubleProperty("Velo Drive V " + getRealSide(), this::getDrive_kV, this::setDrive_kV);
            builder.addDoubleProperty("Velo Drive P " + getRealSide(), this::getDrive_kP, this::setDrive_kP);
            builder.addDoubleProperty("Velo Drive I " + getRealSide(), this::getDrive_kI, this::setDrive_kI);
            builder.addDoubleProperty("Velo Drive D " + getRealSide(), this::getDrive_kD, this::setDrive_kD);

            builder.addDoubleProperty("Motion Steer P " + getRealSide(), this::getSteer_kP, this::setSteer_kP);
            builder.addDoubleProperty("Motion Steer I " + getRealSide(), this::getSteer_kI, this::setSteer_kI);
            builder.addDoubleProperty("Motion Steer D " + getRealSide(), this::getSteer_kD, this::setSteer_kD);

            builder.addDoubleProperty("target State Rotations " + getRealSide(), this::getTargetStateRotiations, null);
        }
    }

    public double getSteerPositionWrapped() {
        return Math.IEEEremainder(getRelDegrees(), 360);
    }

    public double getRelativePositionDegrees() {
        return Math.toDegrees(getTurningPositionRadians());
    }

    public double getRelDegrees() {
        return Math.toDegrees(getTurningPositionRadians());
    }
}