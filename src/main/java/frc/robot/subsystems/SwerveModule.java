package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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
        steerMotor.getConfigurator().apply(steerTalonConfig); // config factory default
        steerMotor.setNeutralMode(NeutralModeValue.Brake); // Brake mode
        //steerMotor.setInverted(true);

        driveTalonConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive); //set config to counter-clockwise
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        //driveMotor.setInverted(false);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
        turningPidController.setTolerance(0.0025, 0.05); // at position tolerance

        absoluteEncoderOffset = Constants.SwerveEncoderOffsets.kCANCoderOffsets[side];
        this.side =side;

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

    public double getAbsoluteEncoderRotation() {
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
        if (side == 0 || side == 1) {
            steerMotor.setPosition((getAbsoluteEncoderRots() - absoluteEncoderOffset) * Constants.SwerveConversions.frontSteerGearRatio);
        } else {
            steerMotor.setPosition((getAbsoluteEncoderRots() - absoluteEncoderOffset) * Constants.SwerveConversions.backSteerGearRatio);
        }
        //steerMotor.setPosition((getAbsoluteEncoderRad() - absoluteEncoderOffset) / Constants.Conversions.SteerRotToRads);
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
        state = SwerveModuleState.optimize(state, getState().angle);
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
        state = SwerveModuleState.optimize(state, getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage((10d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(4d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    /**
     * Turns the motors to an angle
     * @param setpoint in radians
     */
    public void turnToAngle(double setpoint) {
        //steerMotor.setVoltage(12d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), setpoint));
        steerMotor.setVoltage(12d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), Math.toRadians(setpoint)));
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