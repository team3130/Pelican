package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Elevator_1 {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private double L1 = 1;
    private double L2 = 2;
    private double L3 = 3;
    private double L4 = 4;
    private double MaxHeight = 4.1;
    private double MinHeight = 0.1;
    private double home = 0;
    private double speed = 0.1;
    private double targetVelocity = 100;
    private double targetAcceleration = 50;

    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    private boolean atL1 = false;
    private boolean atL2 = false;
    private boolean atL3 = false;
    private boolean atL4 = false;
    private boolean atHome = true;
    private boolean atMaxHeight = false;
    private boolean atMinHeight = false;

    private TalonFXConfiguration config;
    private Slot0Configs slot0Configs;

    private double slot0kG = 1;  //Gravity keeps position in place
    private double slot0kP = 1;  //Proportion gives output/voltage proportional to error based on the error - immediate error
    private double slot0kI = 1;  //Integral calculates difference/error over time between desired and actual, and adjusts output/voltage accordingly - past error
    private double slot0kD = 1;  //Derivative finds the rate of change of the error and makes a prediction in order to dampen/counteract rapid changes (jerk) - future error

    public Elevator_1() {
        leftMotor = new TalonFX(Constants.CAN.ElevatorLeft);
        rightMotor = new TalonFX(Constants.CAN.ElevatorRight);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        slot0Configs = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static);
        slot0Configs.kG = slot0kG;
        slot0Configs.kP = slot0kP;
        slot0Configs.kI = slot0kI;
        slot0Configs.kD = slot0kD;

        config.MotionMagic.withMotionMagicCruiseVelocity(targetVelocity).withMotionMagicAcceleration(targetAcceleration);
        bottomLimitSwitch = new DigitalInput(Constants.IDs.ElevatorBottomLimitSwitch);
        topLimitSwitch = new DigitalInput(Constants.IDs.ElevatorTopLimitSwitch);
    }

    public void goUp(){
        leftMotor.set(speed);
        if (atMaxHeight){
            stop();
        }

    }
    public void goDown(){
        leftMotor.set(-speed);
        if (atMinHeight){
            stop();
        }
    }
    public void stop() {
        leftMotor.set(0);
    }
    public boolean brokeBottomLimitSwitch() {return !bottomLimitSwitch.get();}
    public boolean brokeTopLimitSwitch() {return topLimitSwitch.get();}
}
