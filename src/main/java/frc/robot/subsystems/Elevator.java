// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DigitalInput limitSwitch;
  private final CANcoder encoder;

  private double home = 0;
  private double minPosition = 0;
  private double L1 = 0;
  private double L2 = 0;
  private double L3 = 0;
  private double L4 = 0;
  private double maxPosition = 0;

  private final MotionMagicDutyCycle voltRequest0;
  private Slot0Configs slot0Configs;
  private double slot0kG = 0;
  private double slot0kP = 0;
  private double slot0kI = 0;
  private double slot0kD = 0;

  private boolean zeroed = false;
  private boolean atHome = false;
  private boolean atMinPosition = false;
  private boolean atL1 = false;
  private boolean atL2 = false;
  private boolean atL3 = false;
  private boolean atL4 = false;
  private boolean atMaxPosition = false;
  public Elevator() {
    leftMotor = new TalonFX(Constants.CAN.ElevatorLeft);
    rightMotor = new TalonFX(Constants.CAN.ElevatorRight);
    limitSwitch = new DigitalInput(Constants.IDs.ElevatorLimitSwitch);
    encoder = new CANcoder(Constants.IDs.ElevatorEncoder);

    encoder.getConfigurator().apply(new CANcoderConfiguration());

    rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

    leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightMotor.getConfigurator().apply(new TalonFXConfiguration());

    leftMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    rightMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    leftMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(6));
    rightMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(6));

    voltRequest0 = new MotionMagicDutyCycle(0);
    slot0Configs = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static);
    slot0Configs.kG = slot0kG;
    slot0Configs.kP = slot0kP;
    slot0Configs.kI = slot0kI;
    slot0Configs.kD = slot0kD;

    leftMotor.getConfigurator().apply(slot0Configs);
  }

  public void stop() {
    leftMotor.setControl(voltRequest0.withPosition(0));
  }

  public void goDown() {
    leftMotor.set(-0.2);
  }

  public void goUp() {
    leftMotor.set(0.2);
  }

  public void goToSetpoint(double setpoint) {
    leftMotor.setControl(voltRequest0.withPosition(setpoint));
  }

  public void goToHome() {
    goDown();
    if(brokeLimitSwitch()) {
      setPosition(0);
      setZeroed(true);
    }
    setAtHome(true);
    setAtMinPosition(false);
    setAtL1(false);
    setAtL2(false);
    setAtL3(false);
    setAtL4(false);
  }

  public void goToMinPosition() {
    goToSetpoint(minPosition);
    setAtHome(false);
    setAtMinPosition(true);
    setAtL1(false);
    setAtL2(false);
    setAtL3(false);
    setAtL4(false);
  }


  public void goToL1() {
    goToSetpoint(L1);
    setAtHome(false);
    setAtMinPosition(false);
    setAtL1(true);
    setAtL2(false);
    setAtL3(false);
    setAtL4(false);
  }
  public void goToL2() {
    goToSetpoint(L2);
    setAtHome(false);
    setAtMinPosition(false);
    setAtL1(false);
    setAtL2(true);
    setAtL3(false);
    setAtL4(false);
  }
  public void goToL3() {
    goToSetpoint(L3);
    setAtHome(false);
    setAtMinPosition(false);
    setAtL1(false);
    setAtL2(false);
    setAtL3(true);
    setAtL4(false);
  }
  public void goToL4() {
    goToSetpoint(L4);
    setAtHome(false);
    setAtMinPosition(false);
    setAtL1(false);
    setAtL2(false);
    setAtL3(false);
    setAtL4(true);
  }

  public double getPosition() {return encoder.getPosition().getValueAsDouble();}
  public void setPosition(double value) {encoder.setPosition(value);}

  public boolean brokeLimitSwitch() {return limitSwitch.get();}

  public boolean isZeroed() {return zeroed;}
  public boolean isAtHome() {return atHome;}
  public boolean isAtMinPosition() {return atMinPosition;}
  public boolean isAtL1() {return atL1;}
  public boolean isAtL2() {return atL2;}
  public boolean isAtL3() {return atL3;}
  public boolean isAtL4() {return atL4;}
  public boolean isAtMaxPosition() {return atMaxPosition;}
  public void setZeroed(boolean value) {zeroed = value;}
  public void setAtHome(boolean value) {
    atHome = value;}
  public void setAtMinPosition(boolean value) {
    atMinPosition = value;}
  public void setAtL1(boolean value) {
    atL1 = value;}
  public void setAtL2(boolean value) {
    atL2 = value;}
  public void setAtL3(boolean value) {
    atL3 = value;}
  public void setAtL4(boolean value) {
    atL4 = value;}
  public void setAtMaxPosition(boolean value) {
    atMaxPosition = value;}

  public double getHome() {return home;}
  public double getMinPosition() {return minPosition;}
  public double getL1() {return L1;}
  public double getL2() {return L2;}
  public double getL3() {return L3;}
  public double getL4() {return L4;}
  public double getMaxPosition() {return maxPosition;}
  public void setHome(double value) {home = value;}
  public void setMinPosition(double value) {minPosition = value;}
  public void setL1(double value) {L1 = value;}
  public void setL2(double value) {L2 = value;}
  public void setL3(double value) {L3 = value;}
  public void setL4(double value) {L4 = value;}
  public void setMaxPosition(double value) {maxPosition = value;}

  public double getSlot0kG() {return slot0kG;}
  public double getSlot0kP() {return slot0kP;}
  public double getSlot0kI() {return slot0kI;}
  public double getSlot0kD() {return slot0kD;}
  public void setSlot0kG(double value) {slot0kG = value;}
  public void setSlot0kP(double value) {slot0kG = value;}
  public void setSlot0kI(double value) {slot0kG = value;}
  public void setSlot0kD(double value) {slot0kG = value;}


  /**
   * Initializes the data we send on shuffleboard
   * Calls the default init sendable for Subsystem Bases
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Elevator");

      builder.addDoubleProperty("Position", this::getPosition, null);
      builder.addDoubleProperty("Min Position", this::getMinPosition, this::setMinPosition);
      builder.addDoubleProperty("Home", this::getHome, this::setHome);
      builder.addDoubleProperty("L1", this::getL1, this::setL1);
      builder.addDoubleProperty("L2", this::getL2, this::setL2);
      builder.addDoubleProperty("L3", this::getL3, this::setL3);
      builder.addDoubleProperty("L4", this::getL4, this::setL4);
      builder.addDoubleProperty("Max Position", this::getMaxPosition, this::setMaxPosition);

      builder.addDoubleProperty("Slot 0 kG", this::getSlot0kG, this::setSlot0kG);
      builder.addDoubleProperty("Slot 0 kP", this::getSlot0kP, this::setSlot0kP);
      builder.addDoubleProperty("Slot 0 kI", this::getSlot0kI, this::setSlot0kI);
      builder.addDoubleProperty("Slot 0 kD", this::getSlot0kD, this::setSlot0kD);

      builder.addBooleanProperty("Broke Limit", this::brokeLimitSwitch, null);
      builder.addBooleanProperty("At Zeroed", this::isZeroed, this::setZeroed);
      builder.addBooleanProperty("At Home", this::isAtHome, this::setAtHome);
      builder.addBooleanProperty("At Min Position", this::isAtMinPosition, this::setAtMinPosition);
      builder.addBooleanProperty("At L1", this::isAtL1, this::setAtL1);
      builder.addBooleanProperty("At L2", this::isAtL2, this::setAtL2);
      builder.addBooleanProperty("At L3", this::isAtL3, this::setAtL3);
      builder.addBooleanProperty("At L4", this::isAtL4, this::setAtL4);
      builder.addBooleanProperty("At Max Position", this::isAtMaxPosition, this::setAtMaxPosition);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
