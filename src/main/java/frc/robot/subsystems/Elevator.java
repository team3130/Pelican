// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final DigitalInput elevatorLimit;
  private final WPI_TalonFX elevatorLeadingMotor;
  private final WPI_TalonFX elevatorFollowingMotor;

  private int encoderMaxTicks = 10000;
  private int L4setpoint = 9000;
  private int L3setpoint = 8000;
  private int L2setpoint = 7000;
  private int L1setpoint = 6000;
  private int homeSetpoint = 1000;
  private PIDController elevatorController;
  private double P;
  private double I;
  private double D;
  private boolean hasZeroed = false;
  private boolean isL4 = false;
  private boolean isL3 = false;
  private boolean isL2 = false;
  private boolean isL1 = false;
  private boolean isHome = false;
  private double voltage = 3;
  private double elevatorSpeed = .4;

 
  public Elevator() {
    elevatorLimit = new DigitalInput(Constants.IDs.elevatorLimitDIO);

    elevatorLeadingMotor = new WPI_TalonFX(Constants.CAN.elevatorLeadingMotor);
    elevatorFollowingMotor = new WPI_TalonFX(Constants.CAN.elevatorFollowingMotor);

    elevatorLeadingMotor.configFactoryDefault();
    elevatorFollowingMotor.configFactoryDefault();

    elevatorLeadingMotor.setInverted(false);
    elevatorFollowingMotor.setInverted(false);
    
    elevatorLeadingMotor.configVoltageCompSaturation(voltage);
    elevatorFollowingMotor.configVoltageCompSaturation(voltage);

    elevatorLeadingMotor.enableVoltageCompensation(true);
    elevatorFollowingMotor.enableVoltageCompensation(true);

    elevatorLeadingMotor.setNeutralMode(NeutralMode.brake);
    elevatorFollowingMotor.setNeutralMode(NeutralMode.brake);
    
    elevatorMotors = new MotorControllerGroup(elevatorLeadingMotor, elevatorFollowingMotor);

    elevatorPID = new PIDController(P, I, D);
  }

  public boolean getAtHome(){
    return isHome;
  }

  public void setAtHome(boolean home){
    isHome = home;
  }

  public boolean getAtL1(){
    return isL1;
  }

  public void setAtL1(boolean L1){
    isL1 = L1;
  }

  public boolean getAtL2(){
    return isL2;
  }

  public void setAtL2(boolean L2){
    isL2 = L2;
  }

  public boolean getAtL3(){
    return isL3;
  }
  
  public void setAtL3(boolean L3){
    isL3 = L3;
  }

  public boolean getAtL4(){
    return isL4;
  }

  public void setAtL4(boolean L4){
    isL4 = L4;
  }
  public boolean getHasZeroed(){
    return hasZeroed;
  }
  public void setHasZeroedTrue(){
    hasZeroed = true;
  }
  public int getHomeSetpoint(){
    return homeSetpoint;
  }
  public void setHomeSetpoint(int home){
    homeSetpoint = home;
  }
  public int getL1Setpoint(){
    return L1setpoint;
  }
  public void setL1Setpoint(int L1){
    L1setpoint = L1;
  }
  public int getL2Setpoint(){
    return L2setpoint;
  }
  public void setL2Setpoint(int L2){
    L2setpoint = L2;
  }
  public int getL3Setpoint(){
    return L3setpoint;
  }
  public void setL3Setpoint(int L3){
    L3setpoint = L3;
  }
  public int getL4Setpoint(){
    return L4setpoint;
  }
  public void setL4Setpoint(int L4){
    L4setpoint = L4;
  }
  public boolean isAtSetpoint(){
    return elevatorController.atSetpoint();
  }
  public boolean isAtSetpointWithDeadband(){
    return ((abs(elevatorController.getSetpoint() - getElevatorEncoderPosition())) <= Constants.Elevatro.deadband);
  }
  public double getElevatorEncoderPosition(){
    return -elevatorLeadingMotor.getSelectedSensorPosition();
  }
  public int getEncoderMax(){
    return encoderMaxTicks;
  }
  public void setEncoderMax(int max){
    encoderMaxTicks = max;
  }
  public void elevatorStop(){
    elevatorMotors.set(ControlMode.PercentOutput, 0);
  }
  public boolean getLimitSwitch(){
    return !elevatorLimit.get();
  }
  public double getElevatorSpeed() {
    return elevatorSpeed;
  }
  public void setElevatorSpeed(double speed){
    elevatorSpeed = speed;
  }
  public void ElevatorUp(){
    elevatorMotors.set(ControlMode.PercentOutput, elevatorSpeed);
  }
  public void ElevatorDown(){
    elevatorMotors.set(ControlMode.PercentOutput, -elevatorSpeed);
  }
  public double getP(){
    return P;
  }
  public double getI(){
    return I;
  }
  public double getD(){
    return D;
  }
  public void setP(double newP){
     P = newP;
  }
  public void setI(double newI){
    I = newI;
  }
  public void setD(double newD){
    D = newD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initSendable(SendableBuilder builder){
    if (Constants.debugMode) {
      builder.setSmartDashboardType("elevator");
      builder.addDoublePropty("ElevatorSpeed", this::getElevatorSpeed, this::setElevatorSpeed);
      buulder.addBooleanProperty("Limit Switch", this::getLimitSwitch, null);
      builder.addIntegerProperty("Encoder Max", this::getEncoderMax, this::setEncoderMax);
      builder.addDoublePropty("Encoder Position", this::getElevatorEncoderPosition, null);
      builder.addIntegerProperty("L4 setpoint", this::getL4Setpoint, this::setL4Setpoint);
      builder.addIntegerProperty("L3 setpoint", this::getL3Setpoint, this::setL3Setpoint);
      builder.addIntegerProperty("L2 setpoint", this::getL2Setpoint, this::setL2Setpoint);
      builder.addIntegerProperty("L1 setpoint", this::getL1Setpoint, this::setL1Setpoint);
      builder.addIntegerProperty("Home setpoint", this::getHomeSetpoint, this::setHomeSetpoint);
      builder.addBooleanProperty("Is at Setpoint", this::isAtSetpoint, null);
      builder.addBooleanProperty("Has Zeroed", this::getHasZeroed, null);
      builder.addDoubleProperty("p", this::getP, this::setP);
      builder.addDoubleProperty("i", this::getI, this::setI);
      builder.addDoubleProperty("d", this::getD, this::setD);
      
      builder.addBooleanProperty("is at Home", this::getAtHome, this::setAtHome);
      builder.addBooleanProperty("is at L1", this::getAtL1, this::setAtL1);
      builder.addBooleanProperty("is at L2", this::getAtL2, this::setAtL2);
      builder.addBooleanProperty("is at L3", this::getAtL3, this::setAtL3);
      builder.addBooleanProperty("is at L4", this::getAtL4, this::setAtL4);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
