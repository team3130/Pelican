// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//written in phoenix 5 since TalonSRX doesn't exist in phoenix 6
public class CoralIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonSRX intake;
  private final Servo actuation1;
  private final Servo actuation2;
  private double intakeSpeed = 0.5;

  public CoralIntake() {
    intake = new TalonSRX(Constants.CAN.CoralIntake);
    actuation1 = new Servo(Constants.IDs.CoralIntakeActuation1);
    actuation2 = new Servo(Constants.IDs.CoralIntakeActuation2);

    intake.configFactoryDefault();
    intake.setInverted(false);
  }

  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void runIntake() {
    intake.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void runOuttake() {
    intake.set(ControlMode.PercentOutput, -intakeSpeed);
  }

  public void deactuate1() {actuation1.setPosition(0.4);}
  public void deactuate2() {actuation2.setPosition(0.4);}

  public void actuate1() {
    actuation1.setPosition(1.0);
  }
  public void actuate2() {actuation2.setPosition(1.0);}

  public void gotoSetpoint1(double setpoint) {actuation1.setPosition(setpoint);}
  public void gotoSetpoint2(double setpoint) {actuation2.setPosition(setpoint);}

  public double getIntakeSpeed() {return intakeSpeed;}
  public void setIntakeSpeed(double value) {intakeSpeed = value;}

  /**
   * Initializes the data we send on shuffleboard
   * Calls the default init sendable for Subsystem Bases
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Coral Intake");

      builder.addDoubleProperty("Intake Speed", this::getIntakeSpeed, this::setIntakeSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
