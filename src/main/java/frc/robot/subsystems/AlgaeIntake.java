// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonSRX intake;
  private final TalonSRX actuation;

  private double intakeSpeed = 0.5;
  private double actuationSpeed = 0.5;
  public AlgaeIntake() {
    intake = new TalonSRX(Constants.CAN.AlgaeIntake);
    actuation = new TalonSRX(Constants.CAN.AlgaeIntakeActuation);

    intake.configFactoryDefault();
    intake.setInverted(false);

    actuation.configFactoryDefault();
    actuation.setInverted(false);
  }

  public void runIntake() {
    intake.set(ControlMode.PercentOutput, intakeSpeed);
  }
  public void runOuttake() {
    intake.set(ControlMode.PercentOutput, -intakeSpeed);
  }
  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void actuateIntake() {
    actuation.set(ControlMode.PercentOutput, actuationSpeed);
  }
  public void deactuateIntake() {
    actuation.set(ControlMode.PercentOutput, -actuationSpeed);
  }
  public void stopActuation() {actuation.set(ControlMode.PercentOutput, 0);}

  public double getIntakeSpeed() {return intakeSpeed;}
  public void setIntakeSpeed(double value) {intakeSpeed = value;}

  public double getActuationSpeed() {return actuationSpeed;}
  public void setActuationSpeed(double value) {actuationSpeed = value;}

  /**
   * Initializes the data we send on shuffleboard
   * Calls the default init sendable for Subsystem Bases
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Algae Intake");

      builder.addDoubleProperty("Intake Speed", this::getIntakeSpeed, this::setIntakeSpeed);
      builder.addDoubleProperty("Actuation Speed", this::getActuationSpeed, this::setActuationSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
