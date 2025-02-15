// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonSRX intake;
  private final TalonSRX actuation;
  private final DigitalInput limitSwitch;

  private double intakeSpeed = 0.5;
  private double actuationSpeed = 0.2;
  private double setpoint = 180;

  private boolean actuated = false;
  public AlgaeIntake() {
    intake = new TalonSRX(Constants.CAN.AlgaeIntake);
    actuation = new TalonSRX(Constants.CAN.AlgaeIntakeActuation);
    limitSwitch = new DigitalInput(Constants.IDs.AlgaeIntakeLimitSwitch);

    intake.configFactoryDefault();
    intake.setInverted(false);

    actuation.configFactoryDefault();
    actuation.setInverted(false);
    actuation.setNeutralMode(NeutralMode.Brake);
    actuation.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //actuation.configVoltageCompSaturation(6);
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

  public double getActuationPosition() {return actuation.getSelectedSensorPosition(0);}
  public void setActuationPosition(double value) {actuation.setSelectedSensorPosition(0);}

  public double getSetpoint() {return setpoint;}
  public void setSetpoint(double value) {setpoint = value;}

  public boolean getActuated() {return actuated;}
  public void setActuated(boolean value) {actuated = value;}

  public boolean getSwitch() {return !limitSwitch.get();}

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
      builder.addDoubleProperty("Actuation Position", this::getActuationPosition, this::setActuationPosition);
      builder.addDoubleProperty("Actuation Setpoint", this::getSetpoint, this::setSetpoint);

      builder.addBooleanProperty("Actuated", this::getActuated, this::setActuated);
      builder.addBooleanProperty("Limit Switch", this::getSwitch, null);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
