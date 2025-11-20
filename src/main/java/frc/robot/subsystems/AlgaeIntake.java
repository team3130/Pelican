// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX intake;
  private double intakeSpeed = 1.0;
  private boolean algaeMode = false;

  private final PositionDutyCycle voltRequest;
  private TalonFXConfiguration config;
  private Slot0Configs slot0Configs;
  private double slot0kP = 0.1;
  private double slot0kI = 0;
  private double slot0kD = 0;

  public AlgaeIntake() {
    intake = new TalonFX(Constants.CAN.AlgaeIntake);

    voltRequest = new PositionDutyCycle(0);
    slot0Configs = new Slot0Configs();
    slot0Configs.kP = slot0kP;
    slot0Configs.kI = slot0kI;
    slot0Configs.kD = slot0kD;
    config = new TalonFXConfiguration();
    config.Slot0 = slot0Configs;
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);

    intake.getConfigurator().apply(config);
  }

  public void runIntake() {
    intake.set(intakeSpeed);
  }
  public void runOuttake() {
    intake.set(-intakeSpeed);
  }
  public void stopIntake() {
    intake.set(0);
  }
  public void intakeSetpoint(double setpoint) {
    intake.setControl(voltRequest.withPosition(setpoint));
  }

  public double getIntakeSpeed() {return intakeSpeed;}
  public void setIntakeSpeed(double value) {intakeSpeed = value;}

  public double getPosition() {return intake.getPosition().getValueAsDouble();}

  public boolean getAlgaeMode() {return algaeMode;}
  public void setAlgaeMode(boolean value) {algaeMode = value;}

  /**
   * Initializes the data we send on shuffleboard
   * Calls the default init sendable for Subsystem Bases
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Algae Intake");

      SmartDashboard.putData(intake);
      builder.addDoubleProperty("Intake Speed", this::getIntakeSpeed, this::setIntakeSpeed);
      builder.addDoubleProperty("Algae Position", this::getPosition, null);
      builder.addBooleanProperty("Algae Mode", this::getAlgaeMode, this::setAlgaeMode);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
