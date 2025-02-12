// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX climber;
  private final DigitalInput homeLimit;
  private final DigitalInput extendedLimit;
  private final MotionMagicVoltage voltRequest;
  private Slot0Configs slot0Configs;
  private double kG = 0;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double climberSpeed = 0.5;

  private double homePos = 0;
  private double extendedPos = 0;

  private boolean isZeroed = false;
  public Climber() {
    climber = new TalonFX(Constants.CAN.Climber);
    homeLimit = new DigitalInput(Constants.IDs.ClimberLimitSwitchHome);
    extendedLimit = new DigitalInput(Constants.IDs.ClimberLimitSwitchExtended);

    climber.getConfigurator().apply(new TalonFXConfiguration());
    climber.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    voltRequest = new MotionMagicVoltage(0);
    //slot0Configs = new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine);
    slot0Configs = new Slot0Configs();
    slot0Configs.kG = kG;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
  }

  public void goToSetpoint(double setpoint) {
    climber.setControl(voltRequest.withPosition(setpoint));
  }

  public void goToHome() {
    goToSetpoint(homePos);
  }
  public void goToExtended() {
    goToSetpoint(extendedPos);
  }

  public void climbUp() {
    climber.set(climberSpeed);
  }
  public void climbDown() {
    climber.set(-climberSpeed);
  }
  public void stopClimb() {
    climber.set(0);
  }

  public double getPosition() {return climber.getPosition().getValueAsDouble();}

  public boolean brokeHomeLimit() {return homeLimit.get();}
  public boolean brokeExtendedLimit() {return extendedLimit.get();}

  public double getClimberSpeed() {return climberSpeed;}
  public void setClimberSpeed(double value) {climberSpeed = value;}

  public double getHomePos() {return homePos;}
  public void setHomePos(double value) {homePos = value;}

  public double getExtendedPos() {return extendedPos;}
  public void setExtendedPos(double value) {extendedPos = value;}

  public boolean isZeroed() {return isZeroed;}
  public void setZeroed(boolean value) {isZeroed = value;}

  public double getkG() {return kG;}
  public double getkP() {return kP;}
  public double getkI() {return kI;}
  public double getkD() {return kD;}
  public void setkG(double value) {kG = value;}
  public void setkP(double value) {kP = value;}
  public void setkI(double value) {kI = value;}
  public void setkD(double value) {kD = value;}

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Climber");

      builder.addBooleanProperty("Broke Home Limit", this::brokeHomeLimit, null);
      builder.addBooleanProperty("Broke Extended Limit", this::brokeExtendedLimit, null);
      builder.addBooleanProperty("Is Zeroed", this::isZeroed, this::setZeroed);

      builder.addDoubleProperty("Position", this::getPosition, null);
      builder.addDoubleProperty("Climber Speed", this::getClimberSpeed, this::setClimberSpeed);
      builder.addDoubleProperty("Home Position", this::getHomePos, this::setHomePos);
      builder.addDoubleProperty("Extended Position", this::getExtendedPos, this::setExtendedPos);

      builder.addDoubleProperty("kG", this::getkG, this::setkG);
      builder.addDoubleProperty("kP", this::getkP, this::setkP);
      builder.addDoubleProperty("kI", this::getkI, this::setkI);
      builder.addDoubleProperty("kD", this::getkD, this::setkD);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
