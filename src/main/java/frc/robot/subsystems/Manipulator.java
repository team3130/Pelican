// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonSRX manip;
  private final DigitalInput firstBeam;
  private final DigitalInput secondBeam;
  private boolean isIntaking = false;
  private boolean isOuttaking = false;

  private double manipSpeed = 0.6;
  public Manipulator() {
    manip = new TalonSRX(Constants.CAN.Manipulator);
    firstBeam = new DigitalInput(Constants.IDs.ManipulatorFirstBeam);
    secondBeam = new DigitalInput(Constants.IDs.ManipulatorSecondBeam);

    manip.configFactoryDefault();
    manip.setInverted(true);
  }

  public void runManip() {
    manip.set(ControlMode.PercentOutput, manipSpeed);
  }
  public void reverseManip() {
    manip.set(ControlMode.PercentOutput, -manipSpeed);
  }
  public void manipAtSpeed(double speed) {manip.set(ControlMode.PercentOutput, speed);}
  public void stopManip() {
    manip.set(ControlMode.PercentOutput, 0);
  }

  public boolean getFirstBeam() {return firstBeam.get();}
  public boolean getSecondBeam() {return secondBeam.get();}

  public double getManipSpeed() {return manipSpeed;}
  public void setManipSpeed(double value) {manipSpeed = value;}

  public boolean getIsIntaking() {return isIntaking;}
  public void setIsIntaking(boolean value) {isIntaking = value;}

  public boolean getIsOuttaking() {return isOuttaking;}
  public void setIsOuttaking(boolean value) {isOuttaking = value;}

  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Manipulator");

      builder.addBooleanProperty("Manip First Beam", this::getFirstBeam, null);
      builder.addBooleanProperty("Manip Second Beam", this::getSecondBeam, null);

      builder.addDoubleProperty("Manipulator Speed", this::getManipSpeed, this::setManipSpeed);

      builder.addBooleanProperty("Is Intaking", this::getIsIntaking, this::setIsIntaking);
      builder.addBooleanProperty("Is Outtaking", this::getIsOuttaking, this::setIsOuttaking);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
