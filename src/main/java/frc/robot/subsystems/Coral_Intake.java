// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;


public class Coral_Intake extends SubsystemBase {

    private final TalonFX outerMotor;
    private final TalonFX innerMotor;

  public Coral_Intake() {
    outerMotor = new TalonFX(Constants.Coral_Intake.CAN_CoralOuter);
    innerMotor = new TalonFX(Constants.Coral_Intake.CAN_CoralInner);
  }

  // setters for outer and inner intake motors
  public void setOuterSpeed(double speed) {
    outerMotor.set(speed);
  }

  public void setInnerSpeed(double speed) {
    innerMotor.set(speed);
  }

  public void stopOuter() {
    outerMotor.set(0);
  }

  public void stopInner() {
    innerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
