// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;


public class Coral_Intake extends SubsystemBase {

    private final TalonFX intakeMotor;

  public Coral_Intake() {
     intakeMotor = new TalonFX(Constants.Coral_Intake.CAN_CoralMotor);
     intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  // setters for outer and inner intake motors
  public void setSpeed(double speed) {intakeMotor.set(speed);}


  public void stop() {
    intakeMotor.set(0);
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
