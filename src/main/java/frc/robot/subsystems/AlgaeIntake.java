// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  private final WPI_TalonSRX algaeExtensionMotor;
  private final WPI_TalonSRX algaeSpinMotor;
  /** Creates a new ExampleSubsystem. */
  public AlgaeIntake() {
    algaeExtensionMotor = new WPI_TalonSRX(Constants.CAN.algaeExtensionMotor);
    algaeSpinMotor = new WPI_TalonSRX(Constants.CAN.algaeSpinMotor);

    algaeExtensionMotor.configFactoryDefault();
    algaeSpinMotor.configFactoryDefault();

    algaeExtensionMotor.setNeutralMode(NeutralMode.Brake);
    algaeSpinMotor.setNeutralMode(NeutralMode.Brake);

    algaeExtensionMotor.enableVoltageCompensation(true);
    algaeExtensionMotor.configVoltageCompSaturation(Constants.Voltage.algaeExtensionVoltage);

    algaeSpinMotor.enableVoltageCompensation(true);
    algaeSpinMotor.configVoltageCompSaturation(Constants.Voltage.algaeSpinVoltage);

    algaeExtensionMotor.setInverted(false);
    algaeSpinMotor.setInverted(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  
  public void extendAlgaeIntake(){
    algaeExtensionMotor.set(.5);
  }
  
  public void stopExtension(){
    algaeExtensionMotor.stopMotor();
  }

  public void retractAlgaeIntake(){
    algaeExtensionMotor.set(-.5);
  }

  public void spinAlgaeIntake(){
    algaeSpinMotor.set(.5);
  }

  public void stopAlgaeIntake(){
    algaeSpinMotor.stopMotor();
  }

  public void reverseAlgaeIntake(){
    algaeSpinMotor.set(-.5);
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
