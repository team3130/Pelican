// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import javax.lang.model.element.ElementVisitor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javafx.scene.paint.Color;

public class LEDs extends SubsystemBase {
  AddressableLED LEDs;
  private final int LEDLength = 37; //TODO UPDATE TO NEW LENGTH
  private final int pwmPort = 0;
  public LEDs() {
      //set pwmPort
      LED = new AddressableLED(0);

      //set strip length
      LEDBuffer = new AddressableLEDBuffer(37);
      LED.setLength(LEDBuffer.getLength);
      LED.start();

      //LEDs per Meter
      Distance kLedSpacing = Meters.of(1 / 37);

      //create color palate 

      //Solid colors
      LEDPattern red = LEDPattern.solid(Color.kRed);
      LEDPattern blue = LEDPattern.solid(Color.kBlue);
      LEDPattern green = LEDPattern.solid(Color.kGreen);
      LEDPattern yellow = LEDPattern.solid(Color.kYellow);
      LEDPattern orange = LEDPattern.solid(Color.kOrange);
      LEDPattern purple = LEDPattern.solid(Color.kPurple);

      //animated colors
      LEDPattern rainbow = LEDPattern.rainbow(255, 255);
      LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
      LEDPattern redAndBlue = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));

      LEDPattern timer = LEDPattern.progressMaskLayer(() -> DriverStation.getCurrentTimeSeconds / 135);
      LEDPattern elevatorDeltaL1 = LEDPattern.progressMaskLayer(() -> Elevator.getPosition / getL1);
      LEDPattern elevatorDeltaL2 = LEDPattern.progressMaskLayer(() -> Elevator.getPosition / getL2);
      LEDPattern elevatorDeltaL3 = LEDPattern.progressMaskLayer(() -> Elevator.getPosition / getL3);
      LEDPattern elevatorDeltaL4 = LEDPattern.progressMaskLayer(() -> Elevator.getPosition / getL4);
      LEDPattern elevatorDeltaHome = LEDPattern.progressMaskLayer(() -> Elevator.getPosition / getHome);
      LEDPattern elevatorDeltaMaxPos = LEDPattern.progressMaskLayer(() -> Elevator.getPosition / getMaxPosition);
      LEDPattern elevatorDeltaMinPos = LEDPattern.progressMaskLayer(() -> Elevator.getPosition / getMinPosition);

      setDefaultCommand(runPattern(timer)).withName("Off");
  }

  public setLEDsRed(){red.applyTo(LEDBuffer);}
  public setLEDsBlue(){blue.applyTo(LEDBuffer);}
  public setLEDsGreen(){green.applyTo(LEDBuffer);}
  public setLEDsYellow(){yellow.applyTo(LEDBuffer);}
  public setLEDsOrange(){orange.applyTo(LEDBuffer);}
  public setLEDsPurple(){purple.applyTo(LEDBuffer);}
  public setLEDsRainbow(){rainbow.applyTo(LEDBuffer);}
  public setLEDsScrollingRainbow(){scrollingRainbow.applyTo(LEDBuffer);}
  public setLEDsRedAndBlue(){redAndBlue.applyTo(LEDBuffer);}
  public setLEDsL1Delta(){elevatorDeltaL1.applyTo(LEDBuffer); purple.applyTo(LEDBuffer);}
  public setLEDsL2Delta(){elevatorDeltaL2.applyTo(LEDBuffer); purple.applyTo(LEDBuffer);}
  public setLEDsL3Delta(){elevatorDeltaL3.applyTo(LEDBuffer); purple.applyTo(LEDBuffer);}
  public setLEDsL4Delta(){elevatorDeltaL4.applyTo(LEDBuffer); purple.applyTo(LEDBuffer);}
  public setLEDsHomeDelta(){elevatorDeltaHome.applyTo(LEDBuffer); purple.applyTo(LEDBuffer);}
  public setLEDsMinDelta(){elevatorDeltaMinPos.applyTo(LEDBuffer); purple.applyTo(LEDBuffer);}
  public setLEDsMaxDelta(){elevatorDeltaMaxPos.applyTo(LEDBuffer); purple.applyTo(LEDBuffer);}

  public void setLEDstateElevator(){
    if(Elevator.setAtL4() && ((Elevator.getL4Elevator) Elevator.getPosition == Elevator.getL4)){
      LEDs.setLEDsL4Delta();
    }
    else if (Elevator.setAtL3() && Elevator.getPosition == Elevator.getL3){
      LEDs.setLEDsL3Delta();
    }
    else if (Elevator.setAtL2() && Elevator.getPosition == Elevator.getL2){
      LEDs.setLEDsL2Delta();
    }
    else if (Elevator.setAtL1() && Elevator.getPosition == Elevator.getL1){
      LEDs.setLEDsL1Delta();
    }
    else if (Elevator.setAtHome() && Elevator.getPosition == Elevator.getHome){
      LEDs.setLEDsHomeDelta();
    }
    else if (Elevator.setAtMaxPosition() && Elevator.getPosition == Elevator.getMaxPosition){
      LEDs.setLEDsMaxDelta();
    }
    else if (Elevator.setAtMinPosition() && Elevator.getPosition == Elevator.getMinPosition){
      LEDs.setLEDsMinDelta();
    }
  }

  public void setLEDstateManipulator(){
    if (Manipulator.getFirstBeam && !Manipulator.getSecondBeam){
      LEDs.setLEDsOrange();
    }
    else if (Manipulator.getFirstBeam && Manipulator.getSecondBeam){
      LEDs.setLEDsBlue();
    }
    else if (!Manipulator.getFirstBeam && Manipulator.getSecondBeam){
      LEDs.setLEDsGreen();
    }
  }
  @Override
  public void periodic() {
    LED.setData(LEDBuffer);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
