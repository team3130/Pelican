// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static java.awt.Color.*;
import frc.robot.subsystems.Climber;

public class LEDs extends SubsystemBase{
  private AddressableLED LED;
  private AddressableLEDBuffer LEDBuffer;
  private Elevator elevator;
  private Manipulator manip;
  private Climber climber;
  private final int LEDLength = 129; //should be the correct length as of 3/19/25
  private final int pwmPort = 2;
  private boolean completeClimb = false;

  //LEDs per Meter
  Distance kLedSpacing = Meters.of((double) 1 / LEDLength);

  //create color palate
  //Solid colors
  LEDPattern red = LEDPattern.solid(Color.kRed);
  LEDPattern blue = LEDPattern.solid(Color.kBlue);
  LEDPattern green = LEDPattern.solid(Color.kGreen);
  LEDPattern yellow = LEDPattern.solid(Color.kYellow);
  LEDPattern gold = LEDPattern.solid(Color.kGold);
  LEDPattern orange = LEDPattern.solid(Color.kOrange);
  LEDPattern purple = LEDPattern.solid(Color.kPurple);
  LEDPattern manualYellow = LEDPattern.solid(new Color(255, 135, 0));

  //animated colors
  LEDPattern rainbow = LEDPattern.rainbow(255, 255);
  LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), kLedSpacing);
  LEDPattern redAndBlue = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));

  LEDPattern timer = LEDPattern.progressMaskLayer(() -> DriverStation.getMatchTime() / 135);
  LEDPattern elevatorDeltaL1 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL1()));
  LEDPattern elevatorDeltaL2 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL2()));
  LEDPattern elevatorDeltaL3 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL3()));
  LEDPattern elevatorDeltaL4 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL4()));
  LEDPattern elevatorDeltaHome = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getHome()));
  LEDPattern elevatorDeltaMaxPos = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getMaxPosition()));
  LEDPattern elevatorDeltaMinPos = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getMinPosition()));
  public LEDs(Elevator elevator, Manipulator manip, Climber climber) {
      this.elevator = elevator;
      this.manip = manip;
      this.climber = climber;
      
      //set pwmPort
      LED = new AddressableLED(pwmPort);

      //set strip length
      LEDBuffer = new AddressableLEDBuffer(LEDLength);
      LED.setLength(LEDBuffer.getLength());
      LED.stop();
      manualYellow.applyTo(LEDBuffer);
      LED.setData(LEDBuffer);
      LED.start();
  }
  

  //public void setLEDsRed(){red.applyTo(LEDBuffer);}
  //public void setLEDsBlue(){blue.applyTo(LEDBuffer);}
  //public void setLEDsGreen(){green.applyTo(LEDBuffer);}
  //public void setLEDsYellow(){yellow.applyTo(LEDBuffer);}
  //public void setLEDsOrange(){orange.applyTo(LEDBuffer);}
  //public void setLEDsPurple(){purple.applyTo(LEDBuffer);}
  //public void setLEDsRainbow(){rainbow.applyTo(LEDBuffer);}
  //public void setLEDsScrollingRainbow(){scrollingRainbow.applyTo(LEDBuffer);}
  //public void setLEDsRedAndBlue(){redAndBlue.applyTo(LEDBuffer);}

  // elevator LED colors
  /*public void setLEDsL1Delta(){
    elevatorDeltaL1.applyTo(LEDBuffer);
    purple.applyTo(LEDBuffer);
  }
  public void setLEDsL2Delta(){
    elevatorDeltaL2.applyTo(LEDBuffer);
    purple.applyTo(LEDBuffer);
  }
  public void setLEDsL3Delta(){
    elevatorDeltaL3.applyTo(LEDBuffer);
    purple.applyTo(LEDBuffer);
  }
  public void setLEDsL4Delta(){
    elevatorDeltaL4.applyTo(LEDBuffer);
    purple.applyTo(LEDBuffer);
  }
  public void setLEDsHomeDelta(){
    elevatorDeltaHome.applyTo(LEDBuffer);
    purple.applyTo(LEDBuffer);
  }
  public void setLEDsMinDelta(){
    elevatorDeltaMinPos.applyTo(LEDBuffer);
    purple.applyTo(LEDBuffer);
  }
  public void setLEDsMaxDelta(){
    elevatorDeltaMaxPos.applyTo(LEDBuffer);
    purple.applyTo(LEDBuffer);
  }*/

  /*public void setLEDstateElevator(){
    if(elevator.isAtL4()){
      setLEDsL4Delta();
    }
    else if (elevator.isAtL3()){
      setLEDsL3Delta();
    }
    else if (elevator.isAtL2()){
      setLEDsL2Delta();
    }
    else if (elevator.isAtL1()){
      setLEDsL1Delta();
    }
    else if (elevator.isAtHome()){
      setLEDsHomeDelta();
    }
    else if (elevator.isAtMaxPosition()){
      setLEDsMaxDelta();
    }
    else if (elevator.isAtMinPosition()){
      setLEDsMinDelta();
    }
  }*/

  /*public void setLEDstateManipulator(){
    if (manip.getFirstBeam() && manip.getSecondBeam()){
      setLEDsYellow();
    }
    else if (!manip.getFirstBeam() && !manip.getSecondBeam()){
      setLEDsGreen();
    }
  }*/

  //Added climber LED logic, 
  //when extending climber show orange,
  //when at full extension show red and blue, 
  //when retracting show rainbow, 
  //when finished show scrolling rainbow
  /*public void setLEDstateClimber(){
    if (climber.getHomePos() < climber.getPosition() && climber.getPosition() < climber.getExtendedPos() &&  climber.brokeExtendedLimit()){ //if climber is not at a max position but it has hit the maximum previously, the climber is currently climbing
      setLEDsRainbow();
    }
    else if (climber.brokeHomeLimit() && completeClimb){ //if climber is a min position, and was previously at full extension, climb is completed
      setLEDsScrollingRainbow();
    }
    else if (climber.getHomePos() < climber.getPosition() && climber.getPosition() < climber.getExtendedPos() && !climber.brokeExtendedLimit()){ //if climber is not at either extrema and has not hit the upper limit, it is coming out of robot frame 
      setLEDsOrange();
    }
    //this is placed last because it should only trigger at one particular point, and if placed earlier it would unintentionally trigger even after the desired point
    else if (climber.brokeExtendedLimit()){//if at maximum position, climber is ready to climb
      setLEDsRedAndBlue();
      completeClimb = true;
    }
  } */
}
