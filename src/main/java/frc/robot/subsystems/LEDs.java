// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;

public class LEDs extends SubsystemBase{
  private AddressableLED LED;
  private AddressableLEDBuffer LEDBuffer;
  private Elevator elevator;
  private Manipulator manip;
  private Climber climber;
  private CommandSwerveDrivetrain driveTrain;
  private String pathName;
  private final Pose2d[] bluePathStartingPoses;
  private final Pose2d[] redPathStartingPoses;
  private final int LEDLength = 129; //should be the correct length as of 3/19/25
  private final int pwmPort = 2;
  private final Timer timer = new Timer();
  public LEDs(Elevator elevator, Manipulator manip, Climber climber, CommandSwerveDrivetrain driveTrain) {
      this.elevator = elevator;
      this.manip = manip;
      this.climber = climber;
      this.driveTrain = driveTrain;
      bluePathStartingPoses = new Pose2d[]{
              new Pose2d(7.1, 6.5, new Rotation2d(Math.toRadians(-146.598))), //left starting pose
              new Pose2d(7.157, 4.197, Rotation2d.k180deg), //left middle starting pose
              new Pose2d(7.157, 3.863, Rotation2d.k180deg), //right middle starting pose
              new Pose2d(7.1, 1.5, new Rotation2d(Math.toRadians(136.1)))  //right starting pose
      };
      redPathStartingPoses = new Pose2d[]{
              new Pose2d(10.4, 1.5, new Rotation2d(Math.toRadians(33.404))),
              new Pose2d(10.343, 3.803, Rotation2d.kZero),
              new Pose2d(10.343, 4.137, Rotation2d.kZero),
              new Pose2d(10.4, 6.5, new Rotation2d(43.9))
      };

      //set pwmPort
      LED = new AddressableLED(pwmPort);

      //set strip length
      LEDBuffer = new AddressableLEDBuffer(LEDLength);
      LED.setLength(LEDBuffer.getLength());

      //start LEDs
      LED.start();
    }

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
  LEDPattern manualGreen = LEDPattern.solid(new Color(0, 255, 0));
  LEDPattern flashPurple = purple.blink(Time.ofRelativeUnits(0.25, Seconds.getBaseUnit()));
  LEDPattern flashGreen = manualGreen.blink(Time.ofRelativeUnits(0.25, Seconds.getBaseUnit()));

  //animated colors
  LEDPattern rainbow = LEDPattern.rainbow(255, 255);
  LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), kLedSpacing);
  LEDPattern redAndBlue = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));
  LEDPattern timeProgress = LEDPattern.progressMaskLayer(() -> DriverStation.getMatchTime() / 135);
  LEDPattern elevatorDeltaL1 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL1()));
  LEDPattern elevatorDeltaL2 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL2()));
  LEDPattern elevatorDeltaL3 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL3()));
  LEDPattern elevatorDeltaL4 = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getL4()));
  LEDPattern elevatorDeltaHome = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getHome()));
  LEDPattern elevatorDeltaMaxPos = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getMaxPosition()));
  LEDPattern elevatorDeltaMinPos = LEDPattern.progressMaskLayer(() -> Math.abs(elevator.getPosition() / elevator.getMinPosition()));
  

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

    /*
    public void LEDDisabledState() {
        if (DriverStation.getAlliance().isPresent()) {
            double xDistance = 0;
            double yDistance = 0;
            double rotationDistance = 0;
            boolean inPose = false;
            Pose2d[] chosenStartingPoses;
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                chosenStartingPoses = bluePathStartingPoses;
            } else {
                chosenStartingPoses = redPathStartingPoses;
            }
            if (DriverStation.getAlliance().isPresent())
                for (Pose2d startingPose : chosenStartingPoses) {
                    inPose = false;
                    xDistance = Math.abs(driveTrain.getStatePose().getX() - startingPose.getX());
                    yDistance = Math.abs(driveTrain.getStatePose().getY() - startingPose.getY());
                    rotationDistance = Math.abs(driveTrain.getStatePose().getRotation().getDegrees() - startingPose.getRotation().getDegrees());
                    if (xDistance < 1 && yDistance < 1 && rotationDistance < 5) {
                        inPose = true;
                    }
                }
            if (inPose) {
                blue.applyTo(LEDBuffer);
                LED.setData(LEDBuffer);
            } else {
                orange.applyTo(LEDBuffer);
                LED.setData(LEDBuffer);
            }
        }
    }

     */

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() < 20) {
            //should be less than 20 logic in actual match and greater than 110 when not in match
            rainbow.applyTo(LEDBuffer);
            LED.setData(LEDBuffer);
        } else if(manip.getIsIntaking()) {
            timer.start();
            if(timer.hasElapsed(2.5)) {
                timer.stop();
                timer.reset();
                manip.setIsIntaking(false);
            } else {
                flashPurple.applyTo(LEDBuffer);
                LED.setData(LEDBuffer);
            }
        } else if(manip.getIsOuttaking()) {
            timer.start();
            if(timer.hasElapsed(2.5)) {
                timer.stop();
                timer.reset();
                manip.setIsOuttaking(false);
            } else {
                flashGreen.applyTo(LEDBuffer);
                LED.setData(LEDBuffer);
            }
        } else {
            manualYellow.applyTo(LEDBuffer);
            LED.setData(LEDBuffer);
        }
    }
}
