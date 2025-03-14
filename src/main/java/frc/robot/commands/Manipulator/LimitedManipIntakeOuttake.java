// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

/** An example command that uses an example subsystem. */
public class LimitedManipIntakeOuttake extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Manipulator manip;
    private final Elevator elevator;
    private final Timer timer = new Timer();
    private boolean isIntaking = false;
    private boolean isOuttaking = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param manip The subsystem used by this command.
     */
    public LimitedManipIntakeOuttake(Manipulator manip, Elevator elevator) {
        this.manip = manip;
        this.elevator = elevator;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(manip);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(elevator.isAtMinPosition()) {
            isIntaking = true;
            isOuttaking = false;
            manip.runManip();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (elevator.isAtMinPosition()) {
            isIntaking = true;
            isOuttaking = false;
            if(manip.getFirstBeam() && !manip.getSecondBeam()) {
                timer.start();
                manip.reverseManip();
            }
        } else if (elevator.isAtL1() || elevator.isAtL2() || elevator.isAtL3() || elevator.isAtL4()) {
            manip.runManip();
            isIntaking = false;
            isOuttaking = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        manip.stopManip();
        timer.stop();
        timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(isIntaking) {
            return timer.get() < 0.5;
        } else if(isOuttaking) {
            return manip.getSecondBeam();
        }
        return false;
    }
}
