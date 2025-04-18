package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Elevator.GoToHome;
import frc.robot.commands.Elevator.GoToL4;
import frc.robot.commands.Elevator.GoToMinPosition;
import frc.robot.commands.Manipulator.AutonLimitedManipIntake;
import frc.robot.commands.Manipulator.LimitedManipIntake;
import frc.robot.commands.Manipulator.LimitedManipIntakeReverse;
import frc.robot.commands.Manipulator.LimitedManipOuttake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static com.pathplanner.lib.auto.AutoBuilder.pathfindThenFollowPath;

public class PathChooser {
    private static SendableChooser<Command> pathChooser1 = null;
    private static SendableChooser<Command> pathChooser2 = null;
    private static SendableChooser<Command> pathChooser3 = null;
    private static SendableChooser<Command> stationChooser1 = null;
    private static SendableChooser<Command> stationChooser2 = null;
    private static SendableChooser<Command> stationChooser3 = null;
    private static PathConstraints defaultConstraints = new PathConstraints(
            2, 2,
            Units.degreesToRadians(180), Units.degreesToRadians(180));

    // For convenience a programmer could change this when going to competition.
    private static final boolean isCompetition = true;

    public static SendableChooser<Command> buildAndSendCoralChooser(String chooserType, Manipulator manip, Elevator elevator, LEDs LED) {
        // Build an auto chooser. This will use empty instant command as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        if(chooserType.equals("Coral 1")) {
            pathChooser1 = buildPathChooserWithOptionsModifier(
                    (stream) -> isCompetition
                            ? stream.filter(path -> path.name.startsWith("Follow"))
                            :stream,
                    manip, elevator, LED
            );
            return pathChooser1;
        } else if(chooserType.equals("Coral 2")) {
            pathChooser2 = buildPathChooserWithOptionsModifier(
                    (stream) -> isCompetition
                            ? stream.filter(path -> path.name.startsWith("Follow"))
                            :stream,
                    manip, elevator, LED
            );
            return pathChooser2;
        } else if(chooserType.equals("Coral 3")) {
            pathChooser3 = buildPathChooserWithOptionsModifier(
                    (stream) -> isCompetition
                            ? stream.filter(path -> path.name.startsWith("Follow"))
                            :stream,
                    manip, elevator, LED
            );
            return pathChooser3;
        }
        return null;
    }

    public static SendableChooser<Command> buildAndSendStationChooser(String chooserType, Manipulator manip, Elevator elevator, LEDs LED) {
        if (chooserType.equals("Station 1")) {
            stationChooser1 = new SendableChooser<>();
            stationChooser1.setDefaultOption("None", new InstantCommand());
            try {
                stationChooser1.addOption("Left Station", new SequentialCommandGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationLeft"), defaultConstraints),
                        new SequentialCommandGroup(
                                new LimitedManipIntake(manip, elevator, LED).asProxy(),
                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                        )
                ));
                stationChooser1.addOption("Right Station", new SequentialCommandGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationRight"), defaultConstraints),
                        new SequentialCommandGroup(
                                new LimitedManipIntake(manip, elevator, LED).asProxy(),
                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                        )
                ));
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
            return stationChooser1;
        } else if(chooserType.equals("Station 2")) {
            stationChooser2 = new SendableChooser<>();
            stationChooser2.setDefaultOption("None", new InstantCommand());
            try {
                stationChooser2.addOption("Left Station", new SequentialCommandGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationLeft"), defaultConstraints),
                        new SequentialCommandGroup(
                                new LimitedManipIntake(manip, elevator, LED).asProxy(),
                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                        )
                ));
                stationChooser2.addOption("Right Station", new SequentialCommandGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationRight"), defaultConstraints),
                        new SequentialCommandGroup(
                                new LimitedManipIntake(manip, elevator, LED).asProxy(),
                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                        )
                ));
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
            return stationChooser2;
        } else if(chooserType.equals("Station 3")) {
            stationChooser3 = new SendableChooser<>();
            stationChooser3.setDefaultOption("None", new InstantCommand());
            try {
                stationChooser3.addOption("Left Station", new SequentialCommandGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationLeft"), defaultConstraints),
                        new SequentialCommandGroup(
                                new LimitedManipIntake(manip, elevator, LED).asProxy(),
                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                        )
                ));
                stationChooser3.addOption("Right Station", new SequentialCommandGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationRight"), defaultConstraints),
                        new SequentialCommandGroup(
                                new LimitedManipIntake(manip, elevator, LED).asProxy(),
                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                        )
                ));
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
            return stationChooser3;
        }
        return null;
    }

    public static Command getPathFollowCommand(SendableChooser<Command> pathChooser) {
        if(pathChooser == null) {
            System.out.println("Path Chooser is null");
            return new InstantCommand();
        } else {
            return pathChooser.getSelected();
        }
    }

    public static SendableChooser<Command> buildPathChooserWithOptionsModifier(
            Function<Stream<PathPlannerPath>, Stream<PathPlannerPath>> optionsModifier,
            Manipulator manip, Elevator elevator, LEDs LED) {
        return buildPathChooserWithOptionsModifier("", optionsModifier, manip, elevator, LED);
    }

    public static SendableChooser<Command> buildPathChooserWithOptionsModifier(
            String defaultPathName,
            Function<Stream<PathPlannerPath>, Stream<PathPlannerPath>> optionsModifier,
            Manipulator manip, Elevator elevator, LEDs LED) {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        SendableChooser<Command> chooser = new SendableChooser<>();
        List<String> pathNames = getAllPathNames();

        PathPlannerPath defaultOption = null;
        List<PathPlannerPath> options = new ArrayList<>();

        for (String pathname : pathNames) {
            PathPlannerPath path = null;
            try {
                path = PathPlannerPath.fromPathFile(pathname);
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }

            if (!defaultPathName.isEmpty() && defaultPathName.equals(pathname)) {
                defaultOption = path;
            } else {
                options.add(path);
            }
        }

        if (defaultOption == null) {
            chooser.setDefaultOption("None", new InstantCommand());
        } else {
            chooser.setDefaultOption(defaultOption.name, pathfindThenFollowPath(defaultOption, defaultConstraints));
            chooser.addOption("None", new InstantCommand());
        }

        optionsModifier
                .apply(options.stream())
                .forEach(path -> {
                    try {
                        chooser.addOption(path.name,
                                new SequentialCommandGroup(
                                        new ParallelDeadlineGroup(
                                                pathfindThenFollowPath(path, defaultConstraints),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(0.75),
                                                        new ParallelCommandGroup(
                                                                new GoToL4(elevator, manip, LED),
                                                                new SequentialCommandGroup(
                                                                        new WaitCommand(0.1),
                                                                        new AutonLimitedManipIntake(manip, elevator, LED).asProxy(),
                                                                        new LimitedManipIntakeReverse(manip, LED).asProxy()
                                                                )
                                                        )
                                                )
                                        ),
                                        new LimitedManipOuttake(manip, elevator, LED).asProxy(),
                                        new ParallelCommandGroup(
                                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Reverse" + path.name)),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(2),
                                                        new GoToMinPosition(elevator, LED).asProxy()
                                                )
                                        )
                                ));
                    } catch (IOException | ParseException e) {
                        throw new RuntimeException(e);
                    }
                });

        return chooser;
    }

    public static List<String> getAllPathNames() {
        File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

        if (pathFiles == null) {
            return new ArrayList<>();
        }

        return Stream.of(pathFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".path"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
    }

    public static SequentialCommandGroup buildAutoCommand(Elevator elevator, LEDs LED) {
        //for now only 3 coral and two trips to coral station
        Command coral1Choice = getPathFollowCommand(pathChooser1).asProxy();
        Command coral2Choice = getPathFollowCommand(pathChooser2).asProxy();
        Command coral3Choice = getPathFollowCommand(pathChooser3).asProxy();
        Command stationChoice1 = getPathFollowCommand(stationChooser1).asProxy();
        Command stationChoice2 = getPathFollowCommand(stationChooser2).asProxy();
        Command stationChoice3 = getPathFollowCommand(stationChooser3).asProxy();
        return new SequentialCommandGroup(
                new GoToHome(elevator, LED).asProxy(),
                coral1Choice,
                stationChoice1,
                coral2Choice,
                stationChoice2,
                coral3Choice);
        //return new SequentialCommandGroup(coral1Choice, coral2Choice, coral3Choice);
    }

    public static Command buildWeirdAuton(Elevator elevator, Manipulator manip, LEDs LED) throws IOException, ParseException {
        Command middlePath = AutoBuilder.buildAuto("WeirdMiddle");
        Command placeH = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("FollowL4H"), defaultConstraints),
                        new SequentialCommandGroup(
                                new WaitCommand(0.2),
                                new ParallelCommandGroup(
                                        new GoToL4(elevator, manip, LED),
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.1),
                                                new AutonLimitedManipIntake(manip, elevator, LED).asProxy(),
                                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                                        )
                                )
                        )
                ),
                new LimitedManipOuttake(manip, elevator, LED).asProxy(),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReverseFollowL4H")),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new GoToMinPosition(elevator, LED).asProxy()
                        )
                )
        );
        Command placeG = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        pathfindThenFollowPath(PathPlannerPath.fromPathFile("FollowL4G"), defaultConstraints),
                        new SequentialCommandGroup(
                                new WaitCommand(0.2),
                                new ParallelCommandGroup(
                                        new GoToL4(elevator, manip, LED),
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.1),
                                                new AutonLimitedManipIntake(manip, elevator, LED).asProxy(),
                                                new LimitedManipIntakeReverse(manip, LED).asProxy()
                                        )
                                )
                        )
                ),
                new LimitedManipOuttake(manip, elevator, LED).asProxy(),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReverseFollowL4G")),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new GoToMinPosition(elevator, LED).asProxy()
                        )
                )
        );
        return middlePath;
    }
}
