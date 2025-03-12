package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private static SendableChooser<Command> stationChooser = null;
    private static PathConstraints defaultConstraints = new PathConstraints(
            2.5, 1,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // For convenience a programmer could change this when going to competition.
    private static final boolean isCompetition = true;

    public static void buildAndSendCoralChooser(String chooserType) throws IOException, ParseException {
        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        if(chooserType.equals("Coral 1")) {
            pathChooser1 = buildPathChooserWithOptionsModifier(
                    (stream) -> isCompetition
                            ? stream.filter(path -> path.name.startsWith("follow"))
                            : stream
            );
            SmartDashboard.putData("Coral 1 Path", pathChooser1);

        } else if(chooserType.equals("Coral 2")) {
            pathChooser2 = buildPathChooserWithOptionsModifier(
                    (stream) -> isCompetition
                            ? stream.filter(path -> path.name.startsWith("follow"))
                            : stream
            );
            SmartDashboard.putData("Coral 2 Path", pathChooser2);

        } else if(chooserType.equals("Coral 3")) {
            pathChooser3 = buildPathChooserWithOptionsModifier(
                    (stream) -> isCompetition
                            ? stream.filter(path -> path.name.startsWith("follow"))
                            : stream
            );
            SmartDashboard.putData("Coral 3 Path", pathChooser3);
        }
    }

    public static void buildAndSendStationChooser() throws IOException, ParseException {
        stationChooser = new SendableChooser<>();
        stationChooser.addOption("Left Station", pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationLeft"), defaultConstraints));
        stationChooser.addOption("Right Station", pathfindThenFollowPath(PathPlannerPath.fromPathFile("StationRight"), defaultConstraints));
        SmartDashboard.putData("Station Chooser", stationChooser);
    }

    public static Command getPathFollowCommand(SendableChooser<Command> pathChooser) {
        return pathChooser.getSelected();
    }

    public static SendableChooser<Command> buildPathChooserWithOptionsModifier(
            Function<Stream<PathPlannerPath>, Stream<PathPlannerPath>> optionsModifier) throws IOException, ParseException {
        return buildPathChooserWithOptionsModifier("", optionsModifier);
    }

    public static SendableChooser<Command> buildPathChooserWithOptionsModifier(
            String defaultPathName,
            Function<Stream<PathPlannerPath>, Stream<PathPlannerPath>> optionsModifier) throws IOException, ParseException {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        SendableChooser<Command> chooser = new SendableChooser<>();
        List<String> pathNames = getAllPathNames();

        PathPlannerPath defaultOption = null;
        List<PathPlannerPath> options = new ArrayList<>();

        for (String pathname : pathNames) {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);

            if (!defaultPathName.isEmpty() && defaultPathName.equals(pathname)) {
                defaultOption = path;
            } else {
                options.add(path);
            }
        }

        if (defaultOption == null) {
            chooser.setDefaultOption("None", Commands.none());
        } else {
            chooser.setDefaultOption(defaultOption.name, pathfindThenFollowPath(defaultOption, defaultConstraints));
            chooser.addOption("None", Commands.none());
        }

        optionsModifier
                .apply(options.stream())
                .forEach(path -> chooser.addOption(path.name, pathfindThenFollowPath(path, defaultConstraints)));

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

    public static SequentialCommandGroup buildAutoCommand() {
        //for now only 3 coral and two trips to coral station
        Command coral1Choice = getPathFollowCommand(pathChooser1);
        Command coral2Choice = getPathFollowCommand(pathChooser2);
        Command coral3Choice = getPathFollowCommand(pathChooser3);
        Command stationChoice = getPathFollowCommand(stationChooser);
        return new SequentialCommandGroup(coral1Choice, stationChoice, coral2Choice, stationChoice, coral3Choice);
    }
}
