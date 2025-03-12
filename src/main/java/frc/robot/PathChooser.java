package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

import static com.pathplanner.lib.auto.AutoBuilder.getAllAutoNames;

public class PathChooser {
    private List<String> autonames;
    //private final SendableChooser<String> pathChooser;
    private final SendableChooser<Command> autoChooser;

    // For convenience a programmer could change this when going to competition.
    private final boolean isCompetition = true;

    public PathChooser(String chooserType) {
        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> isCompetition
                        ? stream.filter(auto -> auto.getName().startsWith("comp"))
                        : stream
        );

        SmartDashboard.putData("Coral 1 Path", autoChooser);
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    public static SendableChooser<String> buildPathChooserWithOptionsModifier(
            String defaultAutoName,
            Function<Stream<String>, Stream<String>> optionsModifier){
        SendableChooser<String> chooser = new SendableChooser<>();
        String[] pathNames = {"blah", "blah"};

        String defaultOption = null;

        List<String> options = new ArrayList<>(Arrays.asList(pathNames));

        if (defaultOption == null) {
            chooser.setDefaultOption("None", "null");
        } else {
            chooser.setDefaultOption(defaultOption, defaultOption);
            chooser.addOption("None", "null");
        }

        optionsModifier
                .apply(options.stream())
                .forEach(path -> chooser.addOption(path, path));

        return chooser;
    }
}
