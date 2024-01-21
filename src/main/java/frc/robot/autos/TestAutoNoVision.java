package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class TestAutoNoVision extends SequentialCommandGroup {
    /* An auto to test 2 paths with a wait without vision */
    public TestAutoNoVision(Swerve swerve) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("TestAuto");

        Command first = AutoBuilder.followPath(paths.get(0));
        Command second = AutoBuilder.followPath(paths.get(1));
        
        addCommands(
            first,
            Commands.waitSeconds(1),
            second
        );

    }
}