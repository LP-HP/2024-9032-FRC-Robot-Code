package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends SequentialCommandGroup
{
    public TestAuto()
    {
        Command testPathCommand = AutoBuilder.followPathWithEvents(PathPlannerPath.fromPathFile("Test"));

        addCommands(testPathCommand);
    }
}