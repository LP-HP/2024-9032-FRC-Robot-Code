package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class SwerveShakedown extends SequentialCommandGroup {
    /* An auto to test the swerve drive without vision 
     * 
     * The 1rst path should just drive forward 2 meters
     * The 2nd path should drive to the left in a curve 2 meters while rotating in the movement direction
     * 
    */
    public SwerveShakedown(Swerve swerve) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("SwerveShakedown");

        Command first = AutoBuilder.followPath(paths.get(0));
        Command second = AutoBuilder.followPath(paths.get(1));
        
        addCommands(
            first,
            Commands.waitSeconds(5),
            second
        );
    }
}