package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class SwerveShakedown extends SequentialCommandGroup {
    /* An auto to test the swerve drive without vision 
     * 
     * The 1rst path should drive in a 2 by 2 meter curvy square
     * The 2nd path should drive in the same square, but also rotate left 90 degrees each side
     * 
    */
    public SwerveShakedown(Swerve swerve) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("SwerveShakedown");

        Command first = AutoBuilder.followPath(paths.get(0));
        Command second = AutoBuilder.followPath(paths.get(1));
        
        addCommands(
            /* Reset odometry to the pose of the 1rst path */
            new InstantCommand(() -> swerve.resetOdometry(paths.get(0).getPreviewStartingHolonomicPose()), swerve),
            first,
            Commands.waitSeconds(5),
            second
        );
    }
}