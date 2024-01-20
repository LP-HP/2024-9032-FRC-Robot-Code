package frc.robot.autos;

import java.util.List;
import java.util.function.IntSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;

public class MultiNoteAuto extends SequentialCommandGroup {
    /* An auto to test scoring notes with vision */
    public MultiNoteAuto(Swerve swerve, LimelightVision limelight, IntSupplier notesSup) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("MultiNoteAuto");

        Command first = AutoBuilder.followPath(paths.get(0));
        Command second = AutoBuilder.followPath(paths.get(1));
        
        addCommands(
            /* Reset starting pose to limelight pose */
            new InstantCommand(() -> swerve.resetOdometry(limelight.getPoseEstimate().get().pose), swerve, limelight),
            first,
            Commands.waitSeconds(1),
            second
        );

    }
}