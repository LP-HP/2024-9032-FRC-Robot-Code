package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(Swerve swerve, LimelightVision limelight) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("TestAuto");//An auto to test 2 paths with a wait

        Command first = AutoBuilder.followPath(paths.get(0));
        Command second = AutoBuilder.followPath(paths.get(1));

        addCommands(
            /* Reset starting pose to limelight pose */
            new InstantCommand(() -> swerve.resetOdometry(limelight.getPoseEstimate()), swerve, limelight),
            first,
            Commands.waitSeconds(1),
            second
        );

        //Updates vision every loop until the auto ends //FIXME will this work? it wont because of scheduler conlicts... HELP
        deadlineWith(
            new InstantCommand(
            () -> swerve.updateVisionLocalization(limelight.getPoseEstimate(), limelight.getMeasurementTime()), swerve, limelight)
            .repeatedly()
        );

        //When we switch to teleop make sure we switch pipelines
        finallyDo(() -> limelight.switchToTargetPipeline());
    }
}