package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class MultiNoteAuto extends SequentialCommandGroup {
    /* An auto to test scoring notes with vision */
    public MultiNoteAuto(Swerve swerve, LimelightVision limelight, Shooter shooter, Intake intake, int noteAmt) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("MultiNoteAuto");

        Command firstNoteShootAlignment = AutoBuilder.followPath(paths.get(0));
        Command secondNoteAlignment = AutoBuilder.followPath(paths.get(1));
        
        //Add at least 1 note
        addCommands(
            /* Reset starting pose to limelight pose */
            new InstantCommand(() -> swerve.resetOdometry(limelight.getPoseEstimate().get().pose), swerve, limelight),
            /* Drive to the position to shoot the 1rst note, while moving the arm to the shooting position and lowering the intake */
            firstNoteShootAlignment,
            /* Enable shooter, wait, disable, and store */
            shooter.setShooterVelocityThenWaitThenDisable(5, 0.25),
            shooter.setToStoragePosition()
        );

        //Add a 2nd note
        if(noteAmt > 1) {
            addCommands(
                /* Drive to the position to intake and shoot the 2nd note, while enabling the intake */
                secondNoteAlignment,
                /* Turn the intake off */
                intake.disableIntake(),//TODO use beam break
                /* Move intake and shooter to passthrough position */
                intake.moveToPassthroughPosition().alongWith(shooter.moveArmToPassthroughPosition()), 
                /* Move ring into shooter */
                shooter.enableStorageMotorReceiving(),
                intake.shootIntoShooter(),
                /* Enable shooter, wait, disable, and store */
                shooter.setShooterVelocityThenWaitThenDisable(6, 0.25),
                shooter.setToStoragePosition()
            );
        }

        //TODO To be continued...
    }
}