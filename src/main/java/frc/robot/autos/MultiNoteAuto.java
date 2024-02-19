package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StoreNoteFromGroundSequence;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AutoConstants.*;

public class MultiNoteAuto extends SequentialCommandGroup {
    /* An auto to test scoring notes with vision */
    public MultiNoteAuto(Swerve swerve, LimelightVision limelight, Shooter shooter, Intake intake, int noteAmt) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("MultiNoteAuto");

        Command firstNoteShootAlignment = AutoBuilder.followPath(paths.get(0));
        Command secondNoteAlignment = AutoBuilder.followPath(paths.get(1));
        
        Command getNoteFromGround = new StoreNoteFromGroundSequence(intake, shooter);

        //Add at least 1 note
        addCommands(
            /* Reset starting pose to limelight pose and start adding vision measurements */
            new InstantCommand(() -> swerve.resetOdometry(limelight.getPoseEstimate().get().pose), swerve, limelight),
            new InstantCommand(() -> swerve.addOptionalVisionPoseSupplier(limelight::getPoseEstimate), swerve, limelight),
            /* Drive to the position to shoot the 1rst note, while setting the arm to the shooting position */
            firstNoteShootAlignment
                .alongWith(shooter.setTargetPosition(armPosNote1)),
            /* Enable shooter, wait, and disable */
            shooter.shootSequence(shooterVelocityNote1)
        );

        //Add a 2nd note
        if(noteAmt > 1) {
            addCommands(
                /* Drive to the position to intake and shoot the 2nd note, while running the full sequence for storing a note */
                secondNoteAlignment
                    .alongWith(getNoteFromGround
                        /* When the note is stored, set the arm to the correct position */
                        .andThen(shooter.setTargetPosition(armPosNote2))
                        /* If we take too long, timeout to prevent blocking*/
                        .withTimeout(notePickupTimeout)),
                /* Enable shooter, wait, disable, and store */
                shooter.shootSequence(shooterVelocityNote2)
            );
        }

        //TODO To be continued...
    }
}