package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StoreNoteFromGroundSequence extends SequentialCommandGroup {
    public StoreNoteFromGroundSequence(Intake intake, Shooter shooter) {        
        addCommands(
            /* Put the intake on the ground*/
            intake.setToGroundPositionAndEnable(),
            /* Wait until a note is in the intake */
            Commands.waitUntil(intake::isBeamBreakTriggered),
            /* Store note */
            new StoreNoteSequence(intake, shooter)
        );
    }
}