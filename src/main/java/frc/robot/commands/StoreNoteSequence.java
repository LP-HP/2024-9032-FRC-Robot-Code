package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StoreNoteSequence extends SequentialCommandGroup {
    public StoreNoteSequence(Intake intake, Shooter shooter) {        
        addCommands(
            /* Set intake and shooter to initial positions */
            intake.setToGroundPositionAndEnable(),
            shooter.setToPassthroughPosition(),
            /* Wait until a note is in the intake */
            Commands.waitUntil(intake::isBeamBreakTriggered),
            /* Move the intake to the passthrough position */
            intake.moveToPassthroughPosition(),
            /* Move the note into shooter once the intake has reached the passthrough position */
            shooter.enableStorageMotorReceiving(),
            intake.shootIntoShooter(),
            Commands.waitUntil(shooter::isBeamBreakTriggered),
            /* Make sure to put the intake back in the storage position when the note arrives in the shooter */
            intake.setToStoragePosition()
        );
    }
}