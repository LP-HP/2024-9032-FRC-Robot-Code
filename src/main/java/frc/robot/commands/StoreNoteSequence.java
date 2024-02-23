package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StoreNoteSequence extends SequentialCommandGroup {
    public StoreNoteSequence(Intake intake, Shooter shooter) {
        addCommands(
            /* Move the intake and shooter to the passthrough position to align them */
            intake.moveToPassthroughPosition()
                .alongWith(shooter.moveToPassthroughPosition()),
            /* Move the note into the shooter once they have reached the passthrough position */
            shooter.enableStorageMotorReceiving(),
            intake.shootIntoShooter(),
            Commands.waitUntil(shooter::hasNote),
            /* Make sure to put them back in the storage position when the note arrives in the shooter */
            intake.setToStoragePosition(),
            shooter.setToStoragePosition()
        );
    }
}