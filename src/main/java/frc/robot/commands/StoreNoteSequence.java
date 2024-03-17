package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StoreNoteSequence extends SequentialCommandGroup {
    public StoreNoteSequence(Intake intake, Shooter shooter) {
        addCommands(
            /* Move the intake and shooter to the passthrough position to align them */
            intake.setToPassthroughPosition(true)
                .alongWith(shooter.setToPassthroughPosition(true)),
            /* Move the note into the shooter once they have reached the passthrough position */
            intake.enableTransferToShooter(),
            shooter.receiveNoteFromIntake(),
            /* Make sure to put them back in the storage position and disable when the note arrives in the shooter */
            intake.disableFlywheels(),
            intake.setToPassthroughPosition(false),
            shooter.setToUpPosition(false)
        );
    }
}