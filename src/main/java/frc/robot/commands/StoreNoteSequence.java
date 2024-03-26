package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.ShooterFlywheels;

public class StoreNoteSequence extends SequentialCommandGroup {
    public StoreNoteSequence(Intake intake, ShooterArm shooterArm, ShooterFlywheels shooterFlywheels) {
        addCommands(
            /* Move the intake and shooter to the passthrough position to align them */
            intake.setToPassthroughPosition(true)
                .alongWith(shooterArm.setToPassthroughPosition(true)),
            /* Move the note into the shooter once they have reached the passthrough position */
            intake.enableTransferToShooter(),
            shooterFlywheels.receiveNoteFromIntake(),
            /* Make sure to put them back in the storage position and disable when the note arrives in the shooter */
            intake.disableRollers(),
            intake.setToPassthroughPosition(false),
            shooterArm.setToUpPosition(false),
            /* Spin up flywheels for faster shooting */
            shooterFlywheels.spinUpFlywheels(95.0)//TODO do velocity lookup table if needed
        );
    }
}