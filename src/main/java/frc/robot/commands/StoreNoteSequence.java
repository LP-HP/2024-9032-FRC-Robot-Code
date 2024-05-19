package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.ShooterFlywheels;

public class StoreNoteSequence extends SequentialCommandGroup {
    public StoreNoteSequence(Intake intake, ShooterArm shooterArm, ShooterFlywheels shooterFlywheels, boolean resetShooter) {
        addCommands(
            /* Move the intake and shooter to the passthrough position to align them */
            intake.setToPassthroughPosition(true)
                .alongWith(shooterArm.setToPassthroughPosition(true)),
            /* Move the note into the shooter once they have reached the passthrough position */
            intake.enableTransferToShooter(),
            shooterFlywheels.receiveNoteFromIntake()
                /* If the beam break doesn't see a note in time, recover from a failed passthrough by reintaking the note */
                .deadlineWith(
                    Commands.waitSeconds(Constants.passthroughRecoveryWait)
                    .andThen(Commands.print("Transfer timed out!"))
                    .andThen(intake.getNoteFromGround())
                    .andThen(intake.setToPassthroughPosition(true))
                    .andThen(Commands.waitSeconds(0.5))
                    .andThen(intake.enableTransferToShooter())
                    .andThen(Commands.print("Recovered transfer"))),
            /* Make sure to put them back in the storage position and disable when the note arrives in the shooter */
            intake.disableRollers(),
            /* Spin up flywheels for faster shooting */
            shooterFlywheels.spinUpFlywheels(95.0)//TODO do velocity lookup table if needed
        );

        if(resetShooter)
            addCommands(shooterArm.setToUpPosition(false));
    }
}