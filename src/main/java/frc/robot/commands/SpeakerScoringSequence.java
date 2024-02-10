package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerScoringSequence extends SequentialCommandGroup {
    public SpeakerScoringSequence(Swerve swerve, LimelightVision limelight, Shooter shooter) {
        addCommands(
            /* Rotate to target while moving the arm to the initial limelight reading */
            new AlignWithRotationTarget(swerve, () -> limelight.getAprilTagTarget().xOffset)
                .alongWith(shooter.moveArmToPositionFromTargetY(() -> limelight.getAprilTagTarget().yOffset)),
            /* Move the arm to the current target to ensure that the target has not changed since the initial limelight reading */
            shooter.moveArmToPositionFromTargetY(() -> limelight.getAprilTagTarget().yOffset),
            shooter.setShooterVelocityThenWaitThenDisable(5, 0.25),//TODO do lookup table if needed
            shooter.setToStoragePosition()
        );
    }
}