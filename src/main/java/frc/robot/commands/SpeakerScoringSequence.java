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
                .alongWith(shooter.setToTargetPositionFromTargetY(() -> limelight.getAprilTagTarget().yOffset, false)),
            /* Move the arm to the current target to ensure that the target has not changed since the initial limelight reading */
            shooter.setToTargetPositionFromTargetY(() -> limelight.getAprilTagTarget().yOffset, true),
            shooter.shootSequence(4000.0)//TODO do lookup table if needed
        );
    }
}