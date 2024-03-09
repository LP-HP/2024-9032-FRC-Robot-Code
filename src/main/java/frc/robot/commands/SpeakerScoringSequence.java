package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SpeakerScoringSequence extends SequentialCommandGroup {
    public SpeakerScoringSequence(Swerve swerve, LimelightVision limelight, Shooter shooter) {
        addCommands(
            /* Rotate to target while moving the arm to the initial limelight reading */
            new AlignWithVisionTarget(swerve, limelight, true, true).withTimeout(1.0)//TODO timeout or no
                .alongWith(shooter.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().yOffset, false)),
            /* Move the arm to the current target to ensure that the target has not changed since the initial limelight reading */
            shooter.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().yOffset, true),//TODO use distance
            shooter.shootSequence(95.0)//TODO do lookup table if needed
        );
    }
}