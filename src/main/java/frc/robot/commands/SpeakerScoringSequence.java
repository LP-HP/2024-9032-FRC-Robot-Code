package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Shooter;

public class SpeakerScoringSequence extends SequentialCommandGroup {
    public SpeakerScoringSequence(LimelightVision limelight, Shooter shooter) {
        addCommands(
            /* Move the arm to the target position based on the limelight reading */
            shooter.setToTargetPositionFromDistance(() -> limelight.getAprilTagTarget().yOffset, true),//TODO use distance
            shooter.shootSequence(95.0)//TODO do lookup table if needed
        );
    }
}