package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class VisionSubsystem extends SubsystemBase {
    public abstract VisionTarget getCurrentTarget();
}