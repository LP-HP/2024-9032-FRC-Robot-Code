package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
    }
}