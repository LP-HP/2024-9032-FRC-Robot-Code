package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class SparkMaxWrapper extends CANSparkMax implements Sendable {
    private final SparkMaxConstants constants;
    private final SparkPIDController controller;
    public final RelativeEncoder relativeEncoder;

    private AbsoluteEncoder absoluteEncoder;

    private double closedLoopSetpoint;
    private boolean isConfigured = false;

    public SparkMaxWrapper(SparkMaxConstants constants) {
        super(constants.id(), MotorType.kBrushless);

        restoreFactoryDefaults();

        this.constants = constants;
        controller = getPIDController();
        relativeEncoder = getEncoder();

        SendableRegistry.add(this, constants.name());
    }

    public void configAbsoluteEncoder(boolean invert) {
        if(isConfigured) {
            System.err.println("Call config absolute encoder before calling the normal config method");

            return;
        }

        absoluteEncoder = getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setInverted(invert);
    }

    public void configPIDWrapping(double min, double max) {
        if(isConfigured) {
            System.err.println("Call config PID wrapping before calling the normal config method");

            return;
        }

        controller.setPositionPIDWrappingEnabled(true);
        controller.setPositionPIDWrappingMaxInput(max);
        controller.setPositionPIDWrappingMinInput(min);
    }

    public void config() {
        switch (constants.mode()) {
            case position:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kPositionOnly);
                checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));
                break;
            case positionLeader:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kPositionOnly, true);
                checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));
                break;
            case velocity:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kVelocityOnly);
                checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units??
                break;
            case velocityLeader:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kVelocityOnly, true);
                checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units??
                break;
            case percentOutput:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kMinimal);
                break;
            case velocityControlWithPositionData:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kAll);
                checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));
                checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units??
                break;
        }

        setInverted(constants.inverted());

        checkError(setSmartCurrentLimit(constants.currentLimit()));

        checkError(setIdleMode(constants.idleMode()));

        checkError(controller.setP(constants.pidConstants().kP()));
        checkError(controller.setI(constants.pidConstants().kI()));
        checkError(controller.setD(constants.pidConstants().kD()));
        checkError(controller.setFF(constants.pidConstants().kF()));

        checkError(enableVoltageCompensation(constants.nominalVoltage()));

        if(Constants.burnFlash) {
            Timer.delay(0.5);
            checkError(burnFlash());
            Timer.delay(0.5);

            System.out.println("Burned the flash of " + constants.name());
        }

        isConfigured = true;
    }      

    public void setClosedLoopTarget(double setpoint, double feedforward) {
        if(!isConfigured) {
            System.err.println("Motor not configured!");

            return;
        }

        closedLoopSetpoint = setpoint;

        switch (constants.mode()) {
            case position:
            case positionLeader:
                controller.setReference(setpoint, ControlType.kPosition, 0, feedforward);
                break;
            case velocity:
            case velocityControlWithPositionData:
            case velocityLeader:
                controller.setReference(setpoint, ControlType.kVelocity, 0, feedforward);
                break;
            case percentOutput:
                System.err.println("Cannot set closed loop target on percent output mode");
                break;
        }          
    }

    public void setClosedLoopTarget(double setpoint) {
        if(!isConfigured) {
            System.err.println("Motor not configured!");

            return;
        }

        closedLoopSetpoint = setpoint;

        switch (constants.mode()) {
            case position:
            case positionLeader:
                controller.setReference(setpoint, ControlType.kPosition);
                break;
            case velocity:
            case velocityControlWithPositionData:
            case velocityLeader:
                controller.setReference(setpoint, ControlType.kVelocity);
                break;
            case percentOutput:
                System.err.println("Cannot set closed loop target on percent output mode");
                break;
        }          
    }

    public double getAbsolutePosition() {
        return absoluteEncoder != null ? absoluteEncoder.getPosition() : null;
    }

    private void checkError(REVLibError error) {
        if(error != REVLibError.kOk) 
            System.err.println(constants.name() + " has error " + error);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        switch (constants.mode()) {
            case position:
            case positionLeader:
                builder.addDoubleProperty("Position", relativeEncoder::getPosition, null);
                builder.addDoubleProperty("Absolute Position", this::getAbsolutePosition, null);
                builder.addDoubleProperty("Target Position", () -> closedLoopSetpoint, null);
                break;
            case velocity:
            case velocityLeader:
                builder.addDoubleProperty("Velocity", relativeEncoder::getVelocity, null);
                builder.addDoubleProperty("Target Velocity", () -> closedLoopSetpoint, null);
                break;
            case velocityControlWithPositionData:
                builder.addDoubleProperty("Position", relativeEncoder::getPosition, null);
                builder.addDoubleProperty("Velocity", relativeEncoder::getVelocity, null);
                builder.addDoubleProperty("Target Velocity", () -> closedLoopSetpoint, null);
                break;
            case percentOutput:
                break;
        }   

        builder.addDoubleProperty("Applied Output", this::getAppliedOutput, null);
    }
}