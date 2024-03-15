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
import frc.robot.util.SparkMaxConstants.SparkMaxPIDConstants;

public class SparkMaxWrapper extends CANSparkMax implements Sendable {
    private final SparkMaxConstants constants;
    private final SparkPIDController controller;
    public final RelativeEncoder relativeEncoder;

    private AbsoluteEncoder absoluteEncoder;

    private double closedLoopSetpoint = 0.0;
    private boolean isConfigured = false;
    private boolean hasAbsoluteEncoder = false;
    private boolean hasError = false;

    private static boolean hasAnyMotorErrors = false;

    public SparkMaxWrapper(SparkMaxConstants constants) {
        super(constants.id(), MotorType.kBrushless);

        if(Constants.configMotors)
            restoreFactoryDefaults();

        this.constants = constants;
        controller = getPIDController();
        relativeEncoder = getEncoder();

        SendableRegistry.add(this, constants.name());
    }

    public void configAbsoluteEncoder(boolean invert, double positionConversionFactor, double offset) {
        if(isConfigured) {
            System.err.println("Call config absolute encoder before calling the normal config method");

            return;
        }

        absoluteEncoder = getAbsoluteEncoder(Type.kDutyCycle);

        if(Constants.configMotors) {
            checkError(absoluteEncoder.setInverted(invert));
            checkError(absoluteEncoder.setPositionConversionFactor(positionConversionFactor));
            checkError(absoluteEncoder.setZeroOffset(offset));
        }

        hasAbsoluteEncoder = true;
    }

    public void configPIDWrapping(double min, double max) {
        if(isConfigured) {
            System.err.println("Call config PID wrapping before calling the normal config method");

            return;
        }

        if(Constants.configMotors) {
            checkError(controller.setPositionPIDWrappingEnabled(true));
            checkError(controller.setPositionPIDWrappingMaxInput(max));
            checkError(controller.setPositionPIDWrappingMinInput(min));
        }
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
                checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units?? - /60 seems to work
                break;
            case velocityLeader:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kVelocityOnly, true);
                checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));
                break;
            case percentOutput:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kMinimal);
                break;
            case velocityControlWithPositionData:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kAll);
                checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));
                checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));
                break;
        }

        if(!Constants.configMotors) {
            isConfigured = true;

            return;
        }

        setInverted(constants.inverted());

        checkError(setSmartCurrentLimit(constants.currentLimit()));

        checkError(setIdleMode(constants.idleMode()));

        if(constants.pidConstants() != null) {
            updatePIDConstants(constants.pidConstants());
        }

        checkError(enableVoltageCompensation(constants.nominalVoltage()));

        if(Constants.burnFlash) {
            Timer.delay(0.5);
            checkError(burnFlash());
            Timer.delay(0.5);

            System.out.println("Burned the flash of " + constants.name());
        }

        if(hasError) 
            return;

        System.out.println("Configured " + constants.name());

        isConfigured = true;
    }      

    public void setClosedLoopTarget(double setpoint, double feedforward) {
        if(!isConfigured) {
            System.err.println(constants.name() + " not configured!");

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

    public void updatePIDConstants(SparkMaxPIDConstants constants) {
        checkError(controller.setP(constants.kP()));
        checkError(controller.setI(constants.kI()));
        checkError(controller.setD(constants.kD()));
        checkError(controller.setFF(constants.kF()));
        checkError(controller.setOutputRange(constants.minOutput(), constants.maxOutput()));

        System.out.println("Updated PID constants of " + this.constants.name());
    }

    public void setClosedLoopTarget(double setpoint) {
        if(!isConfigured) {
            System.err.println(constants.name() + " not configured!");

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
        return hasAbsoluteEncoder ? absoluteEncoder.getPosition() : 0.0;
    }

    private void checkError(REVLibError error) {
        if(error != REVLibError.kOk) {
            System.err.println(constants.name() + " has error " + error);

            hasError = true;
            hasAnyMotorErrors = true;
        }
    }

    public double getSetpoint() {
        return closedLoopSetpoint;
    }

    public void setAbsoluteEncoderAsFeedback() {
        if(!hasAbsoluteEncoder) {
            System.err.println("Must config absolute encoder before setting as feedback!");

            return;
        }

        if(Constants.configMotors)
            checkError(controller.setFeedbackDevice(absoluteEncoder));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        switch (constants.mode()) {
            case position:
            case positionLeader:
                builder.addDoubleProperty("Position", relativeEncoder::getPosition, null);
                if(hasAbsoluteEncoder)
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

    public static boolean noMotorErrors() {
        return !hasAnyMotorErrors;
    }
}