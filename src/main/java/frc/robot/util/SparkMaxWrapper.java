package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.swerveutil.CANSparkMaxUtil;
import frc.lib.swerveutil.CANSparkMaxUtil.Usage;

public class SparkMaxWrapper extends CANSparkMax implements Sendable {
    private final SparkMaxConstants constants;
    private final SparkPIDController controller;

    public final RelativeEncoder relativeEncoder;

    private AbsoluteEncoder absoluteEncoder;

    private double closedLoopSetpoint;
    private boolean isConfigured = false;

    public SparkMaxWrapper(SparkMaxConstants constants) {
        super(constants.id(), MotorType.kBrushless);

        this.constants = constants;
        controller = getPIDController();
        relativeEncoder = getEncoder();
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
        boolean burnFlash = false;

        switch (constants.mode()) {
            case position:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kPositionOnly);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor())
                    checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));
                break;
            case positionLeader:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kPositionOnly, true);
                break;
            case velocity:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kVelocityOnly);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor())
                    checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units??
                break;
            case velocityLeader:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kVelocityOnly, true);
                break;
            case percentOutput:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kMinimal);
                break;
            case velocityControlWithPositionData:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kAll);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor()) {
                    checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));
                    checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));
                }
                break;
        }

        checkError(setSmartCurrentLimit(constants.currentLimit()));

        if(getInverted() != constants.inverted()) {
            setInverted(constants.inverted());

            burnFlash = true;
        }

        if(getIdleMode() != constants.idleMode()) {
            checkError(setIdleMode(constants.idleMode()));

            burnFlash = true;
        }

        if(constants.pidConstants() != null && updatePIDConstants())
            burnFlash = true;

        if(getVoltageCompensationNominalVoltage() != constants.nominalVoltage()) {
            checkError(enableVoltageCompensation(constants.nominalVoltage()));

            burnFlash = true;
        }

        if(burnFlash) {
            checkError(burnFlash());
        }

        isConfigured = true;
    }   

    private boolean updatePIDConstants() {
        boolean constantChanged = false;

        if(constants.pidConstants().kP() != controller.getP()) {
            checkError(controller.setP(constants.pidConstants().kP()));

            constantChanged = true;
        }

        if(constants.pidConstants().kI() != controller.getI()) {
            checkError(controller.setI(constants.pidConstants().kI()));

            constantChanged = true;
        }

        if(constants.pidConstants().kD() != controller.getD()) {
            checkError(controller.setD(constants.pidConstants().kD()));

            constantChanged = true;
        }

        if(constants.pidConstants().kF() != controller.getFF()) {
            checkError(controller.setFF(constants.pidConstants().kF()));

            constantChanged = true;
        }

        return constantChanged;
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
        return absoluteEncoder != null ? absoluteEncoder.getPosition() : 0;
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
                builder.addDoubleProperty(constants.name() + " Position", relativeEncoder::getPosition, null);
                builder.addDoubleProperty(constants.name() + " Absolute Position", this::getAbsolutePosition, null);
                builder.addDoubleProperty(constants.name() + " Target Position", () -> closedLoopSetpoint, null);
                builder.addDoubleProperty(constants.name() + " kP", controller::getP, controller::setP);
                builder.addDoubleProperty(constants.name() + " kI", controller::getI, controller::setI);
                builder.addDoubleProperty(constants.name() + " kD", controller::getD, controller::setD);
                builder.addDoubleProperty(constants.name() + " kF", controller::getFF, controller::setFF);
                break;
            case velocity:
            case velocityLeader:
                builder.addDoubleProperty(constants.name() + " Velocity", relativeEncoder::getVelocity, null);
                builder.addDoubleProperty(constants.name() + " Target Velocity", () -> closedLoopSetpoint, null);
                builder.addDoubleProperty(constants.name() + " kP", controller::getP, controller::setP);
                builder.addDoubleProperty(constants.name() + " kI", controller::getI, controller::setI);
                builder.addDoubleProperty(constants.name() + " kD", controller::getD, controller::setD);
                builder.addDoubleProperty(constants.name() + " kF", controller::getFF, controller::setFF);
                break;
            case velocityControlWithPositionData:
                builder.addDoubleProperty(constants.name() + " Position", relativeEncoder::getPosition, null);
                builder.addDoubleProperty(constants.name() + " Velocity", relativeEncoder::getVelocity, null);
                builder.addDoubleProperty(constants.name() + " Target Velocity", () -> closedLoopSetpoint, null);
                builder.addDoubleProperty(constants.name() + " kP", controller::getP, controller::setP);
                builder.addDoubleProperty(constants.name() + " kP", controller::getP, controller::setP);
                builder.addDoubleProperty(constants.name() + " kI", controller::getI, controller::setI);
                builder.addDoubleProperty(constants.name() + " kD", controller::getD, controller::setD);
                builder.addDoubleProperty(constants.name() + " kF", controller::getFF, controller::setFF);
                break;
            case percentOutput:
                break;
        }            

        builder.addDoubleProperty(constants.name() + " Applied Output", this::getAppliedOutput, null);
    }
}