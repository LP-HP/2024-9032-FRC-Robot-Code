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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
        boolean burnFlash = false;

        switch (constants.mode()) {
            case position:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kPositionOnly);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor()) {
                    checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));

                    burnFlash = true;
                }
                break;
            case positionLeader:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kPositionOnly, true);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor()) {
                    checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));

                    burnFlash = true;
                }
                break;
            case velocity:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kVelocityOnly);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor()) {
                    checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units??

                    burnFlash = true;
                }
                break;
            case velocityLeader:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kVelocityOnly, true);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor()) {
                    checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units??

                    burnFlash = true;
                }
                break;
            case percentOutput:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kMinimal);
                break;
            case velocityControlWithPositionData:
                CANSparkMaxUtil.setCANSparkMaxBusUsage(this, Usage.kAll);
                if(relativeEncoder.getPositionConversionFactor() != constants.positionConversionFactor()) {
                    checkError(relativeEncoder.setPositionConversionFactor(constants.positionConversionFactor()));
                    checkError(relativeEncoder.setVelocityConversionFactor(constants.positionConversionFactor() / 60.0));//TODO units??

                    burnFlash = true;
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

        if(constants.pidConstants() != null) {
            if(setkPIfChanged(constants.pidConstants().kP()) || 
            setkIIfChanged(constants.pidConstants().kI()) ||
            setkDIfChanged(constants.pidConstants().kD()) ||
            setkFIfChanged(constants.pidConstants().kF()))
                burnFlash = true;
        }

        if(getVoltageCompensationNominalVoltage() != constants.nominalVoltage()) {
            checkError(enableVoltageCompensation(constants.nominalVoltage()));

            burnFlash = true;
        }

        if(burnFlash) {
            //TODO checkError(burnFlash());

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
        return absoluteEncoder != null ? absoluteEncoder.getPosition() : 0;
    }

    private void checkError(REVLibError error) {
        if(error != REVLibError.kOk) 
            System.err.println(constants.name() + " has error " + error);
    }

    private boolean setkPIfChanged(double kP) {
        if(controller.getP() != kP) {
            checkError(controller.setP(kP));

            System.out.println("Changed " + constants.name() + " kP to " + kP);

            return true;
        }

        return false;
    } 

    private boolean setkIIfChanged(double kI) {
        if(controller.getI() != kI) {
            checkError(controller.setI(kI));

            System.out.println("Changed " + constants.name() + " kI to " + kI);

            return true;
        }

        return false;
    } 

    private boolean setkDIfChanged(double kD) {
        if(controller.getD() != kD) {
            checkError(controller.setD(kD));

            System.out.println("Changed " + constants.name() + " kD to " + kD);

            return true;
        }

        return false;
    } 

    private boolean setkFIfChanged(double kF) {
        if(controller.getFF() != kF) {
            checkError(controller.setFF(kF));

            System.out.println("Changed " + constants.name() + " kF to " + kF);

            return true;
        }

        return false;
    } 

    private void initPIDSendable(SendableBuilder builder) {
        if(Constants.enablePIDTuning) {
            builder.addDoubleProperty("kP", controller::getP, this::setkPIfChanged);
            builder.addDoubleProperty("kI", controller::getI, this::setkIIfChanged);
            builder.addDoubleProperty("kD", controller::getD, this::setkDIfChanged);
            builder.addDoubleProperty("kF", controller::getFF, this::setkFIfChanged);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        switch (constants.mode()) {
            case position:
            case positionLeader:
                builder.addDoubleProperty("Position", relativeEncoder::getPosition, null);
                builder.addDoubleProperty("Absolute Position", this::getAbsolutePosition, null);
                builder.addDoubleProperty("Target Position", () -> closedLoopSetpoint, null);
                initPIDSendable(builder);
                break;
            case velocity:
            case velocityLeader:
                builder.addDoubleProperty("Velocity", relativeEncoder::getVelocity, null);
                builder.addDoubleProperty("Target Velocity", () -> closedLoopSetpoint, null);
                initPIDSendable(builder);
                break;
            case velocityControlWithPositionData:
                builder.addDoubleProperty("Position", relativeEncoder::getPosition, null);
                builder.addDoubleProperty("Velocity", relativeEncoder::getVelocity, null);
                builder.addDoubleProperty("Target Velocity", () -> closedLoopSetpoint, null);
                initPIDSendable(builder);
                break;
            case percentOutput:
                break;
        }   

        builder.addDoubleProperty("Percent Output", this::getAppliedOutput, this::setIfInTestMode);
        builder.setSafeState(() -> set(0.0));//Runs when test mode is exited
    }

    private void setIfInTestMode(double percent) {
        /* Live window is enabled in test mode */
        if(LiveWindow.isEnabled()) {
            set(percent);
        }
    }
}