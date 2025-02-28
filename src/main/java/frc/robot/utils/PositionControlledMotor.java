package frc.robot.utils;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PositionControlledMotor extends SubsystemBase {
    public final TalonFX motor;
    public final DutyCycleEncoder encoder;
    public Boolean debug = false;
    public ProfiledPIDController pidController;

    private final String name;
    private final double positionTolerance;
    private final double minPosition;
    private final double maxPosition;
    
    public TalonFX followerMotor = null;
    private MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0).withSlot(0);

    public PositionControlledMotor(String name, TalonFX motor, DutyCycleEncoder encoder, ProfiledPIDController pidController, double positionTolerance, double minPosition, double maxPosition, double zeroPosition) {
        this.name = name;
        this.motor = motor;
        this.encoder = encoder;
        this.positionTolerance = positionTolerance;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.pidController = pidController;
        
        configureEncoder();
        configureMotor();
    }

    public PositionControlledMotor(String name, TalonFX motor, TalonFX followerMotor, DutyCycleEncoder encoder, ProfiledPIDController pidController, double positionTolerance, double minPosition, double maxPosition, double zeroPosition) {
        this.name = name;
        this.motor = motor;
        this.followerMotor = followerMotor;
        this.encoder = encoder;
        this.positionTolerance = positionTolerance;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;

        configureEncoder();
        configureMotor();
    }

    public void goToPosition(double targetPosition) {
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);// Clamp target to a valid range
        motionMagicDutyCycle.withPosition(targetPosition);

        motor.setControl(motionMagicDutyCycle);

        if (followerMotor != null){
            followerMotor.setControl(motionMagicDutyCycle);
        }
    }

    @Override
    public void periodic() {
        // Use getter methods to access subclass data
        if (debug) {
            SmartDashboard.putNumber(name + " Position", getPosition());
            SmartDashboard.putNumber(name + " Velocity", getVelocity());
            SmartDashboard.putNumber(name + " Current", getCurrent());
            SmartDashboard.putNumber(name + " Voltage", getVoltage());
            SmartDashboard.putNumber(name + " Abs pos", getAbsPosition());
        }
    }

    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }
    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }
    public double getCurrent(){
        return motor.getTorqueCurrent().getValueAsDouble();
    }
    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
    public double getOffset(double conversionFactor) {
        return getAbsPosition() * conversionFactor;
    }
    public double getAbsPosition() {
        return encoder.get();
    }
    public boolean atPosition(double targetPosition) {
        double error = Math.abs(targetPosition - getPosition());
        return error <= positionTolerance;
    }

    public void stop() {
        motor.stopMotor();
        if (followerMotor != null){
            followerMotor.stopMotor();
        }
    }
    public void setVoltage(double volts) {
        VoltageOut voltageRequest = new VoltageOut(0);
        double position = getPosition();
        
        // Only allow voltage if it won't drive us past limits
        if ((position <= minPosition && volts < 0) || (position >= maxPosition && volts > 0)) {   
            stop();
            return;
        }

        motor.setControl(voltageRequest.withOutput(volts));
        if (followerMotor != null){
            followerMotor.setControl(voltageRequest);
        }
    }

    public void resetPosition() {
        motor.setPosition(0);
        // If you have a follower motor, reset its position as well.
        if (followerMotor != null) {
            followerMotor.setPosition(0);
        }
    }
    

    // Abstract methods for subclasses to implement
    public abstract void configureMotor();
    public abstract void configureEncoder();
}