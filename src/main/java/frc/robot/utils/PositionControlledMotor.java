package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public abstract class PositionControlledMotor extends SubsystemBase {
    public final TalonFX motor;
    public final DutyCycleEncoder encoder;
    public Boolean debug = false;
    public ProfiledPIDController pidController;

    private final String name;
    public final double positionTolerance;
    public final double zeroPosition;
    private final double minPosition;
    private final double maxPosition;
    
    public TalonFX followerMotor = null;
    private MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0).withSlot(0);

    private NetworkTable pidTable;
    private NetworkTable configTable;
    private NetworkTableEntry updateConfigEntry;

    public PositionControlledMotor(String name, TalonFX motor, DutyCycleEncoder encoder, ProfiledPIDController pidController, double positionTolerance, double minPosition, double maxPosition, double zeroPosition, boolean debug) {
        this(name, motor, null, encoder, pidController, positionTolerance, minPosition, maxPosition, zeroPosition, debug);        
    }

    public PositionControlledMotor(String name, TalonFX motor, TalonFX followerMotor, DutyCycleEncoder encoder, ProfiledPIDController pidController, double positionTolerance, double minPosition, double maxPosition, double zeroPosition, boolean debug) {
        this.name = name;
        this.motor = motor;
        this.followerMotor = followerMotor;
        this.encoder = encoder;
        this.pidController = pidController;
        this.positionTolerance = positionTolerance;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.zeroPosition = zeroPosition;
        this.debug = debug;

        configureEncoder();
        // configureMotor();

        if (debug) {

            pidTable = NetworkTableInstance.getDefault().getTable("PID/" + name);
            pidTable.getEntry("kP").setDouble(pidController.getP());
            pidTable.getEntry("kI").setDouble(pidController.getI());
            pidTable.getEntry("kD").setDouble(pidController.getD());

            configTable = NetworkTableInstance.getDefault().getTable("MotorConfig/" + name);
            configTable.getEntry("kP").setDouble(0.5);
            configTable.getEntry("kI").setDouble(0.0);
            configTable.getEntry("kD").setDouble(0.0);
            configTable.getEntry("kS").setDouble(0.0);
            configTable.getEntry("kV").setDouble(0.0);
            configTable.getEntry("kG").setDouble(0.0);
            configTable.getEntry("kA").setDouble(0.0);
            configTable.getEntry("maxVelocity").setDouble(10.0);
            configTable.getEntry("maxAcceleration").setDouble(5.0);

            configTable.getEntry("UpdateConfig").setBoolean(false);
        }
    }

    public void goToPosition(double targetPosition) {
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);// Clamp target to a valid range
        motionMagicDutyCycle.withPosition(targetPosition);

        motor.setControl(motionMagicDutyCycle);

        if (followerMotor != null){
            followerMotor.setControl(motionMagicDutyCycle);
        }
    }

    // @Override
    // public void periodic() {
    //     if (debug) {

            // Use getter methods to access subclass data
            // SmartDashboard.putNumber(name + " Position", getPosition());
            // SmartDashboard.putNumber(name + " Velocity", getVelocity());
            // SmartDashboard.putNumber(name + " Current", getCurrent());
            // SmartDashboard.putNumber(name + " Voltage", getVoltage());
            // SmartDashboard.putNumber(name + " Abs pos", getAbsPosition());

            // Publish motor performance data
            // pidTable.getEntry("Position").setDouble(getPosition());
            // pidTable.getEntry("Velocity").setDouble(getVelocity());
            // pidTable.getEntry("Current").setDouble(getCurrent());
            // pidTable.getEntry("Voltage").setDouble(getVoltage());
            // pidTable.getEntry("Abs pos").setDouble(getAbsPosition());
            
            
            // if (configTable.getEntry("UpdateConfig").getBoolean(false)) {
            //     System.out.println("Updating config");

            //     // Update PID values from NetworkTables
            //     double pid_kP = pidTable.getEntry("kP").getDouble(pidController.getP());
            //     double pid_kI = pidTable.getEntry("kI").getDouble(pidController.getI());
            //     double pid_kD = pidTable.getEntry("kD").getDouble(pidController.getD());
            //     pidController.setPID(pid_kP, pid_kI, pid_kD);
                
            //     double kP = configTable.getEntry("kP").getDouble(0.5);
            //     double kI = configTable.getEntry("kI").getDouble(0.0);
            //     double kD = configTable.getEntry("kD").getDouble(0.0);
            //     double kS = configTable.getEntry("kS").getDouble(0.0);
            //     double kV = configTable.getEntry("kV").getDouble(0.0);
            //     double kG = configTable.getEntry("kG").getDouble(0.0);
            //     double kA = configTable.getEntry("kA").getDouble(0.0);
            //     double maxVelocity = configTable.getEntry("maxVelocity").getDouble(10.0);
            //     double maxAcceleration = configTable.getEntry("maxAcceleration").getDouble(5.0);
                
            //     updateTalonConfig();
            //     updateConfigEntry.setBoolean(false);
            // }
    //     }
    // }

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
        StatusCode stat = motor.setPosition(0);
        SmartDashboard.putString("Reset status: ", stat.getDescription());
        SmartDashboard.putNumber("posAfterReset", getPosition());

        // If you have a follower motor, reset its position as well.
        if (followerMotor != null) {
            followerMotor.setPosition(0);
        }
    }
    

    // Abstract methods for subclasses to implement
    // public abstract void configureMotor();
    public abstract void configureEncoder();

    public abstract TalonFXConfiguration getMotorConfig();
    
    public void updateTalonConfig() {
        System.out.println("Config motor " + name);
        TalonFXConfiguration config = getMotorConfig();
        motor.getConfigurator().apply(config);
    }
}