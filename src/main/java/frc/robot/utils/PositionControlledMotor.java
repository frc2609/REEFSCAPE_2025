package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public abstract class PositionControlledMotor extends SubsystemBase {
    public final TalonFX motor;
    public final TalonFX followerMotor;
    public final DutyCycleEncoder encoder;
    public final double positionTolerance;
    public final ProfiledPIDController pidController;
    public Boolean debug = false;

    private final String name;
    private MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0).withSlot(0);
    private NetworkTable pidTable;
    private NetworkTable configTable;

    // Abstract methods for subclasses to implement
    public abstract void configureEncoder();
    public abstract TalonFXConfiguration getMotorConfig();
    
    public PositionControlledMotor(String name, TalonFX motor, DutyCycleEncoder encoder, ProfiledPIDController pidController, double positionTolerance, boolean debug) {
        this(name, motor, null, encoder, pidController, positionTolerance, debug);        
    }

    public PositionControlledMotor(String name, TalonFX motor, TalonFX followerMotor, DutyCycleEncoder encoder, ProfiledPIDController pidController, double positionTolerance, boolean debug) {
        this.name = name;
        this.motor = motor;
        this.followerMotor = followerMotor;
        this.encoder = encoder;
        this.pidController = pidController;
        this.positionTolerance = positionTolerance;
        this.debug = debug;
        
        TalonFXConfiguration talonConfig = getMotorConfig();
        
        configureEncoder();
        configureMotor(talonConfig);
        
        if (debug) {
            
            pidTable = NetworkTableInstance.getDefault().getTable("PID/" + name);
            pidTable.getEntry("kP").setDouble(pidController.getP());
            pidTable.getEntry("kI").setDouble(pidController.getI());
            pidTable.getEntry("kD").setDouble(pidController.getD());

            configTable = NetworkTableInstance.getDefault().getTable("MotorConfig/" + name);
            
            setElasticValues(talonConfig);

            configTable.getEntry("UpdateConfig").setBoolean(false);
        }
    }

    private void setElasticValues(TalonFXConfiguration talonConfig){
        configTable.getEntry("kP").setDouble(talonConfig.Slot0.kP);
        configTable.getEntry("kI").setDouble(talonConfig.Slot0.kI);
        configTable.getEntry("kD").setDouble(talonConfig.Slot0.kD);
        configTable.getEntry("kS").setDouble(talonConfig.Slot0.kS);
        configTable.getEntry("kV").setDouble(talonConfig.Slot0.kV);
        configTable.getEntry("kG").setDouble(talonConfig.Slot0.kG);
        configTable.getEntry("kA").setDouble(talonConfig.Slot0.kA);
        configTable.getEntry("maxVelocity").setDouble(talonConfig.MotionMagic.MotionMagicCruiseVelocity);
        configTable.getEntry("maxAcceleration").setDouble(talonConfig.MotionMagic.MotionMagicAcceleration);
        configTable.getEntry("jerk").setDouble(talonConfig.MotionMagic.MotionMagicJerk);
        configTable.getEntry("supply limit").setDouble(talonConfig.CurrentLimits.SupplyCurrentLimit);
        configTable.getEntry("stator limit").setDouble(talonConfig.CurrentLimits.StatorCurrentLimit);
    }

    public void goToPosition(double targetPosition) {
        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        motor.getConfigurator().refresh(softLimitConfigs);

        double minPosition = softLimitConfigs.ReverseSoftLimitThreshold;
        double maxPosition = softLimitConfigs.ForwardSoftLimitThreshold;

        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);// Clamp target to a valid range
        motionMagicDutyCycle.withPosition(targetPosition);

        motor.setControl(motionMagicDutyCycle);

        if (followerMotor != null){
            followerMotor.setControl(motionMagicDutyCycle);
        }
    }

    @Override
    public void periodic() {
        if (debug) {

            // Use getter methods to access subclass data
            SmartDashboard.putNumber(name + " Position", getPosition());
            SmartDashboard.putNumber(name + " Velocity", getVelocity());
            SmartDashboard.putNumber(name + " Current", getCurrent());
            SmartDashboard.putNumber(name + " Voltage", getVoltage());
            SmartDashboard.putNumber(name + " Abs pos", getAbsPosition());

            // Publish motor performance data
            pidTable.getEntry("Position").setDouble(getPosition());
            pidTable.getEntry("Velocity").setDouble(getVelocity());
            pidTable.getEntry("Current").setDouble(getCurrent());
            pidTable.getEntry("Voltage").setDouble(getVoltage());
            pidTable.getEntry("Abs pos").setDouble(getAbsPosition());
            
            
            if (configTable.getEntry("UpdateConfig").getBoolean(false)) {
                System.out.println("Updating config");

                // Update PID values from NetworkTables
                double pid_kP = pidTable.getEntry("kP").getDouble(pidController.getP());
                double pid_kI = pidTable.getEntry("kI").getDouble(pidController.getI());
                double pid_kD = pidTable.getEntry("kD").getDouble(pidController.getD());
                pidController.setPID(pid_kP, pid_kI, pid_kD);

                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                motor.getConfigurator().refresh(talonConfig);   
                
                double p = configTable.getEntry("kP").getDouble(talonConfig.Slot0.kP);
                double i = configTable.getEntry("kI").getDouble(talonConfig.Slot0.kI);
                double d = configTable.getEntry("kD").getDouble(talonConfig.Slot0.kD);
                double s = configTable.getEntry("kS").getDouble(talonConfig.Slot0.kS);
                double v = configTable.getEntry("kV").getDouble(talonConfig.Slot0.kV);
                double g = configTable.getEntry("kG").getDouble(talonConfig.Slot0.kG);
                double a = configTable.getEntry("kA").getDouble(talonConfig.Slot0.kA);
                double maxVelocity = configTable.getEntry("maxVelocity").getDouble(talonConfig.MotionMagic.MotionMagicCruiseVelocity);
                double maxAcceleration = configTable.getEntry("maxAcceleration").getDouble(talonConfig.MotionMagic.MotionMagicAcceleration);
                double jerk = configTable.getEntry("jerk").getDouble(talonConfig.MotionMagic.MotionMagicJerk);
                double supplyCurrentLimit = configTable.getEntry("supply limit").getDouble(talonConfig.CurrentLimits.SupplyCurrentLimit);
                double statorCurrentLimit = configTable.getEntry("stator limit").getDouble(talonConfig.CurrentLimits.StatorCurrentLimit);
                
                talonConfig.Slot0
                    .withKP(p)
                    .withKI(i)
                    .withKD(d)
                    .withKS(s)
                    .withKV(v)
                    .withKG(g)
                    .withKA(a);

                talonConfig.MotionMagic
                    .withMotionMagicCruiseVelocity(maxVelocity)
                    .withMotionMagicAcceleration(maxAcceleration)
                    .withMotionMagicJerk(jerk);
                
                talonConfig.CurrentLimits
                    .withSupplyCurrentLimit(supplyCurrentLimit)
                    .withStatorCurrentLimit(statorCurrentLimit);

                updateTalonConfig(talonConfig);

                configTable.getEntry("UpdateConfig").setBoolean(false);
            }
        }
    }

    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }
    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }
    public double getCurrent(){
        return motor.getStatorCurrent().getValueAsDouble();
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

        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        motor.getConfigurator().refresh(softLimitConfigs);

        double minPosition = softLimitConfigs.ReverseSoftLimitThreshold;
        double maxPosition = softLimitConfigs.ForwardSoftLimitThreshold;
        
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

        if (followerMotor != null) {
            followerMotor.setPosition(0);
        }
    }
    
    public void updateTalonConfig(TalonFXConfiguration talonConfig) {
        motor.getConfigurator().apply(talonConfig);

        if(followerMotor != null){
            followerMotor.getConfigurator().apply(talonConfig);
        }
    }

    public void configureMotor(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
        if (followerMotor != null){
            followerMotor.getConfigurator().apply(config);
        }
    }
}