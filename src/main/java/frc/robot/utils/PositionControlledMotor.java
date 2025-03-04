package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public abstract class PositionControlledMotor extends SubsystemBase {
    protected Boolean debug = false;

    private final NetworkTable configTable;
    private final TalonFX followerMotor;
    private final TalonFX motor;
    private final String name;

    // Abstract methods for subclasses to implement
    protected abstract TalonFXConfiguration getMotorConfig();
    protected abstract Double getEncoderConversion();
    protected abstract double getPositionTolerance();
    protected abstract DutyCycleEncoder getEncoder();

    public PositionControlledMotor(String name, TalonFX motor, boolean debug) {
        this(name, motor, null, debug);
    }

    
    public PositionControlledMotor(String name, TalonFX motor, TalonFX follower, boolean debug) {
        this.name = name;
        this.motor = motor;
        this.followerMotor = follower;
        this.debug = debug;
        configTable = NetworkTableInstance.getDefault().getTable("MotorConfig/" + name);
        
        TalonFXConfiguration talonConfig = getMotorConfig();
        
        configureMotor(talonConfig);
        
        if (debug) {
            setElasticValues(talonConfig);
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

        configTable.getEntry("UpdateConfig").setBoolean(false);
    }

    public void goToPosition(double targetPosition) {
        MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0).withSlot(0);

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

    private boolean updatePressed(){
        return configTable.getEntry("UpdateConfig").getBoolean(false);
    }

    @Override
    public void periodic() {
        if (debug) {
            SmartDashboard.putNumber(name + " Position", getPosition());
            SmartDashboard.putNumber(name + " Velocity", getVelocity());
            SmartDashboard.putNumber(name + " Current", getCurrent());
            SmartDashboard.putNumber(name + " Voltage", getVoltage());
            SmartDashboard.putNumber(name + " Abs pos", getAbsPosition());
            
            
            if (updatePressed()) {
                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                motor.getConfigurator().refresh(talonConfig);   
                
                getConfigFromElastic(talonConfig);

                updateTalonConfig(talonConfig);

                resetUpdateStatus();
            }
        }
    }
    private void getConfigFromElastic(TalonFXConfiguration talonConfig) {
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
    }
                
    private void resetUpdateStatus() {
        configTable.getEntry("UpdateConfig").setBoolean(false);
    }
    protected double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }
    protected double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }
    protected double getCurrent(){
        return motor.getStatorCurrent().getValueAsDouble();
    }
    protected double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
    protected double getOffset() {
        double offset = getAbsPosition();
        if (offset > 0.5){
            offset -= 1;
        }
        return offset * getEncoderConversion();
    }
    protected double getAbsPosition() {
        DutyCycleEncoder encoder = getEncoder();
        return encoder.get();
    }
    public boolean atPosition(double targetPosition) {
        double error = Math.abs(motor.getClosedLoopError().getValueAsDouble());
        return error <= getPositionTolerance();
        // double velocity = Math.abs(motor.getVelocity().getValueAsDouble());
        // return error <= positionTolerance && velocity <= velocityTolerance;
    }

    public void stop() {
        motor.stopMotor();
        if (followerMotor != null){
            followerMotor.stopMotor();
        }
    }
    protected void setVoltage(double volts) {
        VoltageOut voltageRequest = new VoltageOut(0);
        double position = getPosition();

        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        motor.getConfigurator().refresh(softLimitConfigs);

        double minPosition = softLimitConfigs.ReverseSoftLimitThreshold;
        double maxPosition = softLimitConfigs.ForwardSoftLimitThreshold;
        
        if ((position <= minPosition && volts < 0) || (position >= maxPosition && volts > 0)) {   
            stop();
            return;
        }

        motor.setControl(voltageRequest.withOutput(volts));
        if (followerMotor != null){
            followerMotor.setControl(voltageRequest);
        }
    }

    protected void resetPosition() {
        StatusCode stat = motor.setPosition(0);
        SmartDashboard.putString("Reset status: ", stat.getDescription());
        SmartDashboard.putNumber("posAfterReset", getPosition());

        if (followerMotor != null) {
            followerMotor.setPosition(0);
        }
    }
    
    protected void updateTalonConfig(TalonFXConfiguration talonConfig) {
        motor.getConfigurator().apply(talonConfig);

        if(followerMotor != null){
            followerMotor.getConfigurator().apply(talonConfig);
        }
    }

    protected void configureMotor(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
        if (followerMotor != null){
            followerMotor.getConfigurator().apply(config);
        }
    }
    public void setPosition() {
        motor.setPosition(getOffset());
        
        if (followerMotor != null) {
            followerMotor.setPosition(getOffset());
        }    
    }
}