package frc.robot.utils;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public abstract class PositionControlledMotor extends SubsystemBase {
    protected Boolean debug = false;
    private MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0).withSlot(0);
    private int currentPidSlot = 0;

    private final NetworkTable configTable;
    private final NetworkTable fudgeTable;

    protected final TalonFX followerMotor;
    protected final TalonFX motor;

    private final TalonFXConfiguration talonConfig;
    private final DutyCycleEncoder encoder;
    private final String name;
    private final double positionTolerance;
    private final double gearRatio;
    private final double encoderRatio;
    
    private double minPosition;
    private double fudgeFactor = 0.0;  
    private double maxPosition;
    private double targetPositionDegrees = 0;
    private boolean positionControlEnabled = false;

    public void displayStuff(){};

    public PositionControlledMotor(
        TalonFXConfiguration talonConfig, 
        DutyCycleEncoder encoder, 
        Boolean invertEncoder,
        String name, 
        int motorId, 
        double gearRatio,
        double positionTolerance, 
        boolean debug) 
    {
        this(
            talonConfig,
            encoder,
            invertEncoder,
            name,
            motorId,
            -1,
            gearRatio,
            -1,
            positionTolerance,
            debug);
    }

    
    public PositionControlledMotor(
        TalonFXConfiguration talonConfig, 
        DutyCycleEncoder encoder, 
        Boolean invertEncoder,
        String name, 
        int motorId, 
        int followerId,
        double gearRatio,
        double positionTolerance, 
        boolean debug) 
    {
        this(
            talonConfig,
            encoder,
            invertEncoder,
            name,
            motorId,
            followerId,
            gearRatio,
            -1,
            positionTolerance,
            debug);
    }


    
    public PositionControlledMotor(
        TalonFXConfiguration talonConfig, 
        DutyCycleEncoder encoder, 
        Boolean invertEncoder,
        String name, 
        int motorId, 
        double gearRatio,
        double encoderRatio,
        double positionTolerance, 
        boolean debug) 
    {
        this(
            talonConfig,
            encoder,
            invertEncoder,
            name,
            motorId,
            -1,
            gearRatio,
            encoderRatio,
            positionTolerance,
            debug);
    }

    public PositionControlledMotor(
        TalonFXConfiguration talonConfig, 
        DutyCycleEncoder encoder, 
        Boolean invertEncoder,
        String name, 
        int motorId, 
        int followerId,
        double gearRatio,
        double encoderRatio,
        double positionTolerance, 
        boolean debug) 
    {
        this.talonConfig = talonConfig;
        this.encoder = encoder;
        this.name = name;
        this.gearRatio = gearRatio;
        this.positionTolerance = positionTolerance;
        this.debug = debug;

        encoder.setInverted(invertEncoder);

        this.motor = new TalonFX(motorId, Constants.CANBUS);

        if (followerId != -1){
            followerMotor = new TalonFX(followerId, Constants.CANBUS);
        } else {
            followerMotor = null;
        }
        

        if(encoderRatio == -1 ){
            this.encoderRatio = gearRatio;
        } else {
            this.encoderRatio = encoderRatio;
        }

        configTable = NetworkTableInstance.getDefault().getTable("MotorConfig/" + name);
        fudgeTable = NetworkTableInstance.getDefault().getTable("Fudge");
        fudgeTable.getEntry("fudgeFactor " + name).setDouble(fudgeFactor);

        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
        motor.getConfigurator().refresh(softLimitConfigs);

        minPosition = softLimitConfigs.ReverseSoftLimitThreshold;
        maxPosition = softLimitConfigs.ForwardSoftLimitThreshold;

        configureMotor();
        
        if (debug) {
            setElasticValues();
        }
        setPosition();
    }

    protected double degreesToRotations(double degrees) {
        return degrees / (360 / gearRatio);
    }

    protected double rotationsToDegrees(double rotations) {
        return rotations * (360 / gearRatio);
    }

    public void setNeutralMode(NeutralModeValue mode){
        motor.setNeutralMode(mode);
        if (followerMotor != null){
            followerMotor.setNeutralMode(mode);
        }
    }

    private void setElasticValues(){
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








    public void enablePositionControl() {
        positionControlEnabled = true;
        targetPositionDegrees = getPosition(); 
    }

    public void disablePositionControl() {
        positionControlEnabled = false;
        stop(); 
    }

    public boolean isPositionControlEnabled() {
        return positionControlEnabled;
    }

    public void goToPosition(double targetPositionDegrees) {
        this.targetPositionDegrees = targetPositionDegrees + fudgeFactor;
        if (!positionControlEnabled) {
            enablePositionControl(); 
        }
        updatePosition();
    }

    private void updatePosition() {
        if (!positionControlEnabled) {
            return; 
        }
        double targetPosition = degreesToRotations(targetPositionDegrees);

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
        displayStuff();
     fudgeFactor = fudgeTable.getEntry("fudgeFactor " + name).getDouble(fudgeFactor);
     //System.out.println(fudgeFactor);
        updatePosition();

        if (debug) {
            SmartDashboard.putNumber(name + " Position", getPosition());
            SmartDashboard.putNumber(name + " Velocity", getVelocity());
            SmartDashboard.putNumber(name + " Current", getCurrent());
            SmartDashboard.putNumber(name + " Voltage", getVoltage());
            SmartDashboard.putNumber(name + " Abs pos", getAbsPosition());
            SmartDashboard.putNumber(name + " Raw pos", getRawPosition());
            SmartDashboard.putNumber(name + " Rotor pos", getRotorPosition());
            
            if(followerMotor != null){
                SmartDashboard.putNumber(name + "Follower Position", getPositionFollower());
                SmartDashboard.putNumber(name + "Follower Velocity", getVelocityFollower());
                SmartDashboard.putNumber(name + "Follower Current", getCurrentFollower());
                SmartDashboard.putNumber(name + "Follower Voltage", getVoltageFollower());
                SmartDashboard.putNumber(name + "Follower Raw pos", getRawPositionFollower());
                SmartDashboard.putNumber(name + "Follower Rotor pos", getRotorPositionFollower());
            }
            
            if (updatePressed()) {
                // TalonFXConfiguration newTalonConfig = new TalonFXConfiguration();
                motor.getConfigurator().refresh(talonConfig);   
                
                getConfigFromElastic();

                updateTalonConfig();

                resetUpdateStatus();
            }
        }
    }
    private void getConfigFromElastic() {
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
    public double getPosition(){
        double rotationPosition = motor.getPosition().getValueAsDouble();
        return rotationsToDegrees(rotationPosition);
    }
    public double getRawPosition() {
        return motor.getPosition().getValueAsDouble();
    }
    public double getRotorPosition() {
        return motor.getRotorPosition().getValueAsDouble();
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
    protected double getAbsPosition() {
        return encoder.get();
    }
    public double getPositionFollower(){
        if (followerMotor != null){
            double rotationPosition = followerMotor.getPosition().getValueAsDouble();
            return rotationsToDegrees(rotationPosition);
        } else {
            return -1;
        }
    }
    public double getRawPositionFollower() {
        if (followerMotor != null){

            return followerMotor.getPosition().getValueAsDouble();
        } else {
            return -1;
        }
    }
    public double getRotorPositionFollower() {
        if (followerMotor != null){

            return followerMotor.getRotorPosition().getValueAsDouble();
        } else {
            return -1;
        }
    }
    protected double getVelocityFollower(){
        if (followerMotor != null){

            return followerMotor.getVelocity().getValueAsDouble();
        } else {
            return -1;
        }
    }
    protected double getCurrentFollower(){
        if (followerMotor != null){

            return followerMotor.getStatorCurrent().getValueAsDouble();
        } else {
            return -1;
        }
    }
    protected double getVoltageFollower(){
        if (followerMotor != null){

            return followerMotor.getMotorVoltage().getValueAsDouble();
        } else {
            return -1;
        }
    }
    protected double getOffset() {
        double offset = getAbsPosition();
        if (offset > 0.5){
            offset -= 1;
        }
        return offset * encoderRatio;
    }
    public boolean atPosition() {
        double error = Math.abs(motor.getClosedLoopError().getValueAsDouble());
        return error <= positionTolerance;
        // double velocity = Math.abs(motor.getVelocity().getValueAsDouble());
        // return error <= positionTolerance && velocity <= velocityTolerance;
    }

    public void stop() {
        motor.stopMotor();
        if (followerMotor != null){
            followerMotor.stopMotor();
        }
    }

    protected void resetPosition() {
        motor.setPosition(0);

        if (followerMotor != null) {
            followerMotor.setPosition(0);
        }
    }
    
    protected void updateTalonConfig() {
        motor.getConfigurator().apply(talonConfig);

        if(followerMotor != null){
            followerMotor.getConfigurator().apply(talonConfig);
        }
    }

    protected void configureMotor() {
        motor.getConfigurator().apply(talonConfig);
        if (followerMotor != null){
            followerMotor.getConfigurator().apply(talonConfig);
        }
    }
    public void setPosition() {
        motor.setPosition(getOffset());
        
        if (followerMotor != null) {
            followerMotor.setPosition(getOffset());
        }    
    }
    public void setFudgeFactor(double fudgeFactor){
        this.fudgeFactor += fudgeFactor;
    }

    public void setActivePidSlot(int slotIndex) {
        if (slotIndex >= 0 && slotIndex <= 2) {
            currentPidSlot = slotIndex;
            motionMagicDutyCycle = new MotionMagicDutyCycle(0).withSlot(currentPidSlot);
            updatePosition();
        }
    }
    
    public int getActivePidSlot() {
        return currentPidSlot;
    }
}