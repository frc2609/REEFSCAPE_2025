package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import edu.wpi.first.wpilibj.Timer;

public class PositionControlledMotor {
    public final TalonFX motor;
    private final TalonFX followerMotor;  // Can be null if no follower
    public final MotionMagicDutyCycle motor_motionMagicReq = new MotionMagicDutyCycle(0);
    private double targetPosition;

    
    private final double minPosition;
    private final double maxPosition;
    private final String name;
    private final double positionTolerance = 0.01;  // New field for position tolerance

    // Default motion profile values
    private static final double DEFAULT_MAX_VELOCITY = 2.0;    // rotations per second
    private static final double DEFAULT_MAX_ACCELERATION = 1.5; // rotations per second squared

    // Constructor for PID only with single motor
    public PositionControlledMotor(
            TalonFX motor, IEncoder encoder,
            double kP, double kI, double kD,
            double minPosition, double maxPosition, String name) {
        this(motor, null, encoder, kP, kI, kD, 0, 0, 0, minPosition, maxPosition, name);
    
    }



    // Constructor for PID with FF with single motor
    public PositionControlledMotor(
            TalonFX motor, IEncoder encoder,
            double kP, double kI, double kD,
            double kS, double kG, double kV,
            double minPosition, double maxPosition, String name) {
        this(motor, null, encoder, kP, kI, kD, kS, kG, kV, minPosition, maxPosition, name);
    }

    // Constructor for PID with follower motor
    public PositionControlledMotor(
            TalonFX motor, TalonFX followerMotor, IEncoder encoder,
            double kP, double kI, double kD,
            double minPosition, double maxPosition, String name) {
        this(motor, followerMotor, encoder, kP, kI, kD, 0,0, 0, minPosition, maxPosition, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCELERATION, name);
    }

    // Constructor for PID with FF and follower motor
    public PositionControlledMotor(
            TalonFX motor, TalonFX followerMotor, IEncoder encoder,
            double kP, double kI, double kD,
            double kS, double kG, double kV,
            double minPosition, double maxPosition, String name) {
        this(motor, followerMotor, encoder, kP, kI, kD, kS, kG, kV, minPosition, maxPosition, DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCELERATION, name);
    }

    // Full constructor with all parameters
    public PositionControlledMotor(
            TalonFX motor, TalonFX followerMotor, IEncoder encoder,
            double kP, double kI, double kD,
            double kS, double kG, double kV,
            double minPosition, double maxPosition,
            double maxVelocity, double maxAcceleration, String name) {
        this.motor = motor;
        this.followerMotor = followerMotor;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.name = name;

        TalonFXConfiguration cfg = new TalonFXConfiguration();
  

        // Configure Motion Magic with simpler values for testing
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(10) // Start with very low value
          .withMotionMagicAcceleration(5)   // Start with very low value
          .withMotionMagicJerk(0.0);

        Slot0Configs slot0 = cfg.Slot0;
        // FF values 
        slot0.kS = kS; // Add 0.25 V output to overcome static friction
        slot0.kG = kG; // Overcome gravity
        slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output

        // pid values
        slot0.kP = 0.2; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0.0; // No output for integrated error
        slot0.kD = 0.0000001; // A velocity error of 1 rps results in 0.5 V output

        // Apply the configuration
        StatusCode configStatus = motor.getConfigurator().apply(cfg);
        if (!configStatus.isOK()) {
            System.out.println("Failed to configure motor: " + configStatus.toString());
        }

        // Configure follower if present
        if (followerMotor != null) {
            followerMotor.setControl(motor_motionMagicReq.withPosition(0).withSlot(0));
        }
    }

    public void goToPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        // Clamp target to valid range
        // targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
        
        double currentPosition = getPosition();
       
        // Add debug output
        SmartDashboard.putNumber(name + " Target Position", targetPosition);
        SmartDashboard.putNumber(name + " Current Position", currentPosition);
        
        // Ensure the motor is enabled and set the position
        StatusCode res = motor.setControl(
            motor_motionMagicReq
                .withPosition(this.targetPosition) // Convert rotations to sensor units
                .withSlot(0)
        );
        
        SmartDashboard.putString("Status code", res.getDescription());
        
        // Add more debugging information
        SmartDashboard.putNumber(name + " Velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(name + " Current", motor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber(name + " Voltage", motor.getMotorVoltage().getValueAsDouble());
    }

    public void stop() {
        motor.stopMotor();
        if (followerMotor != null) {
            followerMotor.stopMotor();
        }
    }

    public double getPosition() {
        return this.motor.getPosition().getValueAsDouble();
    }

    public boolean atPosition() {
        return (Math.abs(this.targetPosition - getPosition()) > positionTolerance);
    }
}