package frc.robot.utils;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class PositionControlledMotor {
    private final TalonFX motor;
    private final TalonFX followerMotor;  // Can be null if no follower
    private final IEncoder encoder;
    private final VoltageOut voltageRequest;
    private ArmFeedforward feedforward;
    private final MotionMagicDutyCycle motor_motionMagicReq = new MotionMagicDutyCycle(0);
    private double targetPosition;
    private double currentPosition;

    
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
        this.encoder = encoder;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.name = name;
        this.voltageRequest = new VoltageOut(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
  
   /* Configure Motion Magic */
      MotionMagicConfigs mm = cfg.MotionMagic;
      mm.withMotionMagicCruiseVelocity(5000) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(750) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        .withMotionMagicJerk(0.0);
        motor_motionMagicReq.FeedForward = 0.0;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 0.2; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0.0; // No output for integrated error
        slot0.kD = 0.0000001; // A velocity error of 1 rps results in 0.5 V output


        // Initialize PID controller with custom motion profile values
        // pidController = new ProfiledPIDController(
        //     kP, kI, kD,
        //     new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
        // );

        // Initialize feedforward if parameters are provided
        // if (kS != 0 || kG != 0 || kV != 0) {
        //     feedforward = new ArmFeedforward(kS, kG, kV);
        // }

        // Set tolerance on PID controller
        // pidController.setTolerance(positionTolerance);

        // if (encoder instanceof DutyCycleAdapter){
        //     System.out.println("DutyCycle Detected");
        //     pidController.enableContinuousInput(0.0, 1.0);
        // }
    }

public void setP(double P){
// pidController.setP(P);
}





    public void goToPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        // Clamp target to valid range
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
        
        double currentPosition = getPosition();
       // double output = pidController.calculate(currentPosition, targetPosition);

        // Add feedforward if enabled
        // if (feedforward != null) {
        //     var setpoint = pidController.getSetpoint();
        //     output += feedforward.calculate(setpoint.position, setpoint.velocity);
        // }

        // Add debug output
        SmartDashboard.putNumber(name + " Target Position", targetPosition);
        SmartDashboard.putNumber(name + " Current Position", currentPosition);
        //SmartDashboard.putNumber(name + " Total Output", output);
        
        // // Hard stop at limits
        // if (currentPosition <= minPosition && output < 0) {
        //     output = 0;
        // } else if (currentPosition >= maxPosition && output > 0) {
        //     output = 0;
        // }
        
        //(output);
        motor.setControl( motor_motionMagicReq.withPosition(targetPosition).withSlot(0));
    }

    // public void setVoltage(double volts) {
    //     // Only allow voltage if it won't drive us past limits
    //     double position = getPosition();
    //     if ((position <= minPosition && volts < 0) ||
    //         (position >= maxPosition && volts > 0)) {
    //         stop();
    //         return;
    //     }
    //     motor.setControl(voltageRequest.withOutput(volts));
    //     if (followerMotor != null) {
    //         followerMotor.setControl(voltageRequest.withOutput(volts));
    //     }
    // }

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
        // return pidController.atGoal();
        // return Math.abs(getPosition() - pidController.getGoal().position) <= positionTolerance;
    }
}