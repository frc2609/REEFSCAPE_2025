package frc.robot.utils;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class PositionControlledMotor {
    private final TalonFX motor;
    private final TalonFX followerMotor;  // Can be null if no follower
    private final IEncoder encoder;
    private ProfiledPIDController pidController;
    private final VoltageOut voltageRequest;
    private ArmFeedforward feedforward;
    
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

        // Initialize PID controller with custom motion profile values
        pidController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
        );

        // Initialize feedforward if parameters are provided
        if (kS != 0 || kG != 0 || kV != 0) {
            feedforward = new ArmFeedforward(kS, kG, kV);
        }

        // Set tolerance on PID controller
        pidController.setTolerance(positionTolerance);

        // if (encoder instanceof DutyCycleAdapter){
        //     System.out.println("DutyCycle Detected");
        //     pidController.enableContinuousInput(0.0, 1.0);
        // }
    }

public void setP(double P){
pidController.setP(P);
}





    public void goToPosition(double targetPosition) {
        // Clamp target to valid range
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
        
        double currentPosition = getPosition();
        double output = pidController.calculate(currentPosition, targetPosition);

        // Add feedforward if enabled
        if (feedforward != null) {
            var setpoint = pidController.getSetpoint();
            output += feedforward.calculate(setpoint.position, setpoint.velocity);
        }

        // Add debug output
        SmartDashboard.putNumber(name + " Target Position", targetPosition);
        SmartDashboard.putNumber(name + " Current Position", currentPosition);
        SmartDashboard.putNumber(name + " Total Output", output);
        
        // Hard stop at limits
        if (currentPosition <= minPosition && output < 0) {
            output = 0;
        } else if (currentPosition >= maxPosition && output > 0) {
            output = 0;
        }
        
        setVoltage(output);
    }

    public void setVoltage(double volts) {
        // Only allow voltage if it won't drive us past limits
        double position = getPosition();
        if ((position <= minPosition && volts < 0) ||
            (position >= maxPosition && volts > 0)) {
            stop();
            return;
        }
        motor.setControl(voltageRequest.withOutput(volts));
        if (followerMotor != null) {
            followerMotor.setControl(voltageRequest.withOutput(volts));
        }
    }

    public void stop() {
        motor.stopMotor();
        if (followerMotor != null) {
            followerMotor.stopMotor();
        }
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean atPosition() {
        return pidController.atGoal();
        // return Math.abs(getPosition() - pidController.getGoal().position) <= positionTolerance;
    }
}