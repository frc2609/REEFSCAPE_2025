package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Arm extends SubsystemBase{
    /**
     * The motor controller for the robot's arm subsystem.
     */
    private final TalonFX motor;
    private final TalonFXConfigurator configurator;
    private final TalonFXConfiguration configs; 
    private final VoltageOut voltageCtrlReq;
    private final Encoder encoder;
    private final ProfiledPIDController pidController;
    
    // Constants for arm positions and limits (in rotations)
    private static final double MIN_POSITION = -0.369263; 
    private static final double MAX_POSITION = 0.357178;
    
    // PID constants - tune these values
    private static final double kP = 60.0;
    private static final double POSITION_TOLERANCE = 0.015;
    
    
    public Arm() {
        motor = new TalonFX(50, "CANivore");

        configurator = motor.getConfigurator();
        configs = new TalonFXConfiguration();
        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        configurator.apply(configs);

        voltageCtrlReq = new VoltageOut(0);

        motor.setNeutralMode(NeutralModeValue.Coast);
        
        // Use two DIO ports for quadrature encoder (channelA is blue, ChannelB is yellow)
        encoder = new Encoder(1, 2);
        
        // Configure encoder
        encoder.setDistancePerPulse(1.0 / 2048.0);  // REV Through Bore has 2048 pulses per revolution
        encoder.setReverseDirection(true);
        encoder.reset();  // Start at 0
        
        
        // Configure PID controller
        pidController = new ProfiledPIDController(
            kP, 0, 0,  // PID gains
            new TrapezoidProfile.Constraints(2.0, 3.0)  // Max velocity and acceleration
        );
        pidController.setTolerance(POSITION_TOLERANCE);
    }

    private double getPosition() {
        // Get position in rotations, accounting for 2:1 ratio
        return encoder.getDistance() / 2.0;
    }

    @Override 
    public void periodic() {
        double position = getPosition();
        SmartDashboard.putNumber("Arm Position", position);
        
        // Safety check - stop motor if we're at limits
        if ((position <= MIN_POSITION && motor.getMotorVoltage().getValueAsDouble() < 0) ||
            (position >= MAX_POSITION && motor.getMotorVoltage().getValueAsDouble() > 0)) {
            stop();
        }
    }

    public void goToPosition(double targetPosition) {
        // Clamp target to valid range
        targetPosition = Math.min(Math.max(targetPosition, MIN_POSITION), MAX_POSITION);
        
        double currentPosition = getPosition();
        double error = targetPosition - currentPosition;
        
        // Add debug output
        SmartDashboard.putNumber("Arm Target Position", targetPosition);
        SmartDashboard.putNumber("Arm Current Position", currentPosition);
        SmartDashboard.putNumber("Arm Position Error", error);
        
        double output = error * kP;
        
        // Add velocity-based damping to prevent overshoot
        double velocity = encoder.getRate();
        double damping = velocity * 0.1;  // Adjust this factor as needed
        output -= damping;
        
        // Limit output to prevent too high voltage
        output = Math.min(Math.max(output, -6.0), 6.0);
        
        // Add deadband to prevent oscillation
        if (Math.abs(error) < POSITION_TOLERANCE * 2) {
            output *= 0.5;
        }
        
        // Hard stop at limits
        if (currentPosition <= MIN_POSITION && output < 0) {
            output = 0;
        } else if (currentPosition >= MAX_POSITION && output > 0) {
            output = 0;
        }
        
        setVoltage(output);
    }

    public void stop() {
        motor.stopMotor();
    }
    
    public void setVoltage(double volts) {
        // Only allow voltage if it won't drive us past limits
        double position = getPosition();
        if ((position <= MIN_POSITION && volts < 0) ||
            (position >= MAX_POSITION && volts > 0)) {
            stop();
            return;
        }
        motor.setControl(voltageCtrlReq.withOutput(volts));
    }

    public boolean atPosition() {
        return pidController.atGoal();
    }

    public void resetPosition() {
        encoder.reset();  // Reset encoder to 0
    }
}