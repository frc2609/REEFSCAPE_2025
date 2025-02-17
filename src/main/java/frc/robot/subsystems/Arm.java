package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import frc.robot.utils.PositionControlledMotor;

public class Arm extends SubsystemBase{
    /**
     * The motor controller for the robot's arm subsystem.
     */
    private final TalonFX motor;
    private final TalonFXConfigurator configurator;
    private final TalonFXConfiguration configs;
    private final Encoder encoder;
    private final PositionControlledMotor positionControlledMotor;
    
    // Constants for arm positions and limits (in rotations)
    private static final double MIN_POSITION = -0.369263; 
    private static final double MAX_POSITION = 0.357178;
    
    // PID constants - tune these values
    private static final double kP = 60.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    // private static final double kS = 0.0;  // Static friction
    // private static final double kG = 0.3;  // Gravity compensation
    // private static final double kV = 0.0;  // Velocity feedforward
    
    public Arm() {
        motor = new TalonFX(50, "CANivore");
        
        configurator = motor.getConfigurator();
        configs = new TalonFXConfiguration();
        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        configurator.apply(configs);
        motor.setNeutralMode(NeutralModeValue.Coast);
        
        // Use two DIO ports for quadrature encoder (channelA is blue, ChannelB is yellow)
        encoder = new Encoder(1, 2);
        
        // Configure encoder
        encoder.setDistancePerPulse(1.0 / 2048.0);  // REV Through Bore has 2048 pulses per revolution
        encoder.setReverseDirection(true);
        encoder.reset();  // Start at 0
        
        // Create position controlled motor with both PID and feedforward
        positionControlledMotor = new PositionControlledMotor(
            motor,  // Adapt TalonFX to MotorController interface
            encoder,
            kP, kI, kD,
            // kS, kG, kV,
            MIN_POSITION, MAX_POSITION,
            // 2.0, 3.0,  // Max velocity and acceleration
            "Arm"
        );
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getPosition());
    }

    public void goToPosition(double targetPosition) {
        positionControlledMotor.goToPosition(targetPosition);
    }

    public void stop() {
        positionControlledMotor.stop();
    }
    
    public void setVoltage(double volts) {
        positionControlledMotor.setVoltage(volts);
    }

    public double getPosition() {
        return positionControlledMotor.getPosition();
    }

    public boolean atPosition() {
        return positionControlledMotor.atPosition();
    }

    public void resetPosition() {
        positionControlledMotor.resetPosition();
    }
}