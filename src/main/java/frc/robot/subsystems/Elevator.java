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

public class Elevator extends SubsystemBase {
    private final TalonFX primaryMotor;
    private final TalonFX followerMotor;
    private final TalonFXConfigurator primaryConfigurator;
    private final TalonFXConfigurator followerConfigurator;
    private final TalonFXConfiguration configs;
    private final Encoder encoder;
    private final PositionControlledMotor positionControlledMotor;
    
    // Constants for elevator positions and limits (in rotations)
    // These values need to be tuned for your specific elevator
    private static final double MIN_POSITION = 0.0;  // Bottom position
    private static final double MAX_POSITION = 5.0;  // Top position (adjust based on your elevator's range)
    
    // PID constants - these will need tuning
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    // private static final double kS = 0.0;  // Static friction
    // private static final double kG = 0.1;  // Gravity compensation
    // private static final double kV = 0.0;  // Velocity feedforward
    
    public Elevator() {
        primaryMotor = new TalonFX(61, "CANivore");
        followerMotor = new TalonFX(62, "CANivore");
        
        primaryConfigurator = primaryMotor.getConfigurator();
        followerConfigurator = followerMotor.getConfigurator();
        configs = new TalonFXConfiguration();
        
        // Configure primary motor
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        primaryConfigurator.apply(configs);
        primaryMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Configure follower motor
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        followerConfigurator.apply(configs);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Use two DIO ports for quadrature encoder
        encoder = new Encoder(3, 4);
        
        // Configure encoder
        encoder.setDistancePerPulse(1.0 / 2048.0);  // REV Through Bore has 2048 pulses per revolution
        encoder.setReverseDirection(true);
        encoder.reset();  // Start at 0
        
        // Create position controlled motor with both motors and feedforward
        positionControlledMotor = new PositionControlledMotor(
            primaryMotor,
            followerMotor,
            encoder,
            kP, kI, kD,
            // kS, kG, kV,
            MIN_POSITION, MAX_POSITION,
            "Elevator"
        );
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
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