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
import frc.robot.utils.Constants;
import frc.robot.utils.EncoderAdapter;

public class Elevator extends SubsystemBase {
    private final TalonFX primaryMotor;
    private final TalonFX followerMotor;
    private final TalonFXConfigurator primaryConfigurator;
    private final TalonFXConfigurator followerConfigurator;
    private final TalonFXConfiguration configs;
    private final Encoder encoder;
    private final EncoderAdapter encoderAdapter;
    // private final PositionControlledMotor positionControlledMotor;
    private double P = 0.5;
    
    public Elevator() {
        primaryMotor = new TalonFX(Constants.Elevator.PRIMARY_MOTOR_ID, Constants.CANBUS);
        followerMotor = new TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID, Constants.CANBUS);
        
        primaryConfigurator = primaryMotor.getConfigurator();
        followerConfigurator = followerMotor.getConfigurator();
        configs = new TalonFXConfiguration();
        NeutralModeValue neutralMode = NeutralModeValue.Brake;
        
        // Configure primary motor
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        primaryConfigurator.apply(configs);
        primaryMotor.setNeutralMode(neutralMode);
        
        // Configure follower motor
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        followerConfigurator.apply(configs);
        followerMotor.setNeutralMode(neutralMode);
        
        // Use two DIO ports for quadrature encoder
        encoder = new Encoder(6, 7);
        
        // Configure encoder
        encoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);  // REV Through Bore has 2048 pulses per revolution
        encoder.setReverseDirection(false);
        encoder.reset();  // Start at 0

        encoderAdapter = new EncoderAdapter(encoder);
        
        // Create position controlled motor with both motors and feedforward
        // positionControlledMotor = new PositionControlledMotor(
        //     primaryMotor,
        //     followerMotor,
        //     encoderAdapter,
        //     P, Constants.Elevator.kI, Constants.Elevator.kD,
        //     Constants.Elevator.MIN_POSITION, Constants.Elevator.MAX_POSITION,
        //     "Elevator"

        // );
    }

    // @Override 
    // public void periodic() {
    //     SmartDashboard.putNumber("Elevator Position", getPosition());
    // }

    public void goToPosition(double targetPosition) {
        // positionControlledMotor.goToPosition(targetPosition);
    }

    public void stop() {
        // positionControlledMotor.stop();
    }

    // public double getPosition() {
    //     return positionControlledMotor.getPosition();
    // }

    // public boolean atPosition() {
    //     return positionControlledMotor.atPosition();
    // }
} 