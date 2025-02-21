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

public class Climber extends SubsystemBase{

    private final TalonFX motor;
    private final TalonFXConfigurator configurator;
    private final TalonFXConfiguration configs;
    private final Encoder encoder;
    private final EncoderAdapter encoderAdapter;
    private final PositionControlledMotor positionControlledMotor;
    
    public Climber() {
        motor = new TalonFX(5, Constants.CANBUS);
        
        configurator = motor.getConfigurator();
        configs = new TalonFXConfiguration();
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        configurator.apply(configs);
        motor.setNeutralMode(NeutralModeValue.Brake);
        
        // Use two DIO ports for quadrature encoder (channelA is blue, ChannelB is yellow)
        encoder = new Encoder(1, 2);
        
        // Configure encoder
        encoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);  // REV Through Bore has 2048 pulses per revolution
        encoder.setReverseDirection(true);
        encoder.reset();  // Start at 0

        encoderAdapter = new EncoderAdapter(encoder);
        
        // Create position controlled motor with both PID and feedforward
        positionControlledMotor = new PositionControlledMotor(
            motor,  // Adapt TalonFX to MotorController interface
            encoderAdapter,    
            60, 0, 0,//40kp
            -.38, 0,
            "Climber"
        );
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPosition());
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
}