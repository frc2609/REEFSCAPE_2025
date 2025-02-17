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

public class Arm extends SubsystemBase{

    private final TalonFX motor;
    private final TalonFXConfigurator configurator;
    private final TalonFXConfiguration configs;
    private final Encoder encoder;
    private final PositionControlledMotor positionControlledMotor;
    
    public Arm() {
        motor = new TalonFX(Constants.Arm.MOTOR_ID, Constants.CANBUS);
        
        configurator = motor.getConfigurator();
        configs = new TalonFXConfiguration();
        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        configurator.apply(configs);
        motor.setNeutralMode(NeutralModeValue.Coast);
        
        // Use two DIO ports for quadrature encoder (channelA is blue, ChannelB is yellow)
        encoder = new Encoder(Constants.Arm.ENCODER_CHANNEL_A, Constants.Arm.ENCODER_CHANNEL_B);
        
        // Configure encoder
        encoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);  // REV Through Bore has 2048 pulses per revolution
        encoder.setReverseDirection(true);
        encoder.reset();  // Start at 0
        
        // Create position controlled motor with both PID and feedforward
        positionControlledMotor = new PositionControlledMotor(
            motor,  // Adapt TalonFX to MotorController interface
            encoder,
            Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD,
            Constants.Arm.MIN_POSITION, Constants.Arm.MAX_POSITION,
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