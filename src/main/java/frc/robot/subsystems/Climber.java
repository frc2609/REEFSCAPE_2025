package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.utils.PositionControlledMotor;
import frc.robot.utils.Constants;

public class Climber extends PositionControlledMotor{

    private final static double positionTolerance = 0.01;
    private final static double maxAcceleration = 5;
    private final static Boolean invertEncoder = false;
    private final static double zeroPosition = 0;
    private final static double maxVelocity = 10;
    private final static double minPosition = -2;
    private final static double maxPosition = 2;
    private final static String name = "Climber";
    
    public Climber() {
        super(
            name, 
            new TalonFX(5, Constants.CANBUS), 
            new DutyCycleEncoder(0), 
            new ProfiledPIDController(
                1, 0, 0,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
            ),
            positionTolerance, minPosition, maxPosition, zeroPosition);
            
        super.debug = true;
    }

    public void configureEncoder() {
        encoder.setInverted(invertEncoder);
    }

    public void configureMotor() {
        motor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive);

        talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicAcceleration(maxAcceleration);

        talonConfig.Slot0
            .withKP(0.2)
            .withKD(0.0000001);

        talonConfig.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(2)
            .withReverseSoftLimitThreshold(-2);
        
        motor.getConfigurator().apply(talonConfig);
    }
}