package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.utils.PositionControlledMotor;
import frc.robot.utils.Constants;

public class Climber extends PositionControlledMotor{
    public final static double zeroPosition = 0.35;
    

    public final static double positionTolerance = 0.01;
    private final static double maxAcceleration = 5;
    private final static Boolean invertEncoder = false;
    private final static double maxVelocity = 10;
    private final static double minPosition = -200;
    private final static double maxPosition = 200;
    private final static String name = "Climber";
    
    public Climber() {
        super(
            name, 
            new TalonFX(5, Constants.CANBUS), 
            new DutyCycleEncoder(0), 
            new ProfiledPIDController(
                10, 0, 0,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
            ),
            positionTolerance, minPosition, maxPosition, zeroPosition, true);
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
            .withForwardSoftLimitThreshold(maxPosition)
            .withReverseSoftLimitThreshold(minPosition);
        
        motor.getConfigurator().apply(talonConfig);
    }
        @Override
    public TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive);

        talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicAcceleration(maxAcceleration)
            .withMotionMagicJerk(10*maxAcceleration);

        talonConfig.Slot0
            .withKP(.5)
            .withKI(0)
            .withKD(0);

        talonConfig.Slot0
            .withKS(0)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        talonConfig.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxPosition)
            .withReverseSoftLimitThreshold(minPosition);

        return talonConfig;
    }
}