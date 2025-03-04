package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.utils.PositionControlledMotor;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.Constants;

public class Arm extends PositionControlledMotor{
    // Move the following to config and get gear ratios.
    // We changed how the motor is reset please check the position values
    private final double positionTolerance = 0.01;
    private final static double maxAcceleration = 160;
    private final Boolean invertEncoder = false;
    private final static double maxVelocity = 80;
    private final static double minPosition = -33;
    private final static double maxPosition = 3.85;
    private final static String name = "Arm";
    private final Double encoderConversion = 25.0;
    private final static int motorID = 7;
    private final int encoderID = 1;
    private final double zeroPosition = 0.63;

    
    private final DutyCycleEncoder encoder;
    public static TalonFXConfiguration talonConfig = 
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(maxVelocity)
                    .withMotionMagicAcceleration(maxAcceleration)
                    .withMotionMagicJerk(10)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(.5)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0)
                    .withKG(0.06)
                    .withKV(0)
                    .withKA(0)
                    .withGravityType(GravityTypeValue.Elevator_Static)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(maxPosition)
                    .withReverseSoftLimitThreshold(minPosition)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimit(30)
            );

    public Arm() {
        super(
            name, 
            new TalonFX(motorID, Constants.CANBUS), 
            true
        );
        encoder = new DutyCycleEncoder(encoderID, 1, zeroPosition);
        encoder.setInverted(invertEncoder);
    }

    protected TalonFXConfiguration getMotorConfig() {
        return talonConfig;
    }

    protected Double getEncoderConversion(){
        return encoderConversion;
    }

    protected double getPositionTolerance() {
        return positionTolerance;
    }

    protected DutyCycleEncoder getEncoder() {
        return encoder;
    }
}
