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

public class Climber extends PositionControlledMotor{
    // Move the following to config and get gear ratios.
    // We changed how the motor is reset please check the position values
    private final static double positionTolerance = 0.01;
    private final static double maxAcceleration = 50;
    private final static Boolean invertEncoder = false;
    private final static double maxVelocity = 100;
    private final static double minPosition = -200;
    private final static double maxPosition = 400;
    private final static String name = "Climber";
    private final static Double gearRatio = 648.148;
    private final static int motorId = 5;
    private final static int encoderID = 0;
    private final static double zeroPosition = 0.341;

    private final static DutyCycleEncoder encoder = new DutyCycleEncoder(encoderID, 1, zeroPosition);
    private static TalonFXConfiguration talonConfig = 
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
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
                    .withKG(0)
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

    public Climber() {
        super(
            talonConfig,
            encoder,
            invertEncoder,
            name,
            motorId,
            gearRatio,
            positionTolerance,
            true);
    }
}