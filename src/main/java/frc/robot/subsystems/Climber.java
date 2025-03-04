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

public class Climber extends PositionControlledMotor{
    // Move the following to config and get gear ratios.
    // We changed how the motor is reset please check the position values
    private final double positionTolerance = 0.01;
    private final static double maxAcceleration = 50;
    private final Boolean invertEncoder = false;
    private final static double maxVelocity = 100;
    private final static double minPosition = -20;
    private final static double maxPosition = 400;
    private final static String name = "Climber";
    private final Double encoderConversion = 648.148;
    private final static int motorID = 5;
    private final int encoderID = 0;
    public final static double zeroPosition = 0.39;

    private final DutyCycleEncoder encoder;
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


// private final DutyCycleEncoder encoder = new DutyCycleEncoder(encoderID, 1, constants.climber.zeroPosition);
//
// private TalonFXConfiguration talonConfig = 
// new TalonFXConfiguration()
//     .withMotorOutput(
//         new MotorOutputConfigs()
//             .withInverted(constants.climber.inverted)
//             .withNeutralMode(constants.climber.neutralModeValue)
//     )
//     .withMotionMagic(
//         new MotionMagicConfigs()
//             .withMotionMagicCruiseVelocity(constants.climber.maxVelocity)
//             .withMotionMagicAcceleration(constants.climber.maxAcceleration)
//             .withMotionMagicJerk(constants.climber.jerk)
//     )
//     .withSlot0(
//         new Slot0Configs()
//             .withKP(constants.climber.p)
//             .withKG(constants.climber.g)
//             .withGravityType(constants.climber.gravityType)
//     )
//     .withSoftwareLimitSwitch(
//         new SoftwareLimitSwitchConfigs()
//             .withForwardSoftLimitEnable(true)
//             .withReverseSoftLimitEnable(true)
//             .withForwardSoftLimitThreshold(constants.climber.maxPosition)
//             .withReverseSoftLimitThreshold(constants.climber.minPosition)
//     )
//     .withCurrentLimits(
//         new CurrentLimitsConfigs()
//             .withStatorCurrentLimit(constants.climber.statorLimit)
//             .withSupplyCurrentLimit(constants.climber.supplyLimit)
//     );



// public Climber() {
//     super(
//         talonConfig,
//         encoder,
//         constants.Climber.name, 
//         constants.Climber.motorID, 
//         constants.Climber.followerID,
//         constants.Climber.encoderConversion,
//         constants.Climber.positionTolerance
//     );
// }