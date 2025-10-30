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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Arm extends PositionControlledMotor{
    // Move the following to config and get gear ratios.
    // We changed how the motor is reset please check the position values
    private final static boolean debug = false;
    private final static double positionTolerance = 0.01;
    private final static double maxAcceleration = 325;//200
    private final static Boolean invertEncoder = true;
    private final static double maxVelocity = 270;
    private final static double minPosition = -360;
    private final static double maxPosition = 360;
    private final static String name = "Arm";
    private final static Double encoderRatio = 25.0;
    private final static Double gearRatio = 50.0;
    private final static int motorId = 7;
    private final static int encoderId = 1;
    private final static double zeroPosition =-0.1; // 0.64;

    public final Trigger aboveIntake = new Trigger(() -> getPosition() < -20);
    public final Trigger belowIntake = new Trigger(() -> getPosition() > -20);
    public final Trigger aboveCoral = new Trigger(() -> getPosition() > 10);
    
    private final static DutyCycleEncoder encoder = new DutyCycleEncoder(encoderId, 1, zeroPosition);
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
                    .withMotionMagicJerk(10000)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(.05)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0.01)
                    .withKG(0.015)
                    .withKV(0)
                    .withKA(0)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withReverseSoftLimitEnable(false)
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
            talonConfig,
            encoder,
            invertEncoder,
            name,
            motorId,
            gearRatio,
            encoderRatio,
            positionTolerance,
            debug);
    } 
}
