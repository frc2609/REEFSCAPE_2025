package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.PositionControlledMotor;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class IntakeFlop extends PositionControlledMotor {
    private final static double zeroPosition = 0.39;
    private final static Double gearRatio = 88.888;
    private final static int motorId = 9;
    private final static int encoderId = 3;
    private final static DutyCycleEncoder encoder = new DutyCycleEncoder(encoderId, 1, zeroPosition);
    private final static double positionTolerance = 0.01;
    private final static double maxAcceleration = 300;
    private final static Boolean invertEncoder = false;
    private final static double maxVelocity = 1000;
    private final static double minPosition = -20;
    private final static double maxPosition = 400;
    private final static String name = "IntakeFlop";

    public final Trigger deployedTrigger;
    public final Trigger coralTriggrt;

    public static TalonFXConfiguration talonConfig = 
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(maxVelocity)
                    .withMotionMagicAcceleration(maxAcceleration)
                    .withMotionMagicJerk(10000)
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
                    .withGravityType(GravityTypeValue.Arm_Cosine)
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
                    .withSupplyCurrentLimit(120)
            );
        
    private final DigitalInput intakeBeam = new DigitalInput(4);
    public IntakeFlop() {
        super(
        talonConfig,
        encoder, 
        invertEncoder,
        name,
        motorId,
        gearRatio,
        positionTolerance,
        true);

        deployedTrigger = new Trigger(() -> getPosition() >= 90);
        coralTriggrt = new Trigger(() -> coralPresent());

        setPosition();
    }

    public boolean coralPresent() {
        return intakeBeam.get();
    }

    @Override
    public void setPosition() {
        motor.setPosition(0);
    }

}
