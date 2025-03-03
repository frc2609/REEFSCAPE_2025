package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.utils.PositionControlledMotor;
import frc.robot.utils.Constants;

public class Elevator extends PositionControlledMotor {
    public final static double zeroPosition = 0;
    public final static double positionTolerance = 0.1;
    private final static double maxAcceleration = 5;
    private final static double maxVelocity = 10;
    private final static String name = "ELEVATOR";

    private static final double minPosition = 0;
    private static final double maxPosition = 12.9;

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
    
    public Elevator() {
        super(
            name, 
            new TalonFX(60, Constants.CANBUS), 
            new TalonFX(61, Constants.CANBUS), 
            new DutyCycleEncoder(2,1, zeroPosition), 
            new ProfiledPIDController(
                2, 0, 0,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
            ),
            positionTolerance, true
        );

        super.debug = true;

    }

    public void configureEncoder() {
        encoder.setInverted(false);
    }

    public TalonFXConfiguration getMotorConfig() {
        return Elevator.talonConfig;
    }
}


