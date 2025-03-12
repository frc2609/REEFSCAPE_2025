package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.utils.PositionControlledMotor;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Elevator extends PositionControlledMotor {
    // Move the following to config and get gear ratios.
    // We changed how the motor is reset please check the position values
    private final static double positionTolerance = 1.0;
    private final static double maxAcceleration = 100;
    private final static Boolean invertEncoder = true;
    private final static double maxVelocity = 500;
    private final static String name = "ELEVATOR";
    private final static double minPosition = 0;
    private final static double maxPosition = 37.1;
    private final static int encoderId = 2;
    private final static int motorId = 60;
    private final static int followerId = 61;
    private final static Double gearRatio = 9.921;
    private final static double zeroPosition = -0.466;
    private final double rotationsPerInch = 0.905;

    public final Trigger aboveIntake = new Trigger(() -> getPosition() >= 8);

    private final static DutyCycleEncoder encoder = new DutyCycleEncoder(encoderId, 1, zeroPosition);
    public static TalonFXConfiguration talonConfig = 
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
                    .withMotionMagicJerk(10000)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(.2)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0.2)
                    .withKG(0.03)
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
                    .withSupplyCurrentLimit(20)
            );
    
    public Elevator() {
        super(
            talonConfig,
            encoder,
            invertEncoder,
            name,
            motorId,
            followerId,
            gearRatio,
            positionTolerance,
            true);
    }

    @Override
    public void goToPosition(double targetHeightInInches) {
        double targetRotations = targetHeightInInches * rotationsPerInch;
        double targetDegrees = targetRotations * (360.0 / gearRatio);

        super.goToPosition(targetDegrees);
    }

    @Override
    public double getPosition() {
        double currentDegrees = super.getPosition();
        double currentRotations = currentDegrees / (360.0 / gearRatio);

        return currentRotations / rotationsPerInch;
    }

    @Override
    public void setPosition() {
        StatusCode stat = motor.setPosition(0);

        SmartDashboard.putString("Reset status: ", stat.getDescription());
        SmartDashboard.putNumber("posAfterReset", getPosition());
        
        if (followerMotor != null) {
            stat = followerMotor.setPosition(0);
            SmartDashboard.putString("Follower Reset status: ", stat.getDescription());
        SmartDashboard.putNumber("Follower posAfterReset", getPosition());
        }    
    }

}