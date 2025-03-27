package frc.robot.subsystems;

import java.io.PushbackInputStream;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.PositionControlledMotor;


public class IntakeFlop extends PositionControlledMotor {
    private final static boolean debug = true;
    private final static double zeroPosition = 0.39;
    private final static Double gearRatio = 88.888;
    private final static int motorId = 9;
    private final static int encoderId = 3;
    private final static DutyCycleEncoder encoder = new DutyCycleEncoder(encoderId, 1, zeroPosition);
    private final static double positionTolerance = 0.01;
    private final static double maxAcceleration = 150;//100
    private final static Boolean invertEncoder = false;
    private final static double maxVelocity = 600;//500
    private final static double minPosition = -20;
    private final static double maxPosition = 400;
    private final static String name = "IntakeFlop";

    public final Trigger deployedTrigger = new Trigger(() -> getPosition() >= 85);
    public final Trigger retractedTrigger = new Trigger(() -> getPosition() <= 5);
    public final Trigger coralTrigger = new Trigger(() -> coralDistance() <= 50);

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
        
    private final DigitalInput intakeBeam = new DigitalInput(9);
    private final DigitalSource sourcePWMX = new DigitalInput(8);
    private final DutyCycle dutyPWMX = new DutyCycle(sourcePWMX);
    private double lidarDistance = ((dutyPWMX.getHighTimeNanoseconds()/1000)-1000)/1.36;

    public IntakeFlop() {
        super(
        talonConfig,
        encoder, 
        invertEncoder,
        name,
        motorId,
        gearRatio,
        positionTolerance,
        debug);

        setPosition();
    }

    public double coralDistance() {
        boolean beam = !intakeBeam.get();
        boolean lidar =false;

        return ((dutyPWMX.getHighTimeNanoseconds()/1000)-1000)/1.36;
    }

    @Override 
    public void setPosition() {
        StatusCode stat = motor.setPosition(0);
        
        if (followerMotor != null) {
            stat = followerMotor.setPosition(0);
        }    
    }

    @Override
    public void displayStuff() {
        SmartDashboard.putBoolean("Coral trigger", coralTrigger.getAsBoolean());
        SmartDashboard.putBoolean("Retracted trigger", retractedTrigger.getAsBoolean());
        SmartDashboard.putNumber("Coral distance", coralDistance());
    }
    
}
