package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import frc.robot.utils.PositionControlledMotor;
import frc.robot.utils.Constants;
import frc.robot.utils.EncoderAdapter;

public class Arm extends PositionControlledMotor{
    public final static double zeroPosition = 0.63;

    public final static double positionTolerance = 0.01;
    private final static double maxAcceleration = 160;
    private final static Boolean invertEncoder = false;
    private final static double maxVelocity = 80;
    private final static double minPosition = -33;
    private final static double maxPosition = 3.85;
    private final static String name = "Arm";
    
    public Arm() {
        super(
            name, 
            new TalonFX(7, Constants.CANBUS), 
            new DutyCycleEncoder(1), 
            new ProfiledPIDController(
                20, 1, .5,
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
            .withMotionMagicAcceleration(maxAcceleration)
            .withMotionMagicJerk(10*maxAcceleration);

        talonConfig.Slot0
            .withKP(.5);

        talonConfig.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxPosition)
            .withReverseSoftLimitThreshold(minPosition);
        
        motor.getConfigurator().apply(talonConfig);
    }
}
