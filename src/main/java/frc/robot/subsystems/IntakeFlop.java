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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.PositionControlledMotor;
import frc.robot.utils.Constants;

public class IntakeFlop extends PositionControlledMotor {
    public final static double zeroPosition = 0.39;
    private final Double encoderConversion = 88.888;
    private final static int motorID = 9;
    private final int encoderID = 3;
        private DutyCycleEncoder encoder;
        public final static double positionTolerance = 0.01;
        private final static double maxAcceleration = 50;
        private final static Boolean invertEncoder = false;
        private final static double maxVelocity = 100;
        private final static double minPosition = -20;
        private final static double maxPosition = 400;
        private final static String name = "IntakeFlop";
    
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
                        .withSupplyCurrentLimit(30)
                );
        
        public IntakeFlop() {
            super(
                name, 
                new TalonFX(9, Constants.CANBUS), 
                true
            );
            
            encoder = new DutyCycleEncoder(encoderID, 1, zeroPosition);
            encoder.setInverted(invertEncoder);
    
            super.debug = true;
            resetPosition();
    
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
    }}
