package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Gripper extends SubsystemBase  {
    private final TalonFX gripMotor;
    private final TalonFXConfiguration motorConfig = 
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimit(3)
            );
    // private final SparkMax algaeMotor;
    public Gripper() {
        gripMotor = new TalonFX(8, Constants.CANBUS);
        gripMotor.getConfigurator().apply(motorConfig);

     } 
    public void setSpeed(double speed) {
        gripMotor.set(speed);
    }
    public void stop() {
        gripMotor.set(0);
    }    
}
