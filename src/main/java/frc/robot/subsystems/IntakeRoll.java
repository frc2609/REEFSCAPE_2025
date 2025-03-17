package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
public class IntakeRoll extends SubsystemBase {
    private final TalonFX rollMotor;
    private final TalonFXConfiguration motorConfig = 
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            // )
            // .withCurrentLimits(
            //     new CurrentLimitsConfigs()
            //         .withStatorCurrentLimit(15)
            //         .withSupplyCurrentLimit(15)
            );

    public IntakeRoll() {
        rollMotor = new TalonFX(6, Constants.CANBUS);
        rollMotor.getConfigurator().apply(motorConfig);

    } 
    public void setSpeed(double speed) {
        rollMotor.set(speed);
    }
    public void stop() {
        rollMotor.set(0);
    }    


    
}
