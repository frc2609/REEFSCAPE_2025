package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkMax motor;
    public Intake() {
        motor = new SparkMax(9, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false);
motor.configure(motorConfig, null, null);
        motor.set(0);

     } 
public void setSpeed(double speed) {
    motor.set(speed);
    }
public void stop() {
    motor.set(0);
    }


    
}
