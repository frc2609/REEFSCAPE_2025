package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase  {
    private final SparkMax motor;
    private final SparkMax motorB;
    public Gripper() {
        motor = new SparkMax(8, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false);

        motorB = new SparkMax(18, MotorType.kBrushless);
        SparkMaxConfig motorsConfig = new SparkMaxConfig();
        motorsConfig.inverted(false);

motor.configure(motorConfig, null, null);
        motor.set(0);

        motorB.configure(motorConfig, null, null);
        motorB.set(0);

     } 
public void setSpeed(double speed) {
    motor.set(speed);
    }
public void stop() {
    motor.set(0);
    }


    
}
