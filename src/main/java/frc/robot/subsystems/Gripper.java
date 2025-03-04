package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase  {
    private final SparkMax coralMotor;
    // private final SparkMax algaeMotor;
    public Gripper() {
        coralMotor = new SparkMax(8, MotorType.kBrushless);
        // SparkMaxConfig motorConfig = new SparkMaxConfig();
        // motorConfig.inverted(false);
        
        // coralMotor.configure(motorConfig, null, null);
        coralMotor.set(0);

        // algaeMotor = new SparkMax(18, MotorType.kBrushless);
        // SparkMaxConfig motorsConfig = new SparkMaxConfig();
        // motorsConfig.inverted(false);

        // algaeMotor.configure(motorConfig, null, null);
        // algaeMotor.set(0);

     } 
    public void setCoralSpeed(double speed) {
        coralMotor.set(speed);
    }
    public void stopCoral() {
        coralMotor.set(0);
    }
    // public void setAlgaeSpeed(double speed) {
    //     algaeMotor.set(speed);
    // }
    // public void stopAlgae() {
    //     algaeMotor.set(0);
    // }

    
}
