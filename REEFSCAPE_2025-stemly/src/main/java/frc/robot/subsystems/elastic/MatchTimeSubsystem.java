package frc.robot.subsystems.elastic;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MatchTimeSubsystem extends SubsystemBase {

    public MatchTimeSubsystem() {
        // Constructor code, if necessary
    }
    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putBoolean("intake sensor", intakeBeam.get());

        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Match Time", matchTime);

        double batteryVoltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Battery Voltage", batteryVoltage);
        
    }
}

