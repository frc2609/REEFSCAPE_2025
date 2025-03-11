package frc.robot.subsystems.elastic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class AreWeSeeing extends SubsystemBase{
    @Override
    public void periodic(){
        if(LimelightHelpers.getFiducialID("limelight-april") == -1){
            SmartDashboard.putBoolean("We watching?", false);
        } else {
            SmartDashboard.putBoolean("We watching?", true);
        }
    }
}