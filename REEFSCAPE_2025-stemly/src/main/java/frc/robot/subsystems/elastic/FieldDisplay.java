package frc.robot.subsystems.elastic;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class FieldDisplay extends SubsystemBase {

    public Field2d field = new Field2d();
    public FieldDisplay() {
        // Constructor code, if necessary
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = AutoBuilder.getCurrentPose();
        field.setRobotPose(currentPose);
    }
}