package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class ReleaseGripperCommandL1 extends Command {
    private final Gripper gripper;
    public ReleaseGripperCommandL1(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute(){
        System.out.println("ungripping");
        SmartDashboard.putString("is Ungripping", "ungripping");
        SmartDashboard.putString("state", "ungripping");
        gripper.setSpeed(-1);//-0.85
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("is Ungripping", "ungripping");
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
