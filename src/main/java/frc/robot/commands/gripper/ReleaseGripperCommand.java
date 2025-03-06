package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class ReleaseGripperCommand extends Command {
    private final Gripper gripper;
    public ReleaseGripperCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute(){
        gripper.setSpeed(-1);
    }

    @Override
    public void end(boolean interrupted){
        gripper.stop();
    }
}
