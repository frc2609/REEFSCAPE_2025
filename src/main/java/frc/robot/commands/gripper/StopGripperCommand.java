package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class StopGripperCommand extends Command {
    private final Gripper gripper;
    public StopGripperCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute(){
        gripper.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
