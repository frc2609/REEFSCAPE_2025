package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class GripCoralCommand extends Command {
    private final Gripper gripper;
    public GripCoralCommand(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute(){
        gripper.setCoralSpeed(-0.5);
    }

    @Override
    public void end(boolean interrupted){
        gripper.stopCoral();
    }
}
