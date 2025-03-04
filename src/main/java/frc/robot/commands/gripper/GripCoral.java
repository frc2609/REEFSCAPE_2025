package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

public class GripCoral extends Command {
    private final Gripper gripper;
    public GripCoral(Gripper gripper) {
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
