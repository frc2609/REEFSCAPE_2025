package frc.robot.commands.Intake.roll;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoll;

public class IntakeRollCommand extends Command {
    private final IntakeRoll roll;
    public IntakeRollCommand(IntakeRoll roll){
        this.roll = roll;
        addRequirements(roll);
    }

    @Override
    public void execute() {
        roll.setSpeed(0.5);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
