package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends Command {
    private final Climber climber;

    public ClimbCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }


    @Override
    public void execute() {
        climber.goToPosition(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop climber when command ends
        climber.stop();
    }
}