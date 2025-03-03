package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimberToPositionSDB extends Command{
    private final Climber climber;
    private double targetPosition;

    public MoveClimberToPositionSDB(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        targetPosition = SmartDashboard.getNumber("Target", 0);
        climber.goToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
      //  return climber.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
