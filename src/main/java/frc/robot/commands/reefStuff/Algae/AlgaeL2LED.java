package frc.robot.commands.reefStuff.Algae;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeL2LED extends Command{
    private final Led led;

    /**
     * Creates a new L2AlgaeLed command.
     *
     * @param  The AlgaeL2LED LED subsystem used by this command.
     */
    public AlgaeL2LED(Led AlgaeL2LED) {
      this.led = AlgaeL2LED;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(AlgaeL2LED);
    }
  
    @Override
    public void execute() {
      led.l2algae();
      led.Coral();
    }
  
    public boolean isFinished() {
      return false;
    } 
}
