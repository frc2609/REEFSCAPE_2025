package frc.robot.commands.reefStuff.Algae;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeL3LED extends Command {
    private final Led led;

    /**
     * Creates a new L3AlgaeLed command.
     *
     * @param AlgaeL3LED The LED subsystem used by this command.
     */
    public AlgaeL3LED(Led AlgaeL3LED) {
      this.led = AlgaeL3LED;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(AlgaeL3LED);
    }
  
    @Override
    public void execute() {
      led.l3algae();
      led.Coral();
    }
  
    public boolean isFinished() {
      return false;
    }
}
