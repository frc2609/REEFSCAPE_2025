package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class SendableChooserUtil {
    public static final int NUM_TAGS = 22;

    public static SendableChooser<Integer> createSequentialChooser() {
        SendableChooser<Integer> chooser = new SendableChooser<>();

        for (int i = 2; i <= NUM_TAGS; i++) {
            chooser.addOption(String.valueOf(i), i);
        }

        chooser.setDefaultOption("1", 1);

        return chooser;
    }
}
