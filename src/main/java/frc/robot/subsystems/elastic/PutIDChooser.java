package frc.robot.subsystems.elastic;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class PutIDChooser {
    public SendableChooser<Integer> ID = new SendableChooser<>();

    public void inizialize(){
        ID.addOption("1", 1);
        ID.addOption("2", 2);
        ID.addOption("3", 3);
        ID.addOption("4", 4);
        ID.addOption("5", 5);
        ID.addOption("6", 6);
        ID.addOption("7", 7);
        ID.addOption("8", 8);
        ID.addOption("9", 9);
        ID.addOption("10", 10);
        ID.addOption("11", 11);
        ID.addOption("12", 12);
        ID.addOption("13", 13);
        ID.addOption("14", 14);
        ID.addOption("15", 15);
        ID.addOption("16", 16);
        ID.addOption("17", 17);
        ID.addOption("18", 18);
        ID.addOption("19", 19);
        ID.addOption("20", 20);
        ID.addOption("21", 21);
        ID.addOption("22", 22);
        SmartDashboard.putData("ID Chooser", ID);
    }
    public void periodic(){
        SmartDashboard.putData("ID Chooser", ID);
    }
}
