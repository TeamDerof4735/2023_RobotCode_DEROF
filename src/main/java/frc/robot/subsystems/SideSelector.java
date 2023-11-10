package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SideSelector extends SubsystemBase {
  Joystick joystick;
  
  int currentSideSelection;
  ArrayList<String> sides = new ArrayList<String>();

  String side;
  public static String sideToGo = new String();

  public SideSelector(Joystick joystick) {
    this.joystick = joystick;
    this.currentSideSelection = 0;

    sides.add("Front Side");
    sides.add("Back Side");
  }


  @Override
  public void periodic() {
    displaySelection();
  }

  public void selectionRight() {
    int pov = joystick.getPOV();

    if(pov == 90) {
      currentSideSelection++;
      
      if(currentSideSelection >= sides.size()) {
        currentSideSelection = 0;
      }
    }
  }

  public void selectionLeft() {
    int pov = joystick.getPOV();

    if(pov == 270) {
      currentSideSelection--;
      
      if(currentSideSelection < 0) {
        currentSideSelection = sides.size() - 1;
      }
    }
  }

  public void displaySelection() {
    String currentKeyLevel = sides.get(currentSideSelection);

    if(currentKeyLevel != null) {
      SmartDashboard.putString("Lado Actual", currentKeyLevel);

      sideToGo = side;
    } else{
      SmartDashboard.putString("Lado Actual", "No hay ningun nivel seleccionado");
    }
  }

  public String getLevelName() {
    String currentKey = sides.get(currentSideSelection);
    return currentKey;
  }

}
