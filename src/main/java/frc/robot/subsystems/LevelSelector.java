package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LevelSelector extends SubsystemBase {
  Joystick joystick;
  
  int currentLevelSelection;
  ArrayList<String> levels = new ArrayList<String>();

  String level;
  public static String levelToShoot = new String();

  public LevelSelector(Joystick joystick) {
    this.joystick = joystick;
    this.level = levelToShoot;
    this.currentLevelSelection = 1;

    levels.add("Low Level");
    levels.add("Mid Level");
    levels.add("High Level");
  }

  @Override
  public void periodic() {
    displaySelection();

    
  }

  public void selectionUp() {
    int pov = joystick.getPOV();

    if(pov == 0) {
      currentLevelSelection++;
      
      if(currentLevelSelection >= levels.size()) {
        currentLevelSelection = 0;
      }
    }
  }

  public void selectionDown() {
    int pov = joystick.getPOV();

    if(pov == 180) {
      currentLevelSelection--;
      
      if(currentLevelSelection < 0) {
        currentLevelSelection = levels.size() - 1;
      }
    }
  }

  public void displaySelection() {
    String currentKeyLevel = levels.get(currentLevelSelection);

    if(currentKeyLevel != null) {
      SmartDashboard.putString("Nivel Actual", currentKeyLevel);

      levelToShoot = level;
    } else{
      SmartDashboard.putString("Nivel Actual", "No hay ningun nivel seleccionado");
    }
  }

  public String getLevelName() {
    String currentKey = levels.get(currentLevelSelection);
    return currentKey;
  }
}
