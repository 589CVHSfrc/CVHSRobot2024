package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandSupplier {
    Command m_blue;
    Command m_red;
    public CommandSupplier(Command blue, Command red){
        m_blue = blue;
        m_red = red;
    }
    public Command getAllianceCommand(){
        if (getAlliance()){
            return m_red;
        }
        return m_blue;
    }

    
    public Boolean getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
    
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }
}
