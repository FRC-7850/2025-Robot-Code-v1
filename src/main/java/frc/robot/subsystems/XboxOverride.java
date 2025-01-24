package frc.robot.subsystems;
import edu.wpi.first.wpilibj.XboxController;

public class XboxOverride extends XboxController{
    public XboxOverride(int Port){
        super(Port);
    }
    @Override 
    public double getLeftX() {
        var output = (super.getLeftX() > .8) ? 1 : 0;
            output = (super.getLeftX() < -.8) ? -1 : 0;
        return output;
    }
    @Override
    public double getLeftY() {
        var output = (super.getLeftY() > .8) ? 1 : 0;
            output = (super.getLeftY() < -.8) ? -1 : 0;       
     return output;
    }
    
}
