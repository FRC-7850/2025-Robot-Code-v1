package frc.robot.subsystems;
import edu.wpi.first.wpilibj.XboxController;

public class XboxOverride extends XboxController{
    public XboxOverride(int Port){
        super(Port);
    }
    @Override 
    public double getRightX() {
        var output = (super.getRightX() > .8) ? 1 : 0;
            output = (super.getRightX() < -.8) ? -1 : 0;
        return output;
    }
    @Override
    public double getRightY() {
        var output = (super.getRightY() > .8) ? 1 : 0;
            output = (super.getRightY() < -.8) ? -1 : 0;       
     return output;
    }
}
