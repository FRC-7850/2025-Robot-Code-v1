package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.time.Year;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase{

    AddressableLED m_led = new AddressableLED(Constants.LEDConstants.LEDPWM);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.NUMLED);
    AddressableLEDBufferView m_underglow = m_ledBuffer.createView(Constants.LEDConstants.UNDERGLOWSTART, Constants.LEDConstants.UNDERGLOWEND);
    AddressableLEDBufferView m_elevator = m_ledBuffer.createView(Constants.LEDConstants.ELEVATORSTART, Constants.LEDConstants.ELEVATOREND);
    LEDPattern allianceColor = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));
    LEDPattern elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,Color.kRed, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(25));

    public LEDs(){
        
        m_led.setLength(m_ledBuffer.getLength());
        
        m_led.setData(m_ledBuffer);
        m_led.start();
        // since freshman Year
        // is junior. Robotics Bball and band, big contributer
        // willing to jump in whenever, driver first year, great, calm etc...
        // PLTW student honors

    }

    public void setUnderGlow(boolean isBlue){
        

        if(isBlue){
            allianceColor = LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));
        }

        allianceColor.applyTo(m_underglow); 
        m_led.setData(m_ledBuffer);
    }

    public void setElevatorGradiant(){
        

        elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,Color.kRed, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(25));
        //elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,Color.kRed, Color.kWhite).synchronizedBlink(RobotController::getRSLState);
        elevatorPattern.applyTo(m_elevator); 
       // m_led.setData(m_ledBuffer);
    }

    public void setElevatorRSL(){

        elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,Color.kRed, Color.kWhite).synchronizedBlink(RobotController::getRSLState);
        //elevatorPattern.applyTo(m_elevator); 
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      setUnderGlow(DriverStation.getAlliance().get()==Alliance.Blue);
    }

    
}
