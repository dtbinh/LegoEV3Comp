package org.lejos.ev3.sample.bumpercar;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;


public class EV3BumperCar
{
    //static RegulatedMotor leftMotor = MirrorMotor.invertMotor(Motor.A);
    //static RegulatedMotor rightMotor = MirrorMotor.invertMotor(Motor.B);
    static RegulatedMotor leftMotor = Motor.A;
    static RegulatedMotor rightMotor = Motor.D;
    
    static RegulatedMotor sonarMotor = Motor.B;
    
//    static UltrasonicSensor sonar;
//    static GryroSensor gyro;
//    static boolean running = true;
    
    static int DISTANCE = 7;
    static int FAR_DISTANCE = 15;
    
    static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S1);
 
  
  public static void main(String[] args)
  {
	  MapStageEngine engine = new MapStageEngine();
	  engine.makeProgress(gyroSensor);
	
  }
  
  
  static class MapStageEngine {
	  int currentStage = 0;
	  public MapStageEngine() {
	  }
	  
	  public void completeStage() {
		  currentStage++;
	  }
	  
	  public void makeProgress(EV3GyroSensor gyroSensor) {
		  boolean allowYeilds = false;
		  switch(currentStage) {
		  	case 0:
		  		leftMotor.resetTachoCount();
			    rightMotor.resetTachoCount();
			    leftMotor.rotateTo(0);
				rightMotor.rotateTo(0);	
				
				LCD.drawString("Booom! Lets rock this!",0,1);
			    Button.waitForAnyPress();
			    gyroSensor.reset();
			    
			    forwardUntil6InchesFromWall(allowYeilds);
			    
			    
		  	case 1:
		  		// entering our first turn... woot woot.
		  		turnRight90(allowYeilds, -90f);
				forwardUntil6InchesFromWall(allowYeilds);
				turnRight90(allowYeilds, -180f);
				
		  	case 2:
		  		// second straight away
		  		forwardUntilBlackTape(allowYeilds);
		  		turnLeft90(allowYeilds, -90f);
		  		
		  		// go threw the gap
		  		forwardUntil6InchesFromWall(allowYeilds);
		  		turnRight90(allowYeilds, -180f);
		  		
		  		// 2nd to last straight away
		  		forwardUntil6InchesFromWall(allowYeilds);
		  		turnLeft90(allowYeilds, -90f);
		  		
		  		forwardUntil6InchesFromWall(allowYeilds);
		  		turnLeft90(allowYeilds, 0f);
		  		forwardUntil6InchesFromWall(allowYeilds);
		  }
	  }
	  
	private void endApp() {
		// TODO Auto-generated method stub
		Button.waitForAnyPress();
	}

	private void forwardUntilBlackTape(boolean allowYields) {
		
		ColorSensor c = new ColorSensor();
		c.setDaemon(true);
	    c.start();
		
		setForwardSlow();
		leftMotor.forward();
		rightMotor.forward();
		
		while(!c.hasSeenBlackColor){
			if (allowYields) Thread.yield();
			continue;
		}
		
		rightMotor.stop(true);
		leftMotor.stop(true);
	}

	private void forwardUntil6InchesFromWall(boolean allowYeilds) {
		UltrasonicSensor sonar = new UltrasonicSensor();
	    sonar.setDaemon(true);
	    sonar.start();
		
		setForwardSpeed();
		leftMotor.forward();
		rightMotor.forward();
		
		boolean hasSlowedDown = false;
		while ( sonar.distance > DISTANCE ) {
			if (sonar.distance < FAR_DISTANCE && !hasSlowedDown) {
				hasSlowedDown = true;
				setForwardSlow();
				leftMotor.forward();
			    rightMotor.forward();
			}
			
			if (allowYeilds) Thread.yield();
			continue;
		}
		
		sonar.end();
		rightMotor.stop(true);
		leftMotor.stop(true);
	}

	private void turnRight90(boolean allowYeilds, float targetAngle) {
		
		GryroSensor gyro = new GryroSensor(gyroSensor);
	    gyro.setDaemon(true);
	    gyro.start();
		
		setTurnSpeedFast();
  		leftMotor.forward();
		rightMotor.backward();
		
		boolean hasChangedSpeeds = false;
		while (gyro.angle > targetAngle) {
//			if (gyro.angle < targetAngle / 2  && !hasChangedSpeeds) {
//				hasChangedSpeeds = true;
//				setTurnSpeedSlow();
//				leftMotor.forward();
//				rightMotor.backward();
//			}
			if (allowYeilds) Thread.yield();
			continue;
		}
		
		rightMotor.stop(true);
		leftMotor.stop(true);
		
//		if (gyro.angle <= (-90.1 + startAngle)) {
//			setTurnSpeedSuperSlow();
//			leftMotor.backward();
//			rightMotor.forward();
//			while (gyro.angle < (-90f + startAngle)) {
//				if (allowYeilds) Thread.yield();
//				continue;
//			}
//			
//			rightMotor.stop(true);
//			leftMotor.stop(true);
//		}
		
//		gyro.resetGyro();
		gyro.end();
	}
	
	private void turnLeft90(boolean allowYeilds, float targetAngle) {
		
		GryroSensor gyro = new GryroSensor(gyroSensor);
	    gyro.setDaemon(true);
	    gyro.start();
		
		setTurnSpeedFast();
  		rightMotor.forward();
		leftMotor.backward();
		
		float startAngle = gyro.angle;
		
		boolean hasChangedSpeeds = false;
		while (gyro.angle < targetAngle) {
//			if (gyro.angle > targetAngle / 2 && !hasChangedSpeeds) {
//				hasChangedSpeeds = true;
//				setTurnSpeedSlow();
//				rightMotor.forward();
//				leftMotor.backward();
//			}
			if (allowYeilds) Thread.yield();
			continue;
		}
		
		rightMotor.stop(true);
		leftMotor.stop(true);
		
//		if (gyro.angle >= (90.1+ startAngle)) {
//			setTurnSpeedSuperSlow();
//			rightMotor.backward();
//			leftMotor.forward();
//			while (gyro.angle > (90f+ startAngle)) {
//				if (allowYeilds) Thread.yield();
//				continue;
//			}
//			
//			rightMotor.stop(true);
//			leftMotor.stop(true);
//		}
		
		gyro.end();
	}

	private void setForwardSpeed() {
		leftMotor.setSpeed(800);
		rightMotor.setSpeed(800);
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
	}
	private void setForwardSlow() {
		leftMotor.setSpeed(400);
		rightMotor.setSpeed(400);
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
	}
	
	private void setTurnSpeedFast() {
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
	}
	
	private void setTurnSpeedSlow() {
		leftMotor.setSpeed(30);
		rightMotor.setSpeed(30);
	}
	
	private void setTurnSpeedSuperSlow() {
		leftMotor.setSpeed(40);
		rightMotor.setSpeed(40);
	}
	 
  }
}

class UltrasonicSensor extends Thread
{
	EV3UltrasonicSensor uSensor = new EV3UltrasonicSensor(SensorPort.S4);
    SampleProvider sampleProvider = uSensor.getDistanceMode();
    SampleProvider average = new MeanFilter(sampleProvider, 5);
    public int control = 1;
    public float distance = 20;
    public boolean enableLogging = false;
    public boolean shouldRun = true;
    UltrasonicSensor() { }
    
    public void run()
    {
    	float [] sample = new float[average.sampleSize()];
        while (shouldRun)
        {
        	average.fetchSample(sample, 0);
            distance = convertMetersToInches((float)sample[0]);
            if (enableLogging) {
            	System.out.println(" Distance: " + distance + "in" );
            }
            Thread.yield();
        }  
        
        uSensor.close();
    }
    
    public void end() { shouldRun = false; }
    
    public float convertMetersToInches(float meters) {
    	return meters * 39.370f;
    }
}

class GryroSensor extends Thread
{
	EV3GyroSensor gyroSensor;
	SampleProvider sampleProvider; 
    public float angle = 0;
    public boolean enableLogging = false;
    boolean shouldRun = true;

    GryroSensor(EV3GyroSensor gyroSensor) {
    	this.gyroSensor = gyroSensor;
    	sampleProvider = gyroSensor.getAngleMode();
    }
    
    public void end() {
    	shouldRun = false;
    }
    
    public void run()
    {
    	float [] sample = new float[sampleProvider.sampleSize()];
        while (shouldRun)
        {
        	sampleProvider.fetchSample(sample, 0);
        	angle = (float)sample[0];
        	if (enableLogging) {
        		System.out.println(" GyroAngle: " + angle);
        	}
        }
    }
}

class ColorSensor extends Thread
{
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
	SampleProvider sampleProvider = colorSensor.getMode("RGB");
   
	public boolean hasSeenBlackColor = false;
    public boolean enableLogging = false;

    ColorSensor() { }
      
    public void run()
    {
    	float [] sample = new float[sampleProvider.sampleSize()];
        while (true)
        {
        	sampleProvider.fetchSample(sample, 0);
        	boolean allBelow = true;
        	for(int i = 0; i < sample.length; i++) {
        		float color = sample[i];
        		if (color >= .1f) {
        			allBelow = false;
        			break;
        		}
        	}
        	
        	if (allBelow) {
        		hasSeenBlackColor = true;
        		break;
        	}
        	
        	Thread.yield();
        }
        colorSensor.close();
    }
}

//class IRSensor extends Thread
//{
//    EV3IRSensor ir = new EV3IRSensor(SensorPort.S4);
//    SampleProvider sp = ir.getDistanceMode();
//    public int control = 0;
//    public int distance = 255;
//
//    IRSensor()
//    {
//
//    }
//    
//    public void run()
//    {
//        while (true)
//        {
//            float [] sample = new float[sp.sampleSize()];
//            control = ir.getRemoteCommand(0);
//            sp.fetchSample(sample, 0);
//            distance = (int)sample[0];
//            System.out.println("Control: " + control + " Distance: " + distance);
//            
//        }
//        
//    }
//}



