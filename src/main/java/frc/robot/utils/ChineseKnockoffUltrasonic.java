package frc.robot.utils;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class ChineseKnockoffUltrasonic {
    private DigitalOutput trig;
    private DigitalInput echo;
    private Counter counter;

    public ChineseKnockoffUltrasonic(int trigPin, int echoPin){
        this.trig = new DigitalOutput(trigPin);
        this.echo = new DigitalInput(echoPin);
        this.counter = new Counter(echo);
        trig.set(false);
    }

    public double getDistanceCm(){
        counter.reset();
        trig.set(false);
        Timer.delay(0.000002); 
        trig.set(true);
        Timer.delay(0.00001);
        trig.set(false);
        Timer.delay(0.05);
        double pulseLength = counter.getPeriod(); 
        double distance = pulseLength * 34300 / 2;
        return distance;
    }
}
