package test;

import trajectory.TrajectoryGenerator;
import trajectory.Segment;
import java.util.Timer;
import controller.DriveStraightController;

public class TestTrajectory{

    public static void main(String[] args){
        TrajectoryGenerator testGenerator = new TrajectoryGenerator(10.0, 15.0, 200.0);
        Segment[] leftTrajectory = testGenerator.calcTrajectory(0.0, 0.0, 25.0);
        Segment[] rightTrajectory = testGenerator.calcTrajectory(0.0, 0.0, 25.0);
        double leftMaxV = testGenerator.calcAdjustedMaxV(0.0, 0.0, 25.0);
        double rightMaxV = testGenerator.calcAdjustedMaxV(0.0, 0.0, 25.0);

        /*for(int i=0; i<trajectoryInfo.length; i++){
            System.out.println(trajectoryInfo[i]+"\n");
        }*/
        /*for(int i =0; i<trajectory.length; i++){
            double velocity = trajectory[i].getVelocity();
            double acceleration = trajectory[i].getAcceleration();
            double position = trajectory[i].getPosition();
            System.out.println("Velocity: "+velocity+" --- Acceleration: "+acceleration+" --- Position: "+position+" \n");
        }*/
        Timer timer = new Timer();
        timer.schedule(new DriveStraightController(leftTrajectory, rightTrajectory, leftMaxV, rightMaxV, 200.0), 0, 5);
        System.out.println("Done");
    }
}
