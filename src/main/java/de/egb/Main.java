package de.egb;

import de.egb.referee.RefereeReceiver;
import de.egb.vision.VisionReceiver;

public class Main {

    public static void main(String[] args) throws Exception{
        System.out.println("Starting SSL Zero AI");

        boolean bTestTraj = false;
        for(String s : args)
        {
            if("--traj-test".equals(s))
            {
                controlcore.tests.TrajectoryAndPControllerTest.run();
            }
        }

        VisionReceiver visionReceiver = new VisionReceiver();
        Thread visionThread = new Thread(visionReceiver);
        visionThread.start();

        /**RefereeReceiver refereeReceiver = new RefereeReceiver();
        Thread refereeThread = new Thread(refereeReceiver);
        refereeThread.start();**/

        System.out.println("SSL Zero AI started");

        while(true){
            Thread.sleep(100);
        }
    }

}