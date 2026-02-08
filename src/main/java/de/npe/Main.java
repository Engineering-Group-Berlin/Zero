package de.npe;

import de.npe.referee.RefereeReceiver;
import de.npe.vision.VisionReceiver;

public class Main {

    public static void main(String[] args) throws Exception{
        System.out.println("Starting SSL Zero AI");

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