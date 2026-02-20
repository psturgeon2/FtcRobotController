package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class SharedStorage {
    private static Pose sharedPose;

    private static int testX;

    private static void SetSharedPose(Pose sourcePose)
    {
        sharedPose = sourcePose;
    }

    private static Pose GetSharedPose()
    {
        return sharedPose;
    }

    private static void SetTestX(int x)
    {
        testX = x;
    }
    private static int GetTestX()
    {
        return testX;
    }
}
