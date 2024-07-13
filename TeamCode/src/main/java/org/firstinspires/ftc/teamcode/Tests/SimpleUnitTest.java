package org.firstinspires.ftc.teamcode.Tests;

public interface SimpleUnitTest {
    void testStart();
    void testPeriodic();
    default void testEnd() {}
}
