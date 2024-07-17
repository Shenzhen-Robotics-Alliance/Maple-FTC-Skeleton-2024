package org.firstinspires.ftc.teamcode.Tests;

public interface SimpleUnitTest {
    default void beforeStartPeriodic() {}
    void testStart();
    void testPeriodic();
    default void testEnd() {}
}
