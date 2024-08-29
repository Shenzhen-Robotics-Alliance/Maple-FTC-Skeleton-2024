package org.firstinspires.ftc.teamcode.tests;

public interface SimpleUnitTest {
    default void beforeStartPeriodic() {}
    void testStart();
    void testPeriodic();
    default void testEnd() {}
}
