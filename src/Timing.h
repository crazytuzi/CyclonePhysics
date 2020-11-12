#pragma once

/**
 * @file
 *
 * Holds the timing system for the physics demos.
 */

/**
 * Represents all the information that the demo might need about the
 * timing of the game: current time, fps, frame number, and so on.
 */
class Timing
{
public:
    /**
    * Gets the global timing object.
    */
    static Timing& Get();

    /**
    * Updates the timing system, should be called once per frame.
    */
    static void Update();

    /**
    * Startup the frame information system. Use the overall
    * init function to set up all modules.
    */
    static void StartUp();

    /**
    * Shutdown the frame information system.
    */
    static void ShutDown();

    /**
    * Gets the global system time, in the best resolution possible.
    * Timing is in milliseconds.
    */
    static unsigned GetTime();

    /**
    * Gets the clock ticks since process start.
    */
    static unsigned long GetClock();

private:
    // These are private to stop instances being created: use Get().
    Timing();

private:
    /** The current render frame. This simply increments. */
    unsigned frameCount;

    /**
    * The timestamp when the last frame ended. Times are
    * given in milliseconds since some undefined time.
    */
    unsigned lastFrameTimestamp;

    /**
    * The duration of the last frame in milliseconds.
    */
    unsigned lastFrameDuration;

    /**
    * The clockstamp of the end of the last frame.
    */
    unsigned long lastFrameClockstamp;

    /**
    * The duration of the last frame in clock ticks.
    */
    unsigned long lastFrameClockTicks;

    /**
    * Keeps track of whether the rendering is paused.
    */
    bool isPaused;

    // Calculated data

    /**
    * This is a recency weighted average of the frame time, calculated
    * from frame durations.
    */
    double averageFrameDuration;

    /**
    * The reciprocal of the average frame duration giving the mean
    * fps over a recency weighted average.
    */
    float fps;
};
