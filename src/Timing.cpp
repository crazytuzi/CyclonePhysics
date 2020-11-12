#include "Timing.h"

// Hold internal timing data for the performance counter.
static bool qpcFlag;

#if (__APPLE__ || __unix)
#define TIMING_UNIX	1

#include <stdlib.h>
#include <sys/time.h>

// assume unix based OS
typedef unsigned long long	LONGLONG;
#else
#define TIMING_WINDOWS	1
// assume windows

// Import the high performance timer (c. 4ms).
#include <windows.h>
#include <mmsystem.h>

static double qpcFrequency;
#endif

// Internal time and clock access functions
unsigned SystemTime()
{
#if TIMING_UNIX
    struct timeval tv;
    
    gettimeofday(&tv, 0);

    return tv.tv_sec * 1000 + tv.tv_usec/1000;
#else
    if (qpcFlag)
    {
        static LONGLONG qpcMillisPerTick;

        QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&qpcMillisPerTick));

        return static_cast<unsigned>(qpcMillisPerTick * qpcFrequency);
    }

    return static_cast<unsigned>(timeGetTime());
#endif
}

// Sets up the timing system and registers the performance timer.
void InitTime()
{
#if TIMING_UNIX
    qpcFlag = false;
#else
    LONGLONG time;

    qpcFlag = QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&time)) > 0;

    // Check if we have access to the performance counter at this
    // resolution.
    if (qpcFlag) qpcFrequency = 1000.0 / time;
#endif
}

// Holds the global frame time that is passed around
static Timing* timing = nullptr;

Timing& Timing::Get()
{
    return *timing;
}

void Timing::Update()
{
    if (timing == nullptr)
    {
        return;
    }

    // Advance the frame number.
    if (!timing->isPaused)
    {
        ++timing->frameCount;
    }

    // Update the timing information.
    const auto Time = SystemTime();

    timing->lastFrameDuration = Time - timing->lastFrameTimestamp;

    timing->lastFrameTimestamp = Time;

    // Update the tick information.
    const auto Clock = GetClock();

    timing->lastFrameClockTicks = Clock - timing->lastFrameClockstamp;

    timing->lastFrameClockstamp = Clock;

    // Update the RWA frame rate if we are able to.
    if (timing->frameCount > 1)
    {
        if (timing->averageFrameDuration <= 0)
        {
            timing->averageFrameDuration = static_cast<double>(timing->lastFrameDuration);
        }
        else
        {
            // RWA over 100 frames.
            timing->averageFrameDuration *= 0.99f;

            timing->averageFrameDuration += 0.01f * static_cast<double>(timing->lastFrameDuration);

            // Invert to get FPS
            timing->fps = static_cast<float>(1000.f / timing->averageFrameDuration);
        }
    }
}

void Timing::StartUp()
{
    // Set up the timing system.
    InitTime();

    // Create the frame info object
    if (timing == nullptr)
    {
        timing = new Timing();
    }

    // Set up the frame info structure.
    timing->frameCount = 0;

    timing->lastFrameTimestamp = SystemTime();

    timing->lastFrameDuration = 0;

    timing->lastFrameClockstamp = GetClock();

    timing->lastFrameClockTicks = 0;

    timing->isPaused = false;

    timing->averageFrameDuration = 0;

    timing->fps = 0;
}

void Timing::ShutDown()
{
    if (timing != nullptr)
    {
        delete timing;

        timing = nullptr;
    }
}

unsigned Timing::GetTime()
{
    return SystemTime();
}

#if TIMING_WINDOWS
unsigned long SystemClock()
{
    __asm{
        rdtsc;
        }
}
#endif

unsigned long Timing::GetClock()
{
#if TIMING_UNIX
    struct timeval tv;

    gettimeofday(&tv, 0);

    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
#else
    return SystemClock();
#endif
}

Timing::Timing(): frameCount(0), lastFrameTimestamp(0), lastFrameDuration(0), lastFrameClockstamp(0),
                  lastFrameClockTicks(0), isPaused(false), averageFrameDuration(0), fps(0)
{
}
