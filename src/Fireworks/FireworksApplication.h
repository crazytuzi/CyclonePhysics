#pragma once

#include "Application.h"
#include "FireworkRule.h"
#include "Fireworks.h"

class FireworksApplication : public Application
{
public:
    FireworksApplication();

    ~FireworksApplication();

    /** Sets up the graphic rendering. */
    virtual void StartUp() override;

    /** Returns the window title for the demo. */
    virtual const char* GetTitle() override;

    /** Update the particle positions. */
    virtual void Update() override;

    /** Update the particle positions. */
    virtual void Display() override;

    /** Handle a keypress. */
    virtual void Key(unsigned char key) override;
private:
    /** Dispatches a firework from the origin. */
    void Create(unsigned type, const Firework* parent = nullptr);

    /** Dispatches the given number of fireworks from the given parent. */
    void Create(unsigned type, unsigned number, const Firework* parent = nullptr);

    /** Creates the rules. */
    void InitFireworkRules();
private:
    /**
    * Holds the maximum number of fireworks that can be in use.
    */
    const static unsigned maxFireworks = 1024;

    /** Holds the firework data. */
    Firework fireworks[maxFireworks];

    /** Holds the index of the next firework slot to use. */
    unsigned nextFirework;

    /** Holds the number of rules. */
    const static unsigned ruleCount = 9;

    /** Holds the set of rules. */
    FireworkRule rules[ruleCount];
};
