// Include appropriate OpenGL headers.
#include <gl/glut.h>

// Include the general application structure.
#include "Application.h"

// Include the timing functions
#include "Timing.h"

// Forward declaration of the function that will return the
// application object for this particular demo. This should be
// implemented in the demo's .cpp file.
extern Application* GetApplication();

// Store the global application object.
Application* App;

/**
* Creates a window in which to display the scene.
*/
void CreateSceneWindow(const char* title)
{
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(640, 320);

    glutInitWindowPosition(0, 0);

    glutCreateWindow(title);
}

/**
* Called each frame to update the 3D scene. Delegates to
* the application.
*/
void Update()
{
    // Update the timing.
    Timing::Get().Update();

    // Delegate to the application.
    App->Update();
}

/**
* Called each frame to display the 3D scene. Delegates to
* the application.
*/
void Display()
{
    App->Display();

    // Update the displayed content.
    glFlush();

    glutSwapBuffers();
}

/**
* Called when a mouse button is pressed. Delegates to the
* application.
*/
void Mouse(const int button, const int state, const int x, const int y)
{
    App->Mouse(button, state, x, y);
}

/**
* Called when the display window changes size.
*/
void Reshape(const int width, const int height)
{
    App->Resize(width, height);
}

/**
* Called when a key is pressed.
*/
void Keyboard(const unsigned char key, int, int)
{
    // Note we omit passing on the x and y: they are rarely needed.
    App->Key(key);
}

/**
* Called when the mouse is dragged.
*/
void Motion(const int x, const int y)
{
    App->MouseDrag(x, y);
}

/**
* The main entry point. We pass arguments onto GLUT.
*/
int main(int argc, char** argv)
{
    // Set up GLUT and the timers
    glutInit(&argc, argv);

    Timing::StartUp();

    // Create the application and its window
    App = GetApplication();

    CreateSceneWindow(App->GetTitle());

    // Set up the appropriate handler functions
    glutReshapeFunc(Reshape);

    glutKeyboardFunc(Keyboard);

    glutDisplayFunc(Display);

    glutIdleFunc(Update);

    glutMouseFunc(Mouse);

    glutMotionFunc(Motion);

    // Run the application
    App->StartUp();

    glutMainLoop();

    // Clean up the application
    App->ShutDown();

    delete App;

    App = nullptr;

    Timing::ShutDown();
}
