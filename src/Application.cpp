#include "Application.h"
#include "gl/glut.h"
#include <cstring>

const char* Application::GetTitle()
{
    return "";
}

void Application::StartUp()
{
    glClearColor(0.9f, 0.95f, 1.f, 1.f);

    glEnable(GL_DEPTH_TEST);

    glShadeModel(GL_SMOOTH);

    SetView();
}

void Application::SetView()
{
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    gluPerspective(60.f, static_cast<double>(width) / height, 1.f, 500.f);

    glMatrixMode(GL_MODELVIEW);
}

void Application::ShutDown()
{
}

void Application::Display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_LINES);

    glVertex2i(1, 1);

    glVertex2i(639, 319);

    glEnd();
}

void Application::Update()
{
    glutPostRedisplay();
}

void Application::Key(unsigned char key)
{
}

void Application::Resize(const int width, int height)
{
    // Avoid the divide by zero.
    if (height <= 0)
    {
        height = 1;
    }

    // Set the internal variables and update the view
    Application::width = width;

    Application::height = height;

    glViewport(0, 0, width, height);

    SetView();
}

void Application::Mouse(int button, int state, int x, int y)
{
}

void Application::MouseDrag(int x, int y)
{
}

void Application::RenderText(const float x, float y, const char* text, void* font) const
{
    glDisable(GL_DEPTH_TEST);

    // Temporarily set up the view in orthographic projection.
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();

    glLoadIdentity();

    glOrtho(0.f, static_cast<double>(width), 0.f, static_cast<double>(height), -1.f, 1.f);

    // Move to ModelView mode.
    glMatrixMode(GL_MODELVIEW);

    glPushMatrix();

    glLoadIdentity();

    // Ensure we have a font
    if (font == nullptr)
    {
        font = GLUT_BITMAP_HELVETICA_10;
    }

    // Loop through characters displaying them.
    const auto length = strlen(text);

    glRasterPos2f(x, y);

    for (const auto* letter = text; letter < text + length; ++letter)
    {
        // If we meet a newline, then move down by the line-height
        if (*letter == '\n')
        {
            y -= 12.f;

            glRasterPos2f(x, y);
        }

        glutBitmapCharacter(font, *letter);
    }

    // Pop the matrices to return to how we were before.
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);

    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
}
