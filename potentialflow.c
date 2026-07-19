/* Potential flow. Took around 6 hours */

#include <stdio.h>
#include <raylib.h>
#include <math.h>
#define RADIUS 60
#define POINT_COUNT 250
#define STREAMLINE_COUNT 40
#define STEP_SIZE 4.0f

bool IsMouseOverCircle(Vector2 mousePosition, Vector2 circlePosition);

int main() {
    InitWindow(600, 600, "Black Screen");
    bool dragging = false;
    Vector2 initialPosition = {300, 300}; // Vector 2 defines how many dimensions a vector will have (x and y)
    Vector2 points[STREAMLINE_COUNT][POINT_COUNT]; // defining amount of beads a polyline will have

    while (!WindowShouldClose()) { // while the window is not closed
        BeginDrawing(); // draw to the window
        ClearBackground(BLACK);
        Vector2 mousePosition = GetMousePosition(); // get the mouse position

        if (IsMouseOverCircle(mousePosition, initialPosition) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) { // if mouse is over circle & is mouse = yes = drag
            dragging = true;
        }
        if (dragging) {
            initialPosition = mousePosition;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            dragging = false;
        }

        float R2 = (float)(RADIUS * RADIUS);

        // trace streamlines by integrating the velocity field with RK4
        // this naturally follows the flow around the circle instead of solving column by column
        for (int s = 0; s < STREAMLINE_COUNT; s++) {
            float baseY = s * (600.0f / STREAMLINE_COUNT);

            // start each streamline at the left edge of the screen
            float x = -initialPosition.x;
            float y = baseY - initialPosition.y;

            for (int i = 0; i < POINT_COUNT; i++) {
                float absX = x + initialPosition.x;
                float absY = y + initialPosition.y;

                // mark points inside the circle or off screen as bad
                if (x*x + y*y < R2 || absX > 620.0f || absY < -20.0f || absY > 620.0f) {
                    points[s][i].x = -9999;
                    points[s][i].y = -9999;
                } else {
                    points[s][i].x = absX;
                    points[s][i].y = absY;
                }
                float r2 = fmaxf(x*x + y*y, 1.0f);
                float u1 = 1.0f - R2*(x*x - y*y)/(r2*r2);
                float v1 = -2.0f*R2*x*y/(r2*r2);
                float spd = sqrtf(u1*u1 + v1*v1);
                if (spd < 0.001f) spd = 0.001f;
                float k1x = u1/spd, k1y = v1/spd;

                float x2 = x + 0.5f*STEP_SIZE*k1x, y2 = y + 0.5f*STEP_SIZE*k1y;
                r2 = fmaxf(x2*x2 + y2*y2, 1.0f);
                float u2 = 1.0f - R2*(x2*x2 - y2*y2)/(r2*r2);
                float v2 = -2.0f*R2*x2*y2/(r2*r2);
                spd = sqrtf(u2*u2 + v2*v2); if (spd < 0.001f) spd = 0.001f;
                float k2x = u2/spd, k2y = v2/spd;

                float x3 = x + 0.5f*STEP_SIZE*k2x, y3 = y + 0.5f*STEP_SIZE*k2y;
                r2 = fmaxf(x3*x3 + y3*y3, 1.0f);
                float u3 = 1.0f - R2*(x3*x3 - y3*y3)/(r2*r2);
                float v3 = -2.0f*R2*x3*y3/(r2*r2);
                spd = sqrtf(u3*u3 + v3*v3); if (spd < 0.001f) spd = 0.001f;
                float k3x = u3/spd, k3y = v3/spd;

                float x4 = x + STEP_SIZE*k3x, y4 = y + STEP_SIZE*k3y;
                r2 = fmaxf(x4*x4 + y4*y4, 1.0f);
                float u4 = 1.0f - R2*(x4*x4 - y4*y4)/(r2*r2);
                float v4 = -2.0f*R2*x4*y4/(r2*r2);
                spd = sqrtf(u4*u4 + v4*v4); if (spd < 0.001f) spd = 0.001f;
                float k4x = u4/spd, k4y = v4/spd;

                // advance position along the streamline
                x += STEP_SIZE * (k1x + 2.0f*k2x + 2.0f*k3x + k4x) / 6.0f;
                y += STEP_SIZE * (k1y + 2.0f*k2y + 2.0f*k3y + k4y) / 6.0f;
            }
        }

        DrawCircleV(initialPosition, RADIUS, WHITE); // draw the circle

        // draw all streamlines
        for (int s = 0; s < STREAMLINE_COUNT; s++) {
            for (int i = 0; i < POINT_COUNT - 1; i++) {
                Vector2 p0 = points[s][i];
                Vector2 p1 = points[s][i+1];

                // skip bad/sentinel points where we were inside the circle
                if (p0.x < -999 || p1.x < -999) continue;

                float dx  = p0.x - initialPosition.x;
                float dy  = p0.y - initialPosition.y;
                float dist  = sqrtf(dx*dx + dy*dy);

                float dx2 = p1.x - initialPosition.x;
                float dy2 = p1.y - initialPosition.y;
                float dist2 = sqrtf(dx2*dx2 + dy2*dy2);

                if (dist < RADIUS + 2 || dist2 < RADIUS + 2) continue; // only draw outside the circle

                // color by local flow speed
                float cx = p0.x - initialPosition.x;
                float cy = p0.y - initialPosition.y;
                float r2 = cx*cx + cy*cy;
                if (r2 < 1.0f) r2 = 1.0f;
                float u = 1.0f - R2*(cx*cx - cy*cy)/(r2*r2);
                float v = -2.0f*R2*cx*cy/(r2*r2);
                float speed = sqrtf(u*u + v*v);
                float t = (speed - 0.5f) / 1.5f;
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
                Color lineColor;
                if (t < 0.5f) {
                    float tt = t * 2.0f;
                    lineColor = (Color){0, (unsigned char)(255 * tt), (unsigned char)(255 * (1.0f - tt)), 255};
                } else {
                    float tt = (t - 0.5f) * 2.0f;
                    lineColor = (Color){(unsigned char)(255 * tt), (unsigned char)(255 * (1.0f - tt)), 0, 255};
                }
                DrawLineV(p0, p1, lineColor);
            }
        }

        EndDrawing(); // finish drawing so the window is done
    }
    CloseWindow();
    return 0;
}

bool IsMouseOverCircle(Vector2 mousePosition, Vector2 circlePosition)
{
    return CheckCollisionPointCircle(mousePosition, circlePosition, RADIUS);
}
