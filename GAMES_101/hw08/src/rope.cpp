#include "rope.h"

#include <iostream>
#include <vector>

#include "CGL/vector2D.h"
#include "CGL/vector4D.h"
#include "mass.h"
#include "spring.h"

namespace CGL
{

Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass,
           float k, vector<int> pinned_nodes)

{
    // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and
    // containing `num_nodes` nodes.

    double delta_x = (end.x - start.x) / (num_nodes - 1);
    double delta_y = (end.y - start.y) / (num_nodes - 1);
    double x = start.x;
    double y = start.y;

    for (int i = 0; i < num_nodes; ++i) {
        Mass *node = new Mass({x, y}, node_mass, false);
        if (i != 0) {
            springs.push_back(new Spring(masses.back(), node, k));
        }
        masses.push_back(node);
        x += delta_x;
        y += delta_y;
    }

    // Comment-in this part when you implement the constructor
    for (int i : pinned_nodes) {
        masses[i]->pinned = true;
    }
}
Rope::~Rope()
{
    for (Mass *m : masses) {
        if (m != nullptr) {
            delete m;
        }
    }
    for (Spring *s : springs) {
        if (s != nullptr) {
            delete s;
        }
    }
}

void Rope::simulateEuler(float delta_t, Vector2D gravity)
{
    for (auto *s : springs) {
        // TODO (Part 2): Use Hooke's law to calculate the force on a node
        double length = (s->m2->position - s->m1->position).norm();
        double delta_length = length - s->rest_length;
        Vector2D force_direction = (s->m2->position - s->m1->position).unit();
        float ks = s->k;
        Vector2D f = ks * force_direction * delta_length;
        s->m1->forces += f;
        s->m2->forces += -f;

        // damping force
        // slow down spring oscillation
        // float kd = 0.1f;
        // Vector2D relative_v = s->m2->velocity - s->m1->velocity;
        // Vector2D damping_f =
        //     -kd * dot(force_direction, relative_v) * force_direction;
        // s->m2->forces += damping_f;
    }

    for (auto *m : masses) {
        if (!m->pinned) {
            // TODO (Part 2): Add the force due to gravity, then compute the new
            // velocity and position
            // TODO (Part 2): Add global damping

            float kd = 0.01f;
            // slow down all motion
            Vector2D damping_force = -kd * m->velocity;
            Vector2D acceleration =
                (m->forces + damping_force) / m->mass + gravity;
            // explicit method
            // m->position += m->velocity * delta_t;
            m->velocity += acceleration * delta_t;
            // semi-implicit method
            m->position += m->velocity * delta_t;
        }

        // Reset all forces on each mass
        m->forces = Vector2D(0, 0);
    }
}

void Rope::simulateVerlet(float delta_t, Vector2D gravity)
{
    for (Spring *s : springs) {
        // TODO (Part 3): Simulate one timestep of the rope using explicit
        // Verlet （solving constraints)
        double length = (s->m2->position - s->m1->position).norm();
        double delta_length = length - s->rest_length;
        Vector2D force_direction = (s->m2->position - s->m1->position) / length;

        Vector2D f = s->k * force_direction * delta_length;
        s->m1->forces += f;
        s->m2->forces -= f;
    }

    for (auto *m : masses) {
        if (!m->pinned) {
            // TODO (Part 3.1): Set the new position of the rope mass
            // TODO (Part 4): Add global Verlet damping

            Vector2D current_pos = m->position;
            double damping_factor = 0.00005f;

            Vector2D acceleration = m->forces / m->mass + gravity;
            m->position +=
                (1.0f - damping_factor) * (current_pos - m->last_position) +
                acceleration * delta_t * delta_t;
            m->last_position = current_pos;
        }
        m->forces = Vector2D(0, 0);
    }
}
}  // namespace CGL
