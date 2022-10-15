#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // std::cout << "[Rope] " << "Start: " << start << " End: " << end << " num: " << num_nodes << " mass: "  << node_mass << " k: " << k << std::endl;
        if (num_nodes < 2) return;

        Vector2D delta = (end - start) / (num_nodes - 1);
        // std::cout << "[Rope] " << "Delta: " << delta << std::endl;

        Mass * prePoint = nullptr;
        for (int i = 0; i < num_nodes; i++) {
            Vector2D pos = start + delta * i;
            Mass * point = new Mass(pos, node_mass, false);
            //std::cout << "[Rope] " << "Pos: " << pos << " Point: " << point->position << std::endl;
            masses.push_back(point);

            if (i != 0) {
                Spring * s = new Spring(prePoint, point, k);
                //std::cout << "[Rope] " << "PreP: " << prePoint->position << " CurP: " << point->position << " Spring: " << s << std::endl;
                springs.push_back(s);
            }

            prePoint = point;
        }

        // Comment-in this part when you implement the constructor
        // std::cout << "[Rope] " << "Pinned: ";
        for (auto &i : pinned_nodes) {
            // std::cout << i << ", ";
            masses[i]->pinned = true;
        }
        // std::cout << std::endl;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            // f_{b->a} = - K_s * (b-a) / ||b-a|| * (||b-a|| - l)
            Vector2D vec = s->m2->position - s->m1->position;
            Vector2D force = s->k * vec / vec.norm() * (vec.norm() - s->rest_length);
            s->m1->forces += force;
            s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;

                // TODO (Part 2): Add global damping
                m->forces -= 0.01 * m->velocity;

                // Refresh mass position.
                // There are two method: Explicit method and semi-implicit method.

                Vector2D a = m->forces / m->mass;

                // {
                //    // Explicit method
                //    // x(t+1) = x(t) + v(t) * dt
                //    m->position += m->velocity * delta_t;
                //    m->velocity += a * delta_t;
                // }

                {
                    // Semi-implicit method
                    // x(t+1) = x(t) + v(t+1) * dt
                    m->velocity += a * delta_t;
                    m->position += m->velocity * delta_t;
                }
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D vec = s->m2->position - s->m1->position;
            Vector2D force = s->k * vec / vec.norm() * (vec.norm() - s->rest_length);
            s->m1->forces += force;
            s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;

                // TODO (Part 4): Add global Verlet damping
                // x(t+1) = x(t) + (1 - damp_factor) * (x(t) - x(t-1)) + a(t) * dt * dt
                Vector2D a = m->forces / m->mass;
                float damp_factor = 0.00005;
                Vector2D x_t =  m->position;
                Vector2D x_t_1 =  m->last_position;

                m->position = x_t + ( 1 - damp_factor) * (x_t - x_t_1) + a * delta_t * delta_t;
                m->last_position = x_t;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
