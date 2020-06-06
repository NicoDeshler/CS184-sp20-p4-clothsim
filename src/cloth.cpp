#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.    
    double dx = width / ((double)num_width_points - 1.);
    double dy = height / ((double)num_height_points - 1.);
    double x;
    double y;
    double z;
    int i;
    int j;

    for (i = 0; i < num_height_points; i++) {
        for (j = 0, x = 0; j < num_width_points; j++, x += dx) {
            y = i * dy;
            x = j * dx;

            // set position based on orientation
            if (orientation == HORIZONTAL) {
                z = y;
                y = 1.;
            }
            else {
                z = (rand() - (0.5 * RAND_MAX)) / RAND_MAX / 500;
            }

            // Create the cloth point mass and append to point mass vector (row-major order)
            Vector3D position(x, y, z);
            //PointMass pm(position, isPinned);
            //point_masses.push_back(pm);
            point_masses.emplace_back(position, false);
        }
    }
    // Set pinned point masses
    for (int k = 0; k < pinned.size(); k++) {
        i = pinned[k][0];
        j = pinned[k][1];
        PointMass* pinned_pm = &point_masses[num_width_points * i + j];
        pinned_pm->pinned = true;
    };
 
    // Add structural springs
    for (i = 0; i < num_height_points; i++) {
        for (j = 0; j < num_width_points; j++) {
            PointMass* pm = &point_masses[num_width_points * i + j];
            if (i == 0 && j == 0) {
                //skip
            }
            else if (i == 0) {
                PointMass* leftpm = &point_masses[num_width_points * i + (j - 1)];
                //Spring s(pm, leftpm, STRUCTURAL);
                //springs.push_back(s);
                springs.emplace_back(pm, leftpm, STRUCTURAL);
            }
            else if (j == 0) {
                PointMass* toppm = &point_masses[num_width_points * (i - 1) + j];
                //Spring s(pm, toppm, STRUCTURAL);
                //springs.push_back(s);
                springs.emplace_back(pm, toppm, STRUCTURAL);
            }
            else {
                //cout << "added spring" << "i:" << i << "j:" << j;
                PointMass* leftpm = &point_masses[num_width_points * i + (j - 1)];
                PointMass* toppm = &point_masses[num_width_points * (i - 1) + j];
                //Spring s1(pm, leftpm, STRUCTURAL);
                //Spring s2(pm, toppm, STRUCTURAL);
                //springs.push_back(s1);
                //springs.push_back(s2);
                springs.emplace_back(pm, leftpm, STRUCTURAL);
                springs.emplace_back(pm, toppm, STRUCTURAL);

            }
        }
    }
    
    // Add shearing springs
    for (i = 1; i < num_height_points; i++) {
        for (j = 1; j < num_width_points; j++) {
            PointMass* br = &point_masses[num_width_points * i + j];                      // bottom right
            PointMass* bl = &point_masses[num_width_points * i + (j - 1)];                // bottom left
            PointMass* tr = &point_masses[num_width_points * (i - 1) + j];                // top right
            PointMass* tl = &point_masses[num_width_points * (i - 1) + (j - 1)];          // top left

            //Spring diag1(br, tl, SHEARING); 
            //Spring diag2(bl, tr, SHEARING);

            //springs.push_back(diag1);
            //springs.push_back(diag2);

            springs.emplace_back(br, tl, SHEARING);
            springs.emplace_back(bl, tr, SHEARING);

        }
    }

    
    // Add bending springs
    for (i = 0; i < num_height_points; i++) {
        for (j = 0; j < num_width_points; j++) {
            PointMass* pm = &point_masses[num_width_points * i + j];
            if (i - 2 < 0) {
                if ((j - 2) >= 0) {
                    PointMass* llpm = &point_masses[num_width_points * i + (j - 2)];
                    //Spring s(pm, llpm, BENDING);
                    //springs.push_back(s);
                    springs.emplace_back(pm, llpm, BENDING);
                  
                }
            } else if (j-2 < 0) {
                if ((i-2) >= 0) {
                    PointMass* ttpm = &point_masses[num_width_points * (i - 2) + j];
                    //Spring s(pm, ttpm, BENDING);
                    //springs.push_back(s);
                    springs.emplace_back(pm, ttpm, BENDING);
                }
            } else {
                PointMass* llpm = &point_masses[num_width_points * i + (j - 2)];
                PointMass* ttpm = &point_masses[num_width_points * (i - 2) + j];
                //Spring s1(pm, llpm, BENDING);
                //Spring s2(pm, ttpm, BENDING);
                //springs.push_back(s1);
                //springs.push_back(s2);

                springs.emplace_back(pm, llpm, BENDING);
                springs.emplace_back(pm, ttpm, BENDING);
            }
        }
    }

    
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* pm = &point_masses[i];
      for (int j = 0; j < external_accelerations.size(); j++) {
          pm->forces += mass * external_accelerations[j];
      }
  }
  
  
  for (int i = 0; i < springs.size(); i++) {
      Spring* s = &springs[i];
      Vector3D pa = s->pm_a->position;
      Vector3D pb = s->pm_b->position;
      double rl = s->rest_length;

      if (cp->enable_structural_constraints) {
          //APPLY STRUCTURAL SPRING FORCES
          s->pm_a->forces += cp->ks * ((pa - pb).norm() - rl) * (pb - pa).unit();
          s->pm_b->forces += cp->ks * ((pa - pb).norm() - rl) * (pa - pb).unit();
      } 
      if (cp->enable_shearing_constraints) {
          //APPLY SHEARING SPRING FORCES
          s->pm_a->forces += cp->ks * ((pa - pb).norm() - rl) * (pb - pa).unit();
          s->pm_b->forces += cp->ks * ((pa - pb).norm() - rl) * (pa - pb).unit();
      }
      if (cp->enable_bending_constraints) {
          //APPLY BENDING SPRING FORCES
          s->pm_a->forces += cp->ks * ((pa - pb).norm() - rl) * (pb - pa).unit();
          s->pm_b->forces += cp->ks * ((pa - pb).norm() - rl) * (pa - pb).unit();
      }
  }
  // TODO (Part 2): Use Verlet integration to compute new point mass positions

  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* pm = &point_masses[i];
      
      // Update position using Verlet Integration if point mass is not pinned
      if (!pm->pinned) {
          Vector3D temp_pos = pm->position;
          pm->position = pm->position + (1. - cp->damping / 100.) * (pm->position - pm->last_position) + (pm->forces / mass) * (delta_t * delta_t);
          pm->last_position = temp_pos;
      }
      pm->forces = Vector3D(0);
  }


  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* pm = &point_masses[i];
      self_collide(*pm, simulation_steps);
  }
  


  // TODO (Part 3): Handle collisions with other primitives.
  for (int i = 0; i < (*collision_objects).size(); i++) {
      CollisionObject* obj = (*collision_objects)[i];
      for (int j = 0; j < point_masses.size(); j++) {
          PointMass* pm = &point_masses[j];
          obj->collide(*pm);
      }    
  }



  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  
  bool has_overstretch = false;
  int max_iters = 1;        // Provot's paper only does 1 iteration. But in theory, a more robust method would be to loop several times. There is a possibility of infinite looping if max iteration number is not set. 
  do {
      // Switch turns on only if there is an overstretched spring in the mesh
      bool stretch_switch = false;
      for (int i = 0; i < springs.size(); i++) {

          Spring* s = &springs[i];
          PointMass* pm_a = s->pm_a;
          PointMass* pm_b = s->pm_b;

          double spring_length = (pm_a->position - pm_b->position).norm();
          double overstretch_length = spring_length - (s->rest_length * 1.1);

          // Turn on switch if any of the springs are too long
          stretch_switch = stretch_switch || (overstretch_length > 0);

          // Shorten overstretched springs
          if (overstretch_length > 0) {
              
              if (pm_a->pinned && pm_b->pinned) {
                // do nothing
              }
              else if (pm_a->pinned) {
                  pm_b->position += overstretch_length * (pm_a->position - pm_b->position).unit();
              }
              else if (pm_b->pinned) {
                  pm_a->position += overstretch_length * (pm_b->position - pm_a->position).unit();
              }
              else {
                  pm_a->position += (overstretch_length / 2) * (pm_b->position - pm_a->position).unit();
                  pm_b->position += (overstretch_length / 2) * (pm_a->position - pm_b->position).unit();
          
              }      
          }     
      }
  has_overstretch = stretch_switch;
  max_iters -= 1;
 } while (has_overstretch && (max_iters != 0));
 


}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* pm = &point_masses[i];
      float hash = hash_position(pm->position);
      if (map.find(hash) == map.end()) {
          map[hash] = new vector <PointMass*>();
      }
      map[hash]->push_back(pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    float hash = hash_position(pm.position);
    Vector3D correction(0);
    int count = 0;

    for (PointMass* candidate_pm : *(map[hash])) {
        if (&(*candidate_pm) == &pm) {
            // SKIP
        }
        else {
            //cout << candidate_pm->position << "\n";
            //cout << pm.position << "\n";
            Vector3D sep = (candidate_pm->position - pm.position); // separation vector between candidate point in spatial box and target point
            if (sep.norm() < 2 * thickness) {
                count += 1;
                correction += sep - 2 * thickness * sep.unit();
            }
        } 
    }

    if (count) {
        correction /= (count * simulation_steps);
        pm.position += correction;
    }

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

    float w = 3 * width / num_width_points;
    float h = 3 * height / num_height_points;
    float t = max(w, h);
  // Find discrete coordinates of the spatial partition box in which pos resides.
  // Discrete box coordinate in partitioned space
    int bx = floor(pos.x / w);
    int by = floor(pos.y / h);
    int bz = floor(pos.z / t);

  // Hash value
    return bx + by * by + bz * bz * bz;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
