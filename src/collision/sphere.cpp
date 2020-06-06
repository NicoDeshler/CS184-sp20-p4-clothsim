#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
    Vector3D phi = pm.position - origin;                // vector pointing outwards from sphere's origin to point mass
    double surf_disp = phi.norm() - radius;             // radial diplacement from sphere surface

    if (surf_disp < 0) {

        Vector3D tangent = origin + radius * phi.unit();    // tangent point where the sphere should intersect
        Vector3D correction = tangent - pm.last_position;        // correction vector for last_position to propogate to the tangent
        pm.position = pm.last_position + (1 - friction) * correction;
    }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
