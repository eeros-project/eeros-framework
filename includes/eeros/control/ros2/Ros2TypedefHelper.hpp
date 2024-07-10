#pragma once

namespace eeros {
namespace control {

/**
 * Type to define the plane to which the 2D vector is oriented.
 */
typedef enum{
  PLANE_XY,
  PLANE_XZ,
  PLANE_YZ
}Plane_t;

/**
 * Type to define the axe to which the double is oriented.
 */
typedef enum{
  AXE_X,
  AXE_Y,
  AXE_Z
}Axe_t;

} /* END Namespace: control */
} /* END Namespace: eeros */