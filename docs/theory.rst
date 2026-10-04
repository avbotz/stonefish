======
Theory
======

Unit system
===========

The `International System of Units (SI) <https://en.wikipedia.org/wiki/International_System_of_Units>`_ is used for the definitions and computations throughout the simulation library. For convenience some of the angular quantities are specified in degrees; the computations are always done using radians.

Coordinate frames
=================

Multiple coordinate frames are defined in marine craft theory.
At this stage, the *Stonefish* library is intended for small scale simulations where Earth curvature can be neglected. Therefore, the **North-East-Down (NED) coordinate frame** is used throughout the simulation, as the world reference frame. The NED frame is a Cartesian coordinate frame, tangent to the Earth surface at a chosen geographic location. This location is called the home and defined by latitude and longitude.

Rigid body dynamics and collision
=================================

The *Stonefish* library utilises algoritms implemented in the *Bullet Physics* library for the computation of the rigid body kinematics, dynamics and collision. The simulation engine is implementing velocity-based dynamics with semi-implicit Euler integration. The dynamics are computed using the Sequential Impulse algorithm for separate dynamical bodies. Kinematic trees are handled through an implementation of the Featherstone multi-body algorithm. Rigid collisions are computed using an impulse-based approach. Soft collisions are possible by defining collision stiffeness and damping factors. Frictional forces are computed based on static and dynamic friction coefficients defined between materials. The Stribeck function is used for the transition between sticking and sliding phases.

Hydrodynamics
=============

The *Stonefish* library delivers a novel approach to simulating hydrodynamics and aerodynamics, by performing geometry based computations. The main focus is put on hydrodynamics as the library is directed towards marine robotics. The simulated effects include: added mass, buoyancy and drag. The drag is composed of 2 elements: form drag (quadratic) and skin friction. Hydrodynamic lift is not computed for general bodies but can be found in the model of a rudder actuator, which is considered an actuated hydrofoil.

Added mass
----------

The added mass effect is encountered during the acceleration of bodies submerged in liquid. It is manifested by the dynamical response of the body as if it was heavier than it actually is, because the surrounding liquid has to be moved together with it. The added mass is formulated as a 6x6 matrix, which describes how acceleration in every direction is affected by the fluid. The library uses the translational part (a 3x3 added mass tensor) and the diagonal of the rotational part (added moments of inertia). Their values are computed from an automatic approximation of the body geometry using one of the 3 solids: sphere, cylinder or ellipsoid. The ellipsoid has the same second moments of volume as the body (computed from the closed physics mesh) and its added mass is obtained from the exact potential flow solution (H. Lamb, *Hydrodynamics*), which covers the whole range of shapes from slender rods to thin plates. The added mass of compound bodies is the sum of the added mass of their external parts, including the contribution of the parts' added mass to the moments of inertia of the compound body. The automatically computed values can be replaced by the user, e.g., with values identified experimentally.

The underlying physics library supports only a scalar mass of a body. Therefore, the body is simulated with the mean augmented mass and the external force acting on the body is corrected, in each simulation step, so that the translational motion follows the full added mass tensor. The correction includes the Coriolis and centripetal terms of the added mass and the Munk moment, which tends to turn a body moving at an angle of attack broadside to the flow (T.I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*). The added mass is scaled with the submerged fraction of the body volume, so a body leaving the water behaves as if it had only its own mass.

Buoyancy
--------

The buoyancy force is calculated based on the sum of hydrostatic forces acting on the body surface. The actual geometry is used to compute force at each face of the mesh, depending on the depth of the face centre. It allows for simulating realistic buoyancy force at the surface of the ocean, with and without geometrical waves. When the body is completely submerged the buoyancy force is based on the volume of the mesh, computed automatically during loading. Bodies defined with a wall thickness (shells) are considered flooded, i.e., only the volume of the walls displaces the liquid, both under water and at the surface.

Drag
----

The drag forces are calculated as a sum of forces acting on each face of the body surface. To obtain precise values of these forces it is required to solve Navier-Stokes equations, which is not possible for a general 3D case in realtime. Therefore, the computations implemented in the *Stonefish* library have to be based on the local velocity of fluid as if there was no body. The result is not quantitively correct but it gives a good approximation and allows for effects not possible when using simple formulas, e.g., a water current acting on a part of the body.

The form (pressure) drag of each face facing the flow is proportional to the dynamic pressure and to the area of the face projected onto the flow direction, :math:`\frac{1}{2}\rho C_d A \cos\theta |v|^2`. Summed over the whole body, it gives the standard drag equation with the drag coefficient :math:`C_d` defined with respect to the frontal area. The skin friction of each face is computed for a turbulent boundary layer, :math:`\frac{1}{2}\rho C_f A |v_t| v_t`, where :math:`v_t` is the tangential velocity of the flow. The skin friction coefficient is estimated from the Reynolds number of the body, using the ITTC-1957 correlation line, unless it is defined by the user. The velocity of the flow at each face includes the motion of the body (linear and angular) and the water currents.