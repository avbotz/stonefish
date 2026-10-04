.. _cables:

======
Cables
======

Cables are soft bodies that can be used to simulate lines, ropes, tethers, etc. 
They can be defined as elastic or inelastic, and can be attached to the world or to dynamic bodies, at both ends, only at one end, or left completely free.

Properties
==========

A cable is created between two specified points in the world frame, and the resulting distance is divided into a specified number of segments. 
Cable diameter and physical material are used to compute the distributed mass. Moreover, a stretch factor can be specified to allow for elastic materials.

.. warning::

    The stretch factor is not a physical quantity, neither it represents the stretch fraction, although its range is limitted to [0, 1]. It is passed directly to the soft body configuration of the physics engine. 
    The user needs to experiment with its value to obtain the desired behaviour, which will depend on the update rate of the simulation and the mass of any objects attached to the cable.

Collisions
==========

Cables can collide with dynamic bodies, static bodies, and robot links. However, collision between cables, as well as, self-collisions are not supported.

Hydrodynamics
=============

For cables in the "submerged" or "floating" physics mode, the buoyancy and the hydrodynamic drag are computed for each segment, based on its submerged volume.
The drag is computed using the Morison equation for a cylinder, separately for the components of the flow velocity relative to the segment, normal (``v_n``) and tangential (``v_t``) to the segment axis:

- form drag ``0.5 * rho * Cd * d * L * |v_n| * v_n``, with ``Cd = 1.2``,
- skin friction ``0.5 * rho * Cf * pi * d * L * |v_t| * v_t``, with ``Cf = 0.01``,

where ``rho`` is the density of the fluid, ``d`` is the cable diameter and ``L`` is the submerged length of the segment.
The segment forces are distributed to the nodes of the cable consistently with their lumped masses (all nodes have equal mass), so that the total buoyancy of a submerged cable equals the weight of the displaced fluid and a neutrally buoyant cable stays in equilibrium.

.. note::

    The fluid forces are updated at a lower rate than the simulation (see the ``<fluid_dynamics prescaler="..."/>`` solver setting) and applied unchanged in between. To keep the simulation stable, the drag acting on very thin and light cables is limited, so that it cannot reverse the relative velocity of the cable nodes between two updates.

Anchoring
=========

Cables can be free or anchored at one or both ends. An anchor can be fixed to the world or to a dynamic body. Anchoring cables directly to the robot links is not supported by the physics engine.
There are three anchor types implemented: "free", "world", and "dynamic". Creating a dynamic anchor requires specifying the dynamic body name.

.. note::

    Anchoring a cable to a robot link can be achieved by creating a small dynamic body attached to the link with a fixed constraint, and anchoring the cable to that body. 
    However, the results of this trick will depend on the mass of the small body and the cable.

Instantiation
=============

Cables can be instantiated as follows:

.. code-block:: xml

     <dynamic name="Sphere" type="sphere" physics="submerged" buoyant="true">
        <dimensions radius="0.5"/>    
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>    
        <material name="Steel"/>
        <look name="Yellow"/>
        <world_transform xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </dynamic>
    <cable name="Cable" physics="submerged" buoyant="true" collisions="true">
        <geometry diameter="0.01" number_of_segments="100"/>
        <material name="Steel" stretch_factor="0.0"/>
        <look name="Yellow" uv_scale="10.0"/>
        <first_end position="0.0 0.0 -5.0" anchor="world"/>
        <second_end position="0.0 0.0 -1.0" anchor="dynamic">
            <body name="Sphere"/>
        </second_end>
    </cable>

.. code-block:: cpp

    #include <Stonefish/entities/solids/Sphere.h>
    #include <Stonefish/entities/CableEntity.h>
    
    sf::PhysicsSettings phy;
    phy.mode = sf::PhysicsMode::SUBMERGED;
    phy.collisions = true;
    phy.buoyancy = true;

    sf::SolidEntity* sphere = AddSolidEntity(std::make_unique<sf::Sphere>("Sphere", phy, 0.5, sf::I4(), "Steel", "Yellow"), sf::I4());
    std::unique_ptr<sf::CableEntity> cable = std::make_unique<sf::CableEntity>("Cable", phy, sf::Vector3(0.0, 0.0, -5.0), 
        sf::Vector3(0.0, 0.0, -1.0), 100, 0.01, "Steel", "Yellow", 0.0, 10.f);
    cable->AttachToWorld(sf::CableEnds::FIRST);
    cable->AttachToSolid(sf::CableEnds::SECOND, sphere);
    AddCableEntity(std::move(cable));
