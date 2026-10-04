/*
    Copyright (c) 2026 Patryk Cieslak. All rights reserved.

    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#version 330

/*
    Traces the sun light refracted by the rippled water surface down to a given depth.
    Each vertex of the grid is a point on the surface; it is moved to the place where its ray reaches the depth,
    expressed relative to the point reached by the ray refracted by a flat surface (as looked up in oceanOptics.frag).
*/

layout(location = 0) in vec2 gridCoord; //Coordinate in the grid <0,1>

uniform sampler2D texRipples;
uniform vec3 sunDir; //Unit vector pointing towards the sun (z axis pointing down)
uniform float tileSize; //Size of the ripples tile [m]
uniform float depth; //Depth of the map [m]
uniform float margin; //Extension of the grid beyond the tile (fraction of the tile)

out vec2 surfacePos; //Position of the point on the surface (in tiles)
out vec2 mapPos; //Position of the refracted light in the map (in tiles)

const float air2water = 1.0/1.33;

void main()
{
    //Point on the surface (grid extended to gather the light refracted from the neighbouring tiles)
    surfacePos = mix(vec2(-margin), vec2(1.0 + margin), gridCoord);

    //Normal of the surface pointing up (towards the air)
    vec2 slopes = textureLod(texRipples, surfacePos, 0.0).yz;
    vec3 N = normalize(vec3(-slopes, -1.0));
    vec3 L = -sunDir; //Direction of propagation of the sun light

    //Refracted ray and the ray refracted by a flat surface
    vec3 T = refract(L, N, air2water);
    vec3 T0 = refract(L, vec3(0.0, 0.0, -1.0), air2water);
    T.z = max(T.z, 1e-3);

    mapPos = surfacePos + (T.xy/T.z - T0.xy/T0.z) * depth/tileSize;
    gl_Position = vec4(mapPos * 2.0 - 1.0, 0.0, 1.0);
}
