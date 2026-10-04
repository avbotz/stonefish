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
    Height field of capillary ripples, periodic over a square tile.
    Output: x = elevation of the surface [m], yz = gradient of the elevation (slopes along world X and Y), w = 1.
*/

#define MAX_RIPPLE_WAVES 32 //Keep in sync with OpenGLOcean.h

uniform vec4 waves[MAX_RIPPLE_WAVES]; //Wave vector [rad/m], amplitude [m], phase at current time [rad]
uniform int numWaves;
uniform float tileSize;

in vec2 texcoord;
out vec4 fragColor;

void main()
{
    vec2 x = texcoord * tileSize;
    float h = 0.0;
    vec2 slope = vec2(0.0);

    for(int i=0; i<numWaves; ++i)
    {
        float theta = dot(waves[i].xy, x) + waves[i].w;
        h += waves[i].z * cos(theta);
        slope -= waves[i].z * sin(theta) * waves[i].xy;
    }

    fragColor = vec4(h, slope, 1.0);
}
