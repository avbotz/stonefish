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
    Intensity of the refracted sun light, relative to a flat surface.
    The light passing through a patch of the surface is spread over (or focused into) the patch of the map
    covered by the fragment, so the intensity is the ratio of the two areas. Overlapping triangles (folds)
    are summed with additive blending.
*/

uniform float mapSize; //Resolution of the map [texels]

in vec2 surfacePos;
in vec2 mapPos;

out vec4 fragColor;

void main()
{
    //Area of the surface mapped to this texel, divided by the area of the texel
    vec2 dx = dFdx(surfacePos);
    vec2 dy = dFdy(surfacePos);
    float sourceArea = abs(dx.x * dy.y - dx.y * dy.x);
    float texelArea = 1.0/(mapSize * mapSize);
    float intensity = min(sourceArea/texelArea, 50.0); //Limit singularities at the focal lines
    fragColor = vec4(intensity, 0.0, 0.0, 1.0);
}
