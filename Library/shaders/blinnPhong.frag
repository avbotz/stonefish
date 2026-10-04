/*    
    Copyright (c) 2019 Patryk Cieslak. All rights reserved.

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

//Blinn-Phong model
uniform float shininess;
uniform float specularStrength;

const float PI = 3.14159265359;

//Energy-normalized Blinn-Phong (Lambertian diffuse, normalized specular lobe)
vec3 ShadingModel(vec3 N, vec3 V, vec3 L, vec3 Lcolor, vec3 albedo)
{
	vec3 H = normalize(V+L);
	float NdotL = max(dot(N, L), 0.0);
	float specular = (shininess + 8.0)/(8.0 * PI) * pow(max(dot(N, H), 0.0), shininess) * specularStrength;
    
	return Lcolor * (albedo / PI + specular) * NdotL;
}

//Environment lighting represented by the irradiance E, received from a uniformly bright hemisphere
vec3 AmbientShadingModel(vec3 N, vec3 V, vec3 E, vec3 albedo)
{
	return E / PI * (albedo + vec3(specularStrength));
}

