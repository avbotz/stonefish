/*    
    Copyright (c) 2020 Patryk Cieslak. All rights reserved.

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

/*
    Based on "Real-time Animation and Rendering of Ocean Whitecaps"
    by Jonathan Dupuy and Eric Bruneton.
    https://github.com/jdupuy/whitecaps
*/

#version 330

in vec4 fragPos;
in float logz;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec4 fragNormal;

uniform sampler2DArray texWaveFFT;
uniform sampler3D texSlopeVariance;
uniform vec2 viewport;
uniform vec4 gridSizes;
uniform vec3 eyePos;
uniform mat3 MV;
uniform float FC;
uniform sampler2D texRipples;
uniform float flatOcean; //Surface normals computed from capillary ripples instead of FFT waves
uniform sampler2D texReflection; //Mirror image of the underwater scene (flat surface)
uniform float reflectionEnabled;

const float rippleTileSize = 2.5; //Keep in sync with OpenGLOcean.h

#inject "lightingDef.glsl"

//Atmosphere
vec3 GetSolarLuminance();
vec3 GetSkyLuminance(vec3 camera, vec3 view_ray, float shadow_length, vec3 sun_direction, out vec3 transmittance);
vec3 GetSkyLuminanceToPoint(vec3 camera, vec3 point, float shadow_length, vec3 sun_direction, out vec3 transmittance);
vec3 GetSunAndSkyIlluminance(vec3 p, vec3 normal, vec3 sun_direction, out vec3 sky_irradiance);
vec3 RefractToWater(vec3 I, vec3 N);
vec3 RefractToAir(vec3 I, vec3 N);
vec3 BeerLambert(float d);
float FresnelDielectric(float cosi, float n1, float n2);
vec3 InScatteringSun(vec3 L, vec3 D, vec3 V, float z, float d);
vec3 InScatteringPointLight(vec3 O, vec3 L, vec3 X, vec3 V, vec3 P, float d, float dw);
vec3 InScatteringSpotLight(vec3 O, vec3 D, float w, float fn, vec3 L, vec3 X, vec3 V, vec3 P, float d, float dw);

const float M_PI = 3.14159265358979323846;
const vec3 waterSurfaceN = vec3(0.0, 0.0, -1.0);

void main()
{
    //Logarithmic z-buffer correction
	gl_FragDepth = log2(logz) * FC;

	vec3 P = fragPos.xyz/fragPos.w;
	vec3 toEye = eyePos - P;
	float d = length(toEye);
	vec3 V = toEye/d;
	vec3 center = vec3(0, 0, planetRadiusInUnits);
	vec3 Psky = vec3(P.xy/atmLengthUnitInMeters, clamp(P.z/atmLengthUnitInMeters, -100000.0/atmLengthUnitInMeters, -0.5/atmLengthUnitInMeters));
    float dw = d;

	if(eyePos.z < 0.0)
		dw = max(P.z, 0.0)/dot(V, waterSurfaceN);
	
	//Wave slope
    vec2 waveCoord = P.xy;
	vec2 slopes;
	if(flatOcean > 0.5) //Capillary ripples only
		slopes = texture(texRipples, waveCoord/rippleTileSize).yz;
	else //Layers 1,2 of the FFT ocean
	{
		slopes = texture(texWaveFFT, vec3(waveCoord/gridSizes.x, 1.0)).xy;
		slopes += texture(texWaveFFT, vec3(waveCoord/gridSizes.y, 1.0)).zw;
		slopes += texture(texWaveFFT, vec3(waveCoord/gridSizes.z, 2.0)).xy;
		slopes += texture(texWaveFFT, vec3(waveCoord/gridSizes.w, 2.0)).zw;
	}
		
	//Normal (pointing into water)
	vec3 normal = normalize(vec3(slopes.x, slopes.y, 1.0));
    
    //Reflection/refraction ratio (total internal reflection outside of the Snell's window)
	float fresnel = FresnelDielectric(dot(V, normal), 1.33, 1.0);
    
    //Sky seen through the surface (radiance grows with the square of the refractive index)
    vec3 Lsky = vec3(0.);
	if(fresnel < 1.0)
	{
		vec3 ray = RefractToAir(-V, normal);
		if(ray.z < 0.0)
		{
			vec3 trans;
			Lsky = GetSkyLuminance(Psky - center, ray, 0.0, sunDirection, trans) * (1.33 * 1.33);
		}
	}

	//Water reflected by the surface (replaced by the reflection of the scene where available)
	vec3 skyIlluminance;
	vec3 sunIlluminance = GetSunAndSkyIlluminance(Psky - center, waterSurfaceN, sunDirection, skyIlluminance);
    vec3 S = RefractToWater(-sunDirection, waterSurfaceN);
	vec3 Lwater = vec3(0.0);
	if(fresnel > 0.0 && S.z > 0.0 && sunDirection.z < 0.0)
		Lwater = InScatteringSun(sunIlluminance/whitePoint, S, reflect(-V, normal), 0.0, 1000.0);
	if(fresnel > 0.0 && reflectionEnabled > 0.5)
	{
		//Reflected ray deviates twice as much as the normal of the rippled surface
		vec2 uv = gl_FragCoord.xy/viewport + 1.5 * (MV * vec3(slopes, 0.0)).xy;
		vec4 reflection = texture(texReflection, uv);
		vec2 edge = min(uv, vec2(1.0) - uv);
		float inside = clamp(min(edge.x, edge.y)/0.02, 0.0, 1.0); //Image not available outside of the view
		Lwater = mix(Lwater, reflection.rgb, reflection.a * inside);
	}
	
	//Final color
    fragColor = (1.0-fresnel) * Lsky/whitePoint + fresnel * Lwater;
	
    //Attenuation
	fragColor *= BeerLambert(dw);
	
    //In-scattering
    if(S.z > 0.0 && sunDirection.z < 0.0)
	    fragColor += InScatteringSun(sunIlluminance/whitePoint, S, -V, max(eyePos.z, 0.0), dw);
    
    //In-scattering from point lights
	for(int i=0; i<numPointLights; ++i)
	{
		if(pointLights[i].position.z > 0.0)
			fragColor += InScatteringPointLight(pointLights[i].position, 
												pointLights[i].color,
												eyePos, -V, P, d, dw);
	}

	//In-scattering from spot lights
	for(int i=0; i<numSpotLights; ++i)
	{
		if(spotLights[i].position.z > 0.0)
			fragColor += InScatteringSpotLight(spotLights[i].position,
											   spotLights[i].direction,
											   spotLights[i].cone,
											   spotLights[i].frustumNear,
											   spotLights[i].color,
											   eyePos, -V, P, d, dw);
	}

	//Maximum reflectivity marks the water surface for screen-space reflections (not needed with the mirror image)
	fragNormal = vec4(normalize(MV * normal) * 0.5 + 0.5, reflectionEnabled > 0.5 ? 0.0 : 1.0);
}
