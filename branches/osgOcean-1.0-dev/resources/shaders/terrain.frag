uniform sampler2D uTextureMap;
uniform sampler2D uOverlayMap;
uniform sampler2D uNormalMap;

// osgOcean uniforms
// -------------------
uniform float uDOF_Near;
uniform float uDOF_Focus;
uniform float uDOF_Far;
uniform float uDOF_Clamp;

uniform bool uEnableGlare;
uniform bool uEnableDOF;
uniform bool uEyeUnderwater;

uniform float uUnderwaterFogDensity; 
uniform float uAboveWaterFogDensity; 
uniform float uWaterHeight;

uniform int uLightID;

uniform vec4 uUnderwaterFogColor;
uniform vec4 uAboveWaterFogColor;
uniform vec4 uUnderwaterDiffuse;
// -------------------

varying vec3 vLightDir;
varying vec3 vEyeVec;

varying float vWorldHeight;
varying float unitHeight;

float computeDepthBlur(float depth, float focus, float near, float far, float clampval )
{
   float f;
   if (depth < focus){
      f = (depth - focus)/(focus - near);
   }
   else{
      f = (depth - focus)/(far - focus);
      f = clamp(f, 0.0, clampval);
   }
   return f * 0.5 + 0.5;
}

vec4 lighting( vec4 diffuse, vec4 colormap, vec3 N )
{
	vec4 final_color = gl_LightSource[uLightID].ambient * colormap;
							
	vec3 L = normalize(vLightDir);
	
	float lambertTerm = dot(N,L);
	
	if(lambertTerm > 0.0)
	{
		final_color += diffuse * lambertTerm * colormap;	
		
		vec3 E = normalize(vEyeVec);
		vec3 R = reflect(-L, N);
		
		float specular = pow( max(dot(R, E), 0.0), 2.0 );

		final_color += gl_LightSource[uLightID].specular * specular;	
	}

	return final_color;
}

void main(void)
{
	vec4 baseColor    = texture2D( uTextureMap, gl_TexCoord[0].st );
	vec4 overlayColor = texture2D( uOverlayMap, gl_TexCoord[1].st );
	vec4 bumpColor    = texture2D( uNormalMap,  gl_TexCoord[0].st );

	unitHeight = clamp( unitHeight, 0.0, 1.0);
	vec4 textureColor = mix(overlayColor, baseColor, unitHeight);

	vec3 bump = (bumpColor.xyz*2.0)-1.0;

	float alpha;
	float fogFactor;
	vec4 fogColor;
	vec4 final_color;

	// +2 tweak here as waves peak above average wave height,
	// and surface fog becomes visible.
	if(uEyeUnderwater && vWorldHeight < uWaterHeight+2.0)
	{
		final_color = lighting( uUnderwaterDiffuse, textureColor, bump );
		
		fogColor = uUnderwaterFogColor;
		fogFactor = exp2(uUnderwaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );

		if(uEnableDOF)
			alpha = computeDepthBlur( gl_FogFragCoord, uDOF_Focus, uDOF_Near, uDOF_Far, uDOF_Clamp );
		else
			alpha = final_color.a;
	}
	else
	{
		final_color = lighting( gl_LightSource[uLightID].diffuse, textureColor, bump );

		fogColor = uAboveWaterFogColor;
		fogFactor = exp2(uAboveWaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );
		
		if(uEnableGlare)
			alpha = 0.0;
		else
			alpha = final_color.a;
	}
	
	gl_FragColor = mix( fogColor, final_color, fogFactor );
	gl_FragColor.a = alpha;
}