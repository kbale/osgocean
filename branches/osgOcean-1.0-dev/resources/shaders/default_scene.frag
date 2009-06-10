// osgOcean Uniforms
// -----------------
uniform int uLightID;

uniform float uDOF_Near;
uniform float uDOF_Focus;
uniform float uDOF_Far;
uniform float uDOF_Clamp;

uniform float uUnderwaterFogDensity;
uniform float uAboveWaterFogDensity;   
uniform vec4 uUnderwaterFogColor;
uniform vec4 uAboveWaterFogColor;

uniform vec4 uUnderwaterDiffuse;

uniform float uWaterHeight;

uniform bool uEnableGlare;
uniform bool uEnableDOF;
uniform bool uEyeUnderwater;
// -------------------

uniform sampler2D uTextureMap;

varying vec3 vNormal;
varying vec3 vLightDir;
varying vec3 vEyeVec;
varying float vWorldHeight;

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

vec4 lighting( vec4 diffuse, vec4 colormap )
{
	vec4 final_color = gl_LightSource[uLightID].ambient * colormap;
							
	vec3 N = normalize(vNormal);
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
	vec4 textureColor = texture2D( uTextureMap, gl_TexCoord[0].st );

	vec4 final_color;

	float alpha;

	// +2 tweak here as waves peak above average wave height,
	// and surface fog becomes visible.
	if(uEyeUnderwater && vWorldHeight < uWaterHeight+2.0 ) 
	{
		final_color = lighting( uUnderwaterDiffuse, textureColor );
		
		float fogFactor = exp2(uUnderwaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );
		final_color = mix( uUnderwaterFogColor, final_color, fogFactor );

		if(uEnableDOF)
			final_color.a = computeDepthBlur(gl_FogFragCoord, uDOF_Focus, uDOF_Near, uDOF_Far, uDOF_Clamp);
	}
	else
	{
		final_color = lighting( gl_LightSource[0].diffuse, textureColor );

		float fogFactor = exp2(uAboveWaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );
		final_color = mix( uAboveWaterFogColor, final_color, fogFactor );

		if(uEnableGlare)
			final_color.a = 0.0;
	}

	gl_FragColor = final_color;
}
