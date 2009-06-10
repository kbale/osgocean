uniform bool uEnableReflections;
uniform bool uEnableRefractions;
uniform bool uEnableGlobalReflections;
uniform bool uEnableCrestFoam;

uniform bool uEnableDOF;
uniform bool uEnableGlare;

uniform float uDOF_Near;
uniform float uDOF_Focus;
uniform float uDOF_Far;
uniform float uDOF_Clamp;
uniform float uFresnelMul;

uniform int uLightID;

uniform samplerCube uEnvironmentMap;
uniform sampler2D uReflectionMap;
uniform sampler2D uRefractionMap;
uniform sampler2D uFoamMap;
uniform sampler2D uNoiseMap;

uniform vec3 uEyePosition;
uniform vec3 uOrigin;

// HACK: use the 2D fogging texture here
uniform float uUnderwaterFogDensity; 
uniform float uAboveWaterFogDensity; 
uniform vec4 uUnderwaterFogColor;
uniform vec4 uAboveWaterFogColor;

uniform mat4 osg_ViewMatrixInverse;

uniform float uFoamCapBottom;
uniform float uFoamCapTop;

varying vec3 vNormal;		
varying vec3 vViewerDir;	
varying vec3 vLightDir;		
varying vec4 vVertex;	
varying vec4 vWorldVertex;

varying vec3 vWorldViewDir;	
varying vec3 vWorldNormal;		

mat4 worldObjectMatrix;

const float shininess = 2000.0;

vec4 distortGen( vec4 v, vec3 N )
{
	// transposed
	const mat4 mr = mat4( 0.5, 0.0, 0.0, 0.0,
								 0.0, 0.5, 0.0, 0.0,
								 0.0, 0.0, 0.5, 0.0,
								 0.5, 0.5, 0.5, 1.0 );

	mat4 texgen_matrix = mr * gl_ProjectionMatrix * gl_ModelViewMatrix;
	
	//float disp = 8.0;
	float disp = 4.0;
	
	vec4 tempPos;
	
	tempPos.xy = v.xy + disp * N.xy;
	tempPos.z  = v.z;
	tempPos.w  = 1.0;
	
	return texgen_matrix * tempPos;	
}

vec3 reorientate( vec3 v )
{
	float y = v.y;
	
	v.y = -v.z;
	v.z = y;

	return v;
}

mat3 getLinearPart( mat4 m )
{
	mat3 result;
	
	result[0][0] = m[0][0]; 
	result[0][1] = m[0][1]; 
	result[0][2] = m[0][2]; 

	result[1][0] = m[1][0]; 
	result[1][1] = m[1][1]; 
	result[1][2] = m[1][2]; 
	
	result[2][0] = m[2][0]; 
	result[2][1] = m[2][1]; 
	result[2][2] = m[2][2]; 
	
	return result;
}

vec4 computeCubeMapColor( vec3 N, vec4 V, vec3 E )
{
	mat3 worldObjectMat3x3 = getLinearPart( worldObjectMatrix );
	vec4 world_pos	= worldObjectMatrix *  V;	
	
	vec3 normal = normalize( worldObjectMat3x3 * N ); 
	vec3 eye = normalize( world_pos.xyz - E );	

	vec3 coord = reflect( eye, normal );

	vec3 reflection_vector = vec3( coord.x, coord.y, -coord.z );
	
	return textureCube(uEnvironmentMap, reflection_vector.xzy);
}

float calcFresnel( float dotEN, float mul )
{
	float fresnel = clamp( dotEN, 0.0, 1.0 ) + 1.0;
	return pow(fresnel, -8.0) * mul;	
}

float alphaHeight( float min, float max, float val)
{
	if(max-min == 0.0)
		return 1.0;
	
	return (val - min) / (max - min);
}

float computeDepthBlur(float depth, float focus, float near, float far, float clampval )
{
   float f;

   if (depth < focus){
      // scale depth value between near blur distance and focal distance to [-1, 0] range
      f = (depth - focus)/(focus - near);
   }
   else{
      // scale depth value between focal distance and far blur
      // distance to [0, 1] range
      f = (depth - focus)/(far - focus);

      // clamp the far blur to a maximum blurriness
      f = clamp(f, 0.0, clampval);
   }

   // scale and bias into [0, 1] range
   return f * 0.5 + 0.5;
}

float luminance( vec4 color )
{
	return (0.3*color.r) + (0.59*color.g) + (0.11*color.b);
}

// -------------------------------
//          Main Program
// -------------------------------

void main( void )
{
	vec4 final_color;

	vec3 noiseNormal = vec3( texture2D( uNoiseMap, gl_TexCoord[0].xy ) * 2.0 - 1.0 );
	noiseNormal += vec3( texture2D( uNoiseMap, gl_TexCoord[0].zw ) * 2.0 - 1.0 );

	worldObjectMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;

	if(gl_FrontFacing)
	{
		vec3 N = normalize( vNormal + noiseNormal );
		vec3 L = normalize( vLightDir );
		vec3 E = normalize( vViewerDir );
		vec3 R = reflect( -L, N );

		vec4 diffuse_color;
		vec4 specular_color;

		float lambertTerm = dot(N,L);

		if( lambertTerm > 0.0 )
		{
			diffuse_color =  gl_LightSource[uLightID].diffuse * lambertTerm;	
			
			float specCoeff = pow( max( dot(R, E), 0.0 ), shininess );

			specular_color = gl_LightSource[uLightID].diffuse * specCoeff * 6.0;	
		}

		float dotEN = dot(E, N);				
		float dotLN = dot(L, N);

		float dl = max( dotLN * abs(1.0 - max(dotEN, 0.2) ), 0.0);	
		
		vec4 refraction_color = vec4( gl_Color.rgb, 1.0 );
		
		// To cubemap or not to cubemap that is the question
		// projected reflection looks pretty nice anyway
		// cubemap looks wrong with fixed skydome
		//vec4 env_color = computeCubeMapColor(N, vWorldVertex, uEyePosition);
		
		vec4 env_color = texture2DProj( uReflectionMap, distortGen(vVertex, N) );

		//env_color = vec4(1.0,0.0,0.0,1.0);
			
		//vec4 env_color = texture2DProj( uReflectionMap, distortGen(vVertex, N) );
		
		env_color.a = 1.0;

		float fresnel = calcFresnel(dotEN, uFresnelMul );
		vec4 water_color = mix(refraction_color, env_color, fresnel) + specular_color;

		final_color = water_color;

		if(uEnableReflections)
		{
		//	vec4 reflect_color = texture2DProj( uReflectionMap, distortGen(vVertex, N) );

		//	if(reflect_color.a != 0.0)
		//	{
		//		final_color = vec4( mix( reflect_color.rgb, final_color.rgb, 0.9 ), 1.0 );
		//	}
		}
	
		// Calculate luminance before foam and fog.
		float lum = luminance(final_color);

		if(uEnableCrestFoam)
		{
			if( vVertex.z > uFoamCapBottom )
			{
				vec4 foam_color  = texture2D( uFoamMap, gl_TexCoord[1].st );

				float alpha = alphaHeight( uFoamCapBottom, uFoamCapTop, vVertex.z ) * (fresnel*2.0);

				final_color = mix( final_color, foam_color, alpha );
			}
		}

		// exp2 fog
		float fogFactor = exp2(uAboveWaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );
		final_color = mix( uAboveWaterFogColor, final_color, fogFactor );

		if(uEnableGlare)
		{
			final_color.a = lum;
		}

		gl_FragColor = final_color;
	}
	else
	{
		vec3 E = normalize( vViewerDir );
		vec3 N = -normalize( (vWorldNormal + noiseNormal) );
		
		vec3 incident = normalize( vWorldViewDir );
		
		//------ Find the reflection
		// not really usable as we would need to use cubemap again..
		// the ocean is blue not much to reflect back
		//vec3 reflected = reflect( incident, -N );
		//reflected		= reorientate( reflected );
		//vec3 reflVec	= normalize( reflected );

		//------ Find the refraction from cubemap
		vec3 refracted = refract( incident, N, 1.3333333333 );   // 1.1 looks better? - messes up position of godrays though
		refracted.z = refracted.z - 0.015;                       // on the fringes push it down to show base texture color
		refracted = reorientate( refracted );

		vec4 refractColor = textureCube( uEnvironmentMap, refracted );

		//------ Project texture where the light isn't internally reflected
		if(uEnableRefractions)
		{
			// if alpha is 1.0 then it's a sky pixel
			if(refractColor.a == 1.0 )
			{
				vec4 env_color = texture2DProj( uRefractionMap, distortGen(vVertex, N) );
				refractColor.rgb = mix( refractColor.rgb, env_color.rgb, env_color.a );
			}
		}

		// if it's not refracting in, add a bit of highlighting with fresnel
		// FIXME: should be using the fog map
		if( refractColor.a == 0.0 )
		{
			float fresnel = calcFresnel( dot(E, N), 0.7 );
			refractColor.rgb = uUnderwaterFogColor.rgb*fresnel + (1.0-fresnel)* refractColor.rgb;
		}

		float fogFactor = exp2(uUnderwaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );
		final_color = mix( uUnderwaterFogColor, refractColor, fogFactor );

		if(uEnableDOF)
		{
			final_color.a = computeDepthBlur( gl_FogFragCoord, uDOF_Focus, uDOF_Near, uDOF_Far, uDOF_Clamp );
		}

		gl_FragColor = final_color;
	}
}




