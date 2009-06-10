// Based on Jon Kennedy's heat haze shader
// Copyright (c) 2002-2006 3Dlabs Inc. Ltd.

uniform float uFrequency;
uniform float uOffset;
uniform int   uSpeed;
uniform vec2  uScreenRes;

uniform samplerRect uFrameBuffer;

varying vec4 vEyePos;

void main (void)
{
	vec2 index;

	// perform the div by w to put the texture into screen space
	float recipW = 1.0 / vEyePos.w;
	vec2 eye = vEyePos.xy * vec2(recipW);

	float blend = max(1.0 - eye.y, 0.0);   
	  
	// calc the wobble
	// index.s = eye.x ;
	index.s = eye.x + blend * sin( uFrequency * 5.0 * eye.x + uOffset * float(uSpeed) ) * 0.004;
	index.t = eye.y + blend * sin( uFrequency * 5.0 * eye.y + uOffset * float(uSpeed) ) * 0.004;
	  
	// scale and shift so we're in the range 0-1
	index.s = index.s * 0.5 + 0.5;
	index.t = index.t * 0.5 + 0.5;

	vec2 recipRes = vec2(1.0/uScreenRes.x, 1.0/uScreenRes.y);

	index.s = clamp(index.s, 0.0, 1.0 - recipRes.x);
	index.t = clamp(index.t, 0.0, 1.0 - recipRes.y);

	// scale the texture so we just see the rendered framebuffer
	index.s = index.s * uScreenRes.x;
	index.t = index.t * uScreenRes.y;
	  
	vec3 RefractionColor = vec3( textureRect( uFrameBuffer, index ) );

	gl_FragColor = vec4( RefractionColor, 1.0 );
}