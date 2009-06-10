uniform samplerRect uColorBuffer;
uniform samplerRect uStreakBuffer1;
uniform samplerRect uStreakBuffer2;
uniform samplerRect uStreakBuffer3;
uniform samplerRect uStreakBuffer4;

void main(void)
{
	vec4 fullColor    = textureRect(uColorBuffer,   gl_TexCoord[0].st );
	vec4 streakColor1 = textureRect(uStreakBuffer1, gl_TexCoord[1].st );
	vec4 streakColor2 = textureRect(uStreakBuffer2, gl_TexCoord[1].st );
	vec4 streakColor3 = textureRect(uStreakBuffer3, gl_TexCoord[1].st );
	vec4 streakColor4 = textureRect(uStreakBuffer4, gl_TexCoord[1].st );

	vec4 streak = streakColor1+streakColor2+streakColor3+streakColor4;
	
	gl_FragColor = vec4( streak.rgb+fullColor.rgb, 1.0);;
}