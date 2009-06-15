uniform samplerRect osgOcean_ColorBuffer;
uniform samplerRect osgOcean_StreakBuffer1;
uniform samplerRect osgOcean_StreakBuffer2;
uniform samplerRect osgOcean_StreakBuffer3;
uniform samplerRect osgOcean_StreakBuffer4;

void main(void)
{
	vec4 fullColor    = textureRect(osgOcean_ColorBuffer,   gl_TexCoord[0].st );
	vec4 streakColor1 = textureRect(osgOcean_StreakBuffer1, gl_TexCoord[1].st );
	vec4 streakColor2 = textureRect(osgOcean_StreakBuffer2, gl_TexCoord[1].st );
	vec4 streakColor3 = textureRect(osgOcean_StreakBuffer3, gl_TexCoord[1].st );
	vec4 streakColor4 = textureRect(osgOcean_StreakBuffer4, gl_TexCoord[1].st );

	vec4 streak = streakColor1+streakColor2+streakColor3+streakColor4;

	gl_FragColor = vec4( streak.rgb+fullColor.rgb, 1.0);;
}