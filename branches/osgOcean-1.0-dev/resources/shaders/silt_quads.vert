uniform vec4 particleColour;

uniform float inversePeriod;
uniform float particleSize;
uniform float osg_SimulationTime;
uniform float osg_DeltaSimulationTime;

varying vec4 colour;
varying vec2 texCoord;

void main(void)
{
    float startTime = gl_MultiTexCoord1.x;
    texCoord = gl_MultiTexCoord0.xy;

	 float disp = (osg_SimulationTime - startTime)*inversePeriod;

    vec4 v_previous = gl_Vertex;
	
	 vec3 direction = sign(gl_Normal);

	 v_previous.x = direction.x * fract( disp + gl_Vertex.x );
	 v_previous.y = direction.y * fract( disp + gl_Vertex.y );
	 v_previous.z = direction.z * fract( disp + gl_Vertex.z );

    vec4 v_current =  v_previous;

	 v_current.x += ( osg_DeltaSimulationTime * inversePeriod );
	 v_current.y += ( osg_DeltaSimulationTime * inversePeriod );
	 v_current.z += ( osg_DeltaSimulationTime * inversePeriod );

    colour = particleColour;
    
    vec4 v1 = gl_ModelViewMatrix * v_current;
    vec4 v2 = gl_ModelViewMatrix * v_previous;
    
    vec3 dv = v2.xyz - v1.xyz;
    
    vec2 dv_normalized = normalize(dv.xy);
    dv.xy += dv_normalized * particleSize;
    vec2 dp = vec2( -dv_normalized.y, dv_normalized.x ) * particleSize;
    
    float area = length(dv.xy);
    colour.a = 0.05+(particleSize)/area;
    
    v1.xyz += dv*texCoord.y;
    v1.xy += dp*texCoord.x;

    gl_Position = gl_ProjectionMatrix * v1;
	 gl_Position.z = 0.01;
    gl_ClipVertex = v1;
};