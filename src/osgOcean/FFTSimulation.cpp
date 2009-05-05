#include <osgOcean/FFTSimulation>

using namespace osgOcean;

FFTSimulation::FFTSimulation( int gridsize,
										const osg::Vec2f& winddir,
										float windspeed,
										float waveheight,
										float tilesize,
										float cycletime ):
	_PI2			( 2.0*osg::PI ),
	_GRAVITY		( 9.81 ),
	N				( gridsize ), 
	W				( winddir ), 
	v				( windspeed ), 
	a				( waveheight ),
	Lm				( tilesize ),
	w0				( _PI2 / cycletime )

{
	_h0tilde.resize( (N+1)*(N+1) );

	compute_h0tilde();

	_htilde.resize( N*(N/2+1) );

	fft_in  = new fftwf_complex[ N*(N/2+1) ];
	fft_in2 = new fftwf_complex[ N*(N/2+1) ];

	fft_out  = new float[ N*N ];
	fft_out2 = new float[ N*N ];

	plan  = fftwf_plan_dft_c2r_2d(N, N, fft_in,  fft_out,  0);
	plan2 = fftwf_plan_dft_c2r_2d(N, N, fft_in2, fft_out2, 0);
}

FFTSimulation::~FFTSimulation()
{
	fftwf_destroy_plan(plan);
	fftwf_destroy_plan(plan2);

	delete[] fft_in;
	delete[] fft_in2;

	delete[] fft_out;
	delete[] fft_out2;
}

float FFTSimulation::myrnd()
{
	return (float)rand()/RAND_MAX;
}

complex FFTSimulation::gaussrand()
{
	float x1, x2, w;

	do 
	{
		x1 = 2.f * myrnd() - 1.f;
		x2 = 2.f * myrnd() - 1.f;
		w = x1 * x1 + x2 * x2;
	} 
	while ( w >= 1.f );

	w = sqrt( (-2.f * log( w ) ) / w );

	return complex(x1 * w, x2 * w);
}

float FFTSimulation::phillips(const osg::Vec2f& K) const
{
	float k2 = K.length2();

	if (k2 == 0.f) 
		return 0.f;

	float v2 = v * v;

	float v4 = v2 * v2;

	float k4 = k2 * k2;

	float g2 = _GRAVITY * _GRAVITY;

	float KdotW = K.x() * W.x() + K.y() * W.y(); //K * W;

	float KdotWhat = KdotW*KdotW/k2;

	float eterm = exp(-g2 / (k2 * v4)) / k4;

	float dampfac = 1.f/100.f;

	float l2 = v4/g2 * dampfac*dampfac;	// damping of very small waves

	float result = a * eterm * KdotWhat * exp(-k2*l2);	

	if (KdotW < 0.f)	// filter out waves moving against the wind
		result *= 0.25f;

	return result;
}

complex FFTSimulation::h0_tilde( const osg::Vec2f& K ) const
{
	// f = sin(2*M_PI*(rand())/RAND_MAX);
	complex g = gaussrand();	

	float p = sqrt( 0.5f * phillips(K) );

	return g * p;	// * f;
}

void FFTSimulation::compute_h0tilde()
{
	for (int y = 0, y2 = -N/2; y <= N; ++y, ++y2) 
	{
		float Ky = _PI2*y2/Lm;

		for (int x = 0, x2 = -N/2; x <= N; ++x, ++x2) 
		{
			float Kx = _PI2*x2/Lm;

			_h0tilde[y*(N+1)+x] = h0_tilde(osg::Vec2f(Kx, Ky));
		}
	}
}


complex FFTSimulation::h_tilde(const osg::Vec2f& K, int kx, int ky, float time) const
{
	complex h0_tildeK = _h0tilde[ky*(N+1)+kx];
	complex h0_tildemKconj = conj(_h0tilde[(N-ky)*(N+1)+(N-kx)]);

	float wK = sqrt( _GRAVITY * K.length() );
	float wK2 = floor(wK/w0)*w0;
	float xp = wK2 * time;
	float cxp = cos(xp);
	float sxp = sin(xp);

	return h0_tildeK * complex(cxp, sxp) + h0_tildemKconj * complex(cxp, -sxp);
}

void FFTSimulation::compute_htilde(float time)
{
	for (int y = 0; y <= N/2; ++y) 
	{
		for (int x = 0; x < N; ++x) 
		{
			osg::Vec2f K( _PI2*(x-N/2)/Lm, _PI2*(y-N/2)/Lm );
			_htilde[y*N+x] = h_tilde(K, x, y, time);
		}
	}
}

void FFTSimulation::set_time(float time)
{
	compute_htilde( time );
}

void FFTSimulation::compute_heights( osg::FloatArray* waveheights ) const
{
	for (int y = 0; y <= N/2; ++y) 
	{
		for (int x = 0; x < N; ++x) 
		{
			const complex& c = _htilde[y*N+x];

			int ptr = x*(N/2+1)+y;

			fft_in[ptr][0] = c.real();
			fft_in[ptr][1] = c.imag();
		}
	}

	fftwf_execute(plan);

	if (waveheights->size() != (unsigned int)(N*N) )
		waveheights->resize(N*N);

	const float signs[2] = { 1.f, -1.f };

	for (int y = 0; y < N; ++y)
	{
		for (int x = 0; x < N; ++x)
		{
			waveheights->at(y*N+x) = fft_out[y*N+x] * signs[(x + y) & 1];
		}
	}
}

void FFTSimulation::compute_displacements(const float& scalefac, osg::Vec2Array* wavedisplacements) const
{
	for (int y = 0; y <= N/2; ++y) 
	{
		for (int x = 0; x < N; ++x) 
		{
			const complex& c = _htilde[y*N+x];

			osg::Vec2f K( (float)(x-N/2), (float)(y-N/2) );	

			float k = K.length();

			osg::Vec2f Kh;

			if (k != 0)
				Kh = K * (1.f/k);
			else
				Kh.x() = Kh.y() = 0.f;

			int ptr = x*(N/2+1)+y;

			fft_in[ptr][0] =  c.imag() * Kh.x();
			fft_in[ptr][1] = -c.real() * Kh.x();
			fft_in2[ptr][0] =  c.imag() * Kh.y();
			fft_in2[ptr][1] = -c.real() * Kh.y();
		}
	}

	fftwf_execute(plan);
	fftwf_execute(plan2);

	if (wavedisplacements->size() != (unsigned int)(N*N) )
		wavedisplacements->resize(N*N);

	float signs[2] = { 1.f, -1.f };

	unsigned ptr = 0;

	for (int y = 0; y < N; ++y)
	{
		for (int x = 0; x < N; ++x) 
		{
			float s = signs[(x + y) & 1];
			wavedisplacements->at(ptr) = osg::Vec2f(fft_out[ptr], fft_out2[ptr]) * s * scalefac;
			++ptr;
		}
	}
}
