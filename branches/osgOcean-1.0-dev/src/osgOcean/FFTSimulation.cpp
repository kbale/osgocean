/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <osgOcean/FFTSimulation>

using namespace osgOcean;

FFTSimulation::FFTSimulation( int fourierSize,
										const osg::Vec2f& windDir,
										float windSpeed,
										float waveScale,
										float tileRes,
										float loopTime ):
	_PI2        ( 2.0*osg::PI ),
	_GRAVITY    ( 9.81 ),
	_N          ( fourierSize ), 
	_nOver2     ( fourierSize/2 ),
	_windDir    ( windDir ), 
	_windSpeed  ( windSpeed ), 
	_A				( float(_N)*waveScale ),
	_length		( tileRes ),
	_w0         ( _PI2 / loopTime )
{
	_baseAmplitudes.resize( (_N+1)*(_N+1) );

	computeBaseAmplitudes();
	
	_curAmplitudes.resize( _N*_N );

	_complexData0 = new fftw_complex[ _N*_N ];
	_complexData1 = new fftw_complex[ _N*_N ];

	_realData0 = new fftw_complex[ _N*_N ];
	_realData1 = new fftw_complex[ _N*_N ];

	_fftPlan0 = fftw_plan_dft_2d( _N, _N, _complexData0, _realData0, FFTW_BACKWARD, FFTW_ESTIMATE );
	_fftPlan1 = fftw_plan_dft_2d( _N, _N, _complexData1, _realData1, FFTW_BACKWARD, FFTW_ESTIMATE );
}

FFTSimulation::~FFTSimulation()
{
	fftw_destroy_plan(_fftPlan0);
	fftw_destroy_plan(_fftPlan1);

	delete[] _complexData0;
	delete[] _complexData1;

	delete[] _realData0;
	delete[] _realData1;
}

float FFTSimulation::unitRand()
{
	return (float)rand()/RAND_MAX;
}

complex FFTSimulation::gaussianRand()
{
	float x1, x2, w;

	do {
		x1 = 2.f * unitRand() - 1.f;
		x2 = 2.f * unitRand() - 1.f;
		w = x1 * x1 + x2 * x2;
	} 
	while ( w >= 1.f );

	w = sqrt( (-2.f * log( w ) ) / w );

	return complex(x1 * w, x2 * w);
}

float FFTSimulation::phillipsSpectrum(const osg::Vec2f& K) const
{
	float k2 = K.length2();

	if (k2 == 0.f) 
		return 0.f;

	float v2 = _windSpeed * _windSpeed;

	float v4 = v2 * v2;

	float k4 = k2 * k2;

	float g2 = _GRAVITY * _GRAVITY;

	float KdotW = K.x() * _windDir.x() + K.y() * _windDir.y(); 

	float KdotWhat = KdotW*KdotW/k2;

	float eterm = exp(-g2 / (k2 * v4)) / k4;

	float damping = 0.001f;

	float l2 = v4/g2 * damping*damping;	

	float result = _A * eterm * KdotWhat * exp(-k2*l2);	

	if (KdotW < 0.f)	
		result *= 0.25f;

	return result;
}

complex FFTSimulation::h0Tilde( const osg::Vec2f& K ) const
{
	complex g = gaussianRand();	

	double p = sqrt( 0.5 * phillipsSpectrum(K) );

	return g * p;
}

void FFTSimulation::computeBaseAmplitudes()
{
	for (int y = 0, y2 = -_nOver2; y <= _N; ++y, ++y2) 
	{
		float Ky = _PI2*y2/_length;

		for (int x = 0, x2 = -_nOver2; x <= _N; ++x, ++x2) 
		{
			float Kx = _PI2*x2/_length;

			_baseAmplitudes[y*(_N+1)+x] = h0Tilde(osg::Vec2f(Kx, Ky));
		}
	}
}

complex FFTSimulation::hTilde(const osg::Vec2f& K, int kx, int ky, float time) const
{
	complex h0_tildeK = _baseAmplitudes[ ky*(_N+1)+kx ];
	complex h0_tildemKconj = conj( _baseAmplitudes[ (_N-ky)*(_N+1)+(_N-kx) ] );

	float wK = sqrt( _GRAVITY * K.length() );
	float wK2 = floor(wK/_w0)*_w0;
	float xp = wK2 * time;
	float cxp = cos(xp);
	float sxp = sin(xp);

	return h0_tildeK * complex(cxp, sxp) + h0_tildemKconj * complex(cxp, -sxp);
}

void FFTSimulation::computeCurrentAmplitudes(float time)
{
	float oneOverLen = 1.f/(float)_length;
	osg::Vec2f K;

	for (int y = 0; y < _N; ++y) 
	{
		for (int x = 0; x < _N; ++x) 
		{
			K.x() = _PI2 * ( float(x-_nOver2) * oneOverLen );
			K.y() = _PI2 * ( float(y-_nOver2) * oneOverLen );

			_curAmplitudes[y*_N+x] = hTilde(K, x, y, time);
		}
	}
}

void FFTSimulation::setTime(float time)
{
	computeCurrentAmplitudes(time);
}

void FFTSimulation::computeHeights( osg::FloatArray* waveheights ) const
{
	// populate input array
	for(int y = 0; y < _N; ++y )
	{
		for (int x = 0; x < _N; ++x) 
		{
			int ptr = y*_N+x;

			const complex& c = _curAmplitudes[ptr];

			_complexData0[ptr][0] = c.real();
			_complexData0[ptr][1] = c.imag();
		}
	}

	fftw_execute(_fftPlan0);

	if (waveheights->size() != (unsigned int)(_N*_N) ){
		waveheights->resize(_N*_N);
	}

	const float signs[2] = { 1.f, -1.f };

	for(int y = 0; y < _N; ++y)
	{
		for(int x = 0; x < _N; ++x )
		{
			waveheights->at(y*_N+x) = _realData0[x*_N+y][0]  * signs[(x + y) & 1];
		}
	}
}

void FFTSimulation::computeDisplacements(const float& scaleFactor, osg::Vec2Array* waveDisplacements) const
{
	for (int y = 0; y < _N; ++y) 
	{
		for (int x = 0; x < _N; ++x) 
		{
			const complex& c = _curAmplitudes[y*_N+x];

			osg::Vec2f K( (float)(x-_nOver2), (float)(y-_nOver2) );	

			float k = K.length();

			osg::Vec2f Kh;

			if (k != 0)
				Kh = K * (1.f/k);
			else
				Kh.x() = Kh.y() = 0.f;

			int ptr = x*_N+y;

			_complexData0[ptr][0] =  c.imag() * Kh.x();
			_complexData0[ptr][1] = -c.real() * Kh.x();
			
			_complexData1[ptr][0] =  c.imag() * Kh.y();
			_complexData1[ptr][1] = -c.real() * Kh.y();
		}
	}

	fftw_execute(_fftPlan0);
	fftw_execute(_fftPlan1);

	if (waveDisplacements->size() != (unsigned int)(_N*_N) )
		waveDisplacements->resize(_N*_N);

	float signs[2] = { 1.f, -1.f };

	unsigned int ptr = 0;

	for (int y = 0; y < _N; ++y)
	{
		for (int x = 0; x < _N; ++x) 
		{
			ptr = x*_N+y;

			double s = signs[(x + y) & 1];
			waveDisplacements->at(y*_N+x) = osg::Vec2(_realData0[ptr][0], _realData1[ptr][0]) * s * (double)scaleFactor;
		}
	}
}

