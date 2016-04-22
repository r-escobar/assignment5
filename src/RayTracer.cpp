// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>

extern TraceUI* traceUI;

#include <iostream>
#include <fstream>

using namespace std;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = false;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

Vec3d RayTracer::trace(double x, double y)
{
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) scene->intersectCache.clear();
  ray r(Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY);
  scene->getCamera().rayThrough(x,y,r);
  Vec3d ret = traceRay(r, traceUI->getDepth());
  ret.clamp();
  return ret;
}

Vec3d RayTracer::tracePixel(int i, int j)
{
	Vec3d col(0,0,0);

	if( ! sceneLoaded() ) return col;

	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	col = trace(x, y);

	double subPixWidth = 1.0 / (double(buffer_width) * traceUI->m_nSamples);
	double subPixHeight = 1.0 / (double(buffer_height) * traceUI->m_nSamples);

	double testX = x + (traceUI->m_nSamples - 1) * subPixWidth;
	double testY = y + (traceUI->m_nSamples - 1) * subPixHeight;

	Vec3d testColor1 = trace(x, y);
	Vec3d testColor2 = trace(testX, testY);
	Vec3d testColor3 = trace(x, testY);
	Vec3d testColor4 = trace(testX, y);

	double testVal1 = (abs(testColor1[0] - testColor2[0]) + abs(testColor1[2] - testColor2[2]) + abs(testColor1[2] - testColor2[2])) / 3.0; 
	double testVal2 = (abs(testColor3[0] - testColor4[0]) + abs(testColor3[2] - testColor4[2]) + abs(testColor3[2] - testColor4[2])) / 3.0; 
	double testVal3 = (abs(testColor1[0] - testColor3[0]) + abs(testColor1[2] - testColor3[2]) + abs(testColor1[2] - testColor3[2])) / 3.0; 
	double testVal4 = (abs(testColor1[0] - testColor4[0]) + abs(testColor1[2] - testColor4[2]) + abs(testColor1[2] - testColor4[2])) / 3.0; 
	double testVal5 = (abs(testColor3[0] - testColor2[0]) + abs(testColor3[2] - testColor2[2]) + abs(testColor3[2] - testColor2[2])) / 3.0; 
	double testVal6 = (abs(testColor2[0] - testColor4[0]) + abs(testColor2[2] - testColor4[2]) + abs(testColor2[2] - testColor4[2])) / 3.0; 


	double sampleThresh = traceUI->m_sampleThreshold / 1000.0;

	if(testVal1 > sampleThresh || testVal2 > sampleThresh || testVal3 > sampleThresh || testVal4 > sampleThresh || testVal5 > sampleThresh || testVal6 > sampleThresh) {
		for(int i = 0; i < traceUI->m_nSamples; i++) {
			for(int j = 0; j < traceUI->m_nSamples; j++) {
				double newX = x + i * subPixWidth;
				double newY = y + j * subPixHeight;
				Vec3d newCol = trace(newX, newY);
				col += newCol;			
			}
		}

		col /= double(pow(traceUI->m_nSamples, 2));
	}


	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);

	return col;
}


// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
Vec3d RayTracer::traceRay(ray& r, int depth)
{
	isect i;
	Vec3d colorC;

	if(scene->intersect(r, i)) {
		// YOUR CODE HERE

		// An intersection occurred!  We've got work to do.  For now,
		// this code gets the material for the surface that was intersected,
		// and asks that material to provide a color for the ray.  	

		// This is a great place to insert code for recursive ray tracing.
		// Instead of just returning the result of shade(), add some
		// more steps: add in the contributions from reflected and refracted
		// rays.

		//std::cout << "INTERSECTED WITH SOMETHING!\n";


	  


	  const Material& m = i.getMaterial();
	  Vec3d color = m.shade(scene, r, i);

	  if(depth == 0) {
	  	return color;
	  }




	  /* REFLECTION */
	  if(m.Refl()) {
	  	Vec3d reflectDir = 2 * (-r.d * i.N) * i.N + r.d;
	  	reflectDir.normalize();

	  	ray reflectedRay(r.at(i.t), reflectDir, ray::REFLECTION);


	  	color += prod(m.kr(i), traceRay(reflectedRay, depth - 1));
	  }

	  /* REFRACTION*/ 

	  if(m.Trans()) {
  		  double ni, nt;
		  Vec3d transmittedDir;

		  int direction;

	  	  double cosI = -r.d * i.N;

		  if(cosI > 0.0) {
		  	ni = 1.0;
		  	nt = m.index(i);
		  	direction = 1;
		  } else {
		  	ni = m.index(i);
		  	nt = 1.0;
		  	direction = -1;
		  }

		  double nCoeff = ni/nt;

		  Vec3d adjustedNormal = direction * i.N;


		  double TIRtest = (nCoeff * nCoeff) * (1 - (cosI * cosI));
		  Vec3d xProj = nCoeff * (i.N * (-r.d * i.N) + r.d);


		  if(TIRtest <= 1.0) {
		  	transmittedDir = (direction * nCoeff * r.d) + nCoeff * adjustedNormal * (-r.d * i.N) - i.N * sqrt(1 - xProj * xProj);

		  	transmittedDir.normalize();

		  	ray transmittedRay(r.at(i.t), direction * transmittedDir, ray::REFRACTION);

		  	color += prod(m.kt(i), traceRay(transmittedRay, depth - 1));
		  }
	  }

	  colorC = color;


	} else {
		// No intersection.  This ray travels to infinity, so we color
		// it according to the background color, which in this (simple) case
		// is just black.
		colorC = Vec3d(0.0, 0.0, 0.0);
	}
	return colorC;
}

RayTracer::RayTracer()
	: scene(0), buffer(0), buffer_width(256), buffer_height(256), m_bBufferReady(false)
{}

RayTracer::~RayTracer()
{
	delete scene;
	delete [] buffer;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene( char* fn ) {
	ifstream ifs( fn );
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}
	
	// Strip off filename, leaving only the path:
	string path( fn );
	if( path.find_last_of( "\\/" ) == string::npos ) path = ".";
	else path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
    Parser parser( tokenizer, path );
	try {
		delete scene;
		scene = 0;
		scene = parser.parseScene();
	} 
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	}
	catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	}
	catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}

	if( !sceneLoaded() ) return false;

	scene->buildKdTree();

	return true;
}

void RayTracer::traceSetup(int w, int h)
{
	if (buffer_width != w || buffer_height != h)
	{
		buffer_width = w;
		buffer_height = h;
		bufferSize = buffer_width * buffer_height * 3;
		delete[] buffer;
		buffer = new unsigned char[bufferSize];
	}
	memset(buffer, 0, w*h*3);
	m_bBufferReady = true;
}

