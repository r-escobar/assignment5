#include <cmath>
#include <float.h>
#include <algorithm>
#include <assert.h>
#include "trimesh.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

using namespace std;

Trimesh::~Trimesh()
{
	for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
		delete *i;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const Vec3d &v )
{
    vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
    materials.push_back( m );
}

void Trimesh::addNormal( const Vec3d &n )
{
    normals.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
    int vcnt = vertices.size();

    if( a >= vcnt || b >= vcnt || c >= vcnt ) return false;

    TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
    newFace->setTransform(this->transform);
    if (!newFace->degen) faces.push_back( newFace );


    // Don't add faces to the scene's object list so we can cull by bounding box
    // scene->add(newFace);
    return true;
}

char* Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
    if( !materials.empty() && materials.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of materials.";
    if( !normals.empty() && normals.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of normals.";

    return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	double tmin = 0.0;
	double tmax = 0.0;
	typedef Faces::const_iterator iter;
	bool have_one = false;

    if(kdtree) kdtree->intersect(r, i, have_one);
    else {
        for( iter j = faces.begin(); j != faces.end(); ++j ){
            isect cur;
            if( (*j)->intersectLocal( r, cur ) )
              {
                if( !have_one || (cur.t < i.t) )
                  {
                    i = cur;
                    have_one = true;
                  }
              }
      }
    }

	if( !have_one ) i.setT(1000.0);
	return have_one;
}

bool TrimeshFace::intersect(ray& r, isect& i) const {

  return intersectLocal(r, i);
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{



    const Vec3d& a = parent->vertices[ids[0]];
    const Vec3d& b = parent->vertices[ids[1]];
    const Vec3d& c = parent->vertices[ids[2]];

    // YOUR CODE HERE

    Vec3d bMinusA = b - a;
    Vec3d cMinusA = c - a;


    double d = -(a * normal);

    double t = -(r.p * normal + d) / (r.d * normal);

    if(t < RAY_EPSILON) {
        //std::cout << "RAY MISSES TRIANGLE!\n";

        return false;
    }

    Vec3d p = r.p + t * r.d;

    Vec3d pMinusA = p - a;
    Vec3d pMinusB = p - b;
    Vec3d pMinusC = p - c;
    Vec3d cMinusB = c - b;
    Vec3d aMinusC = a - c;

    double test1 = bMinusA.cross(pMinusA) * normal;
    double test2 = cMinusB.cross(pMinusB) * normal;
    double test3 = aMinusC.cross(pMinusC) * normal;


     if(test1 >= RAY_EPSILON && test2 >= RAY_EPSILON && test3 >= RAY_EPSILON) {


        i.t = t;
        i.setN(normal);


        double d00 = bMinusA * bMinusA;
        double d01 = bMinusA * cMinusA;
        double d11 = cMinusA * cMinusA;
        double d20 = pMinusA * bMinusA;
        double d21 = pMinusA * cMinusA;

        double denom = d00 * d11 - d01 * d01;

        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1.0f - v - w;

        i.uvCoordinates[0] = u;
        i.uvCoordinates[1] = v;

        i.setBary(Vec3d(u, v, w));
        i.setUVCoordinates(Vec2d(u, v));


        if(parent->vertNorms) {
            Vec3d aNormal = parent->normals[ids[0]];
            Vec3d bNormal = parent->normals[ids[1]];
            Vec3d cNormal = parent->normals[ids[2]];
            Vec3d newNormal = u * aNormal + v * bNormal + w * cNormal;
            newNormal.normalize();
            i.setN(newNormal);
        }

        if(!parent->materials.empty()) {
            Material aMat = *(parent->materials[ids[0]]);
            Material bMat = *(parent->materials[ids[1]]);
            Material cMat = *(parent->materials[ids[2]]);
            Material newMat = u * aMat;
            newMat += v * bMat;
            newMat += w * cMat;
            i.setMaterial(newMat);
        }




        i.setObject(this);

        //std::cout << "RAY INTERSECTS TRIANGLE!\n";
        return true;
     }

    //std::cout << "RAY MISSES TRIANGLE!\n";

    return false;
}

void Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
    int cnt = vertices.size();
    normals.resize( cnt );
    int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
    memset( numFaces, 0, sizeof(int)*cnt );
    
    for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
    {
		Vec3d faceNormal = (**fi).getNormal();
        
        for( int i = 0; i < 3; ++i )
        {
            normals[(**fi)[i]] += faceNormal;
            ++numFaces[(**fi)[i]];
        }
    }

    for( int i = 0; i < cnt; ++i )
    {
        if( numFaces[i] )
            normals[i]  /= numFaces[i];
    }

    delete [] numFaces;
    vertNorms = true;
}
