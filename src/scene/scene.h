//
// scene.h
//
// The Scene class and the geometric types that it can contain.
//

#pragma warning (disable: 4786)


#ifndef __SCENE_H__
#define __SCENE_H__

#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include <memory>

#include "ray.h"
#include "material.h"
#include "camera.h"
#include "bbox.h"

#include "../vecmath/vec.h"
#include "../vecmath/mat.h"

class Light;
class Scene;

template <typename Obj>
class KdTree {
  public:
    BoundingBox bbox;
    KdTree<Obj>* left;
    KdTree<Obj>* right;
    std::vector<Obj*>* objectList;
    int splittingAxis;
    double splitAxisCoord;

    KdTree(std::vector<Obj*>& primitives, int currentDepth) : bbox(Vec3d(0.0, 0.0, 0.0), Vec3d(0.0, 0.0, 0.0)){
      left = NULL;
      right = NULL;
      objectList = NULL;
      splittingAxis = 0;
      splitAxisCoord = 0;

      int numObjs = primitives.size();

      // Get the combined bounding box of all primitives below this node
      if(numObjs > 0) {
        bbox = primitives[0]->getBoundingBox();
        for(int i = 1; i < numObjs; i++) {
          bbox.merge(primitives[i]->getBoundingBox());
        }
      }

      if(currentDepth >= 15 || numObjs <= 10) {
        // We've reached a leaf. Fill it's object list with the current primitive list
        objectList = new std::vector<Obj*>(primitives);
        return;
      }

      std::vector<Obj*> rightObjList;
      std::vector<Obj*> leftObjList;

      // Get the longest axis of the overall bounding box
      splittingAxis = bbox.longestAxis();

      // Get the center point of the overall bounding box
      splitAxisCoord = bbox.getCenterPoint()[splittingAxis];

      // Split up the triangles based on which side of the splittingAxis their centers are in
      for(int i = 0; i < primitives.size(); i++) {
        Vec3d currentCenter = primitives[i]->getBoundingBox().getCenterPoint();

        if(currentCenter[splittingAxis] >= splitAxisCoord) {
          rightObjList.push_back(primitives[i]);
        } else {
          leftObjList.push_back(primitives[i]);
        }
      }

      if(rightObjList.size() == 0 && leftObjList.size() > 0) rightObjList = leftObjList;
      if(leftObjList.size() == 0 && rightObjList.size() > 0) leftObjList = rightObjList;


      right = new KdTree<Obj>(rightObjList, currentDepth + 1);
      left = new KdTree<Obj>(leftObjList, currentDepth + 1);


    };


    void intersect(ray& r, isect& i, bool& have_one) {
      // Minimum and maximum times of intersection
      double tmin;
      double tmax;

      std::cout << "Checking for intersections (within KdTree)\n";

      // Check for intersection with this node's bbox 
      if(bbox.intersect(r, tmin, tmax)) {
        std::cout << "Detected a hit (within KdTree)\n";

        // If we aren't on a leaf, recurse down it's children
        if(!left && !right) {
          left->intersect(r, i, have_one);
          right->intersect(r, i, have_one);
        } else {
          // We're on a leaf

          isect currentIntersection;

          // Check for intersections with all triangles contained by this node
          for(int index = 0; index < objectList->size(); index++) {
            if((*objectList)[index]->intersect(r, currentIntersection)) {
              // Intersected with a triangle
              if(!have_one || (currentIntersection.t < i.t)) {
                i = currentIntersection;
                have_one = true;
              }
            }
          }
        }
      }
      return;
    }

};


class SceneElement {

public:
  virtual ~SceneElement() {}

  Scene *getScene() const { return scene; }

  // For debugging purposes, draws using OpenGL
  virtual void glDraw(int quality, bool actualMaterials, bool actualTextures) const  { }

 protected:
 SceneElement( Scene *s )
   : scene( s ) {}

  Scene *scene;
};

class TransformNode {

protected:

  // information about this node's transformation
  Mat4d    xform;
  Mat4d    inverse;
  Mat3d    normi;

  // information about parent & children
  TransformNode *parent;
  std::vector<TransformNode*> children;
    
 public:
  typedef std::vector<TransformNode*>::iterator          child_iter;
  typedef std::vector<TransformNode*>::const_iterator    child_citer;

  ~TransformNode() {
      for(child_iter c = children.begin(); c != children.end(); ++c ) delete (*c);
    }

  TransformNode *createChild(const Mat4d& xform) {
    TransformNode *child = new TransformNode(this, xform);
    children.push_back(child);
    return child;
  }
    
  // Coordinate-Space transformation
  Vec3d globalToLocalCoords(const Vec3d &v) { return inverse * v; }

  Vec3d localToGlobalCoords(const Vec3d &v) { return xform * v; }

  Vec4d localToGlobalCoords(const Vec4d &v) { return xform * v; }

  Vec3d localToGlobalCoordsNormal(const Vec3d &v) {
    Vec3d ret = normi * v;
    ret.normalize();
    return ret;
  }

  const Mat4d& transform() const		{ return xform; }

protected:
  // protected so that users can't directly construct one of these...
  // force them to use the createChild() method.  Note that they CAN
  // directly create a TransformRoot object.
 TransformNode(TransformNode *parent, const Mat4d& xform ) : children() {
      this->parent = parent;
      if (parent == NULL) this->xform = xform;
      else this->xform = parent->xform * xform;  
      inverse = this->xform.inverse();
      normi = this->xform.upper33().inverse().transpose();
    }
};

class TransformRoot : public TransformNode {
 public:
 TransformRoot() : TransformNode(NULL, Mat4d()) {}
};

// A Geometry object is anything that has extent in three dimensions.
// It may not be an actual visible scene object.  For example, hierarchical
// spatial subdivision could be expressed in terms of Geometry instances.
class Geometry : public SceneElement {

protected:
  // intersections performed in the object's local coordinate space
  // do not call directly - this should only be called by intersect()
  virtual bool intersectLocal(ray& r, isect& i ) const = 0;

public:
  // intersections performed in the global coordinate space.
  bool intersect(ray& r, isect& i) const;

  virtual bool hasBoundingBoxCapability() const;
  const BoundingBox& getBoundingBox() const { return bounds; }
  Vec3d getNormal() { return Vec3d(1.0, 0.0, 0.0); }

  virtual void ComputeBoundingBox() {
    // take the object's local bounding box, transform all 8 points on it,
    // and use those to find a new bounding box.

    BoundingBox localBounds = ComputeLocalBoundingBox();
        
    Vec3d min = localBounds.getMin();
    Vec3d max = localBounds.getMax();

    Vec4d v, newMax, newMin;

    v = transform->localToGlobalCoords( Vec4d(min[0], min[1], min[2], 1) );
    newMax = v;
    newMin = v;
    v = transform->localToGlobalCoords( Vec4d(max[0], min[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], max[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], max[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], min[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], min[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], max[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], max[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
		
    bounds.setMax(Vec3d(newMax));
    bounds.setMin(Vec3d(newMin));
  }

  // default method for ComputeLocalBoundingBox returns a bogus bounding box;
  // this should be overridden if hasBoundingBoxCapability() is true.
  virtual BoundingBox ComputeLocalBoundingBox() { return BoundingBox(); }

  virtual bool isTrimesh() { return false; }
  virtual void buildKdTree() {std::cout << "Attempting to build a KdTree. Do nothing\n";};

  void setTransform(TransformNode *transform) { this->transform = transform; };
    
 Geometry(Scene *scene) : SceneElement( scene ) {}

  // For debugging purposes, draws using OpenGL
  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

  // The defult does nothing; this is here because it is not required
  // that you implement this function if you create your own scene objects.
  virtual void glDrawLocal(int quality, bool actualMaterials, bool actualTextures) const { }

 protected:
  BoundingBox bounds;
  TransformNode *transform;
};

// A SceneObject is a real actual thing that we want to model in the 
// world.  It has extent (its Geometry heritage) and surface properties
// (its material binding).  The decision of how to store that material
// is left up to the subclass.
class SceneObject : public Geometry {

 public:
  virtual const Material& getMaterial() const = 0;
  virtual void setMaterial(Material *m) = 0;

  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

 protected:
 SceneObject( Scene *scene )
   : Geometry( scene ) {}
};

// A simple extension of SceneObject that adds an instance of Material
// for simple material bindings.
class MaterialSceneObject : public SceneObject {

public:
  virtual ~MaterialSceneObject() { delete material; }

  virtual const Material& getMaterial() const { return *material; }
  virtual void setMaterial(Material* m)	{ delete material; material = m; }

protected:
 MaterialSceneObject(Scene *scene, Material *mat) 
   : SceneObject(scene), material(mat) {}

  Material* material;
};

class Scene {

public:
  typedef std::vector<Light*>::iterator	liter;
  typedef std::vector<Light*>::const_iterator cliter;
  typedef std::vector<Geometry*>::iterator giter;
  typedef std::vector<Geometry*>::const_iterator cgiter;

  TransformRoot transformRoot;
  KdTree<Geometry>* kdtreeRoot;


  Scene() : transformRoot(), objects(), lights() {}
  virtual ~Scene();

  void add( Geometry* obj ) {
    obj->ComputeBoundingBox();
	sceneBounds.merge(obj->getBoundingBox());
    objects.push_back(obj);
  }
  void add(Light* light) { lights.push_back(light); }

  bool intersect(ray& r, isect& i) const;

  std::vector<Light*>::const_iterator beginLights() const { return lights.begin(); }
  std::vector<Light*>::const_iterator endLights() const { return lights.end(); }

  std::vector<Geometry*>::const_iterator beginObjects() const { return objects.begin(); }
  std::vector<Geometry*>::const_iterator endObjects() const { return objects.end(); }
        
  const Camera& getCamera() const { return camera; }
  Camera& getCamera() { return camera; }

  // For efficiency reasons, we'll store texture maps in a cache
  // in the Scene.  This makes sure they get deleted when the scene
  // is destroyed.
  TextureMap* getTexture( string name );

  // These two functions are for handling ambient light; in the Phong model,
  // the "ambient" light is considered a property of the _scene_ as a whole
  // and hence should be set here.
  Vec3d ambient() const	{ return ambientIntensity; }
  void addAmbient( const Vec3d& ambient ) { ambientIntensity += ambient; }

  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

  const BoundingBox& bounds() const { return sceneBounds; }

  void buildKdTree();

 private:
  std::vector<Geometry*> objects;
  std::vector<Geometry*> nonboundedobjects;
  std::vector<Geometry*> boundedobjects;
  std::vector<Light*> lights;
  Camera camera;

  // This is the total amount of ambient light in the scene
  // (used as the I_a in the Phong shading model)
  Vec3d ambientIntensity;

  typedef std::map< std::string, TextureMap* > tmap;
  tmap textureCache;
	
  // Each object in the scene, provided that it has hasBoundingBoxCapability(),
  // must fall within this bounding box.  Objects that don't have hasBoundingBoxCapability()
  // are exempt from this requirement.
  BoundingBox sceneBounds;
  

 public:
  // This is used for debugging purposes only.
  mutable std::vector<std::pair<ray*, isect*> > intersectCache;
};

#endif // __SCENE_H__
