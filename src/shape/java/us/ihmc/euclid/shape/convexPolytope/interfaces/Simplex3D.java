package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Simplex3D
{
   /**
    * Return the shortest distance from the point to the simplex
    * 
    * @param point the spatial point from which the distance to the simplex is to be computed
    * @return shortest distance from the specified point to the simplex
    */
   double distance(Point3DReadOnly point);

   /**
    * Get a vector in the direction of the specified point from its closest point on the simplex
    * 
    * @param point the point that the vector should point towards
    * @param supportVectorToPack the vector in which the computed result is to be stored
    */
   default Vector3DBasics getSupportVectorDirectionTo(Point3DReadOnly point)
   {
      Vector3D supportVector = new Vector3D();
      getSupportVectorDirectionTo(point, supportVector);
      return supportVector;
   }

   /**
    * Get a vector in the direction of the specified point from its closest point on the simplex
    * 
    * @param point the point that the vector should point towards
    * @param supportVectorToPack the vector in which the computed result is to be stored
    */
   // TODO So sometimes it is normalized sometimes the support vector's length is the distance between the query and the closest point, not sure that is acceptable.
   void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack);

   /**
    * The smallest simplex member on which the projection of the specified point lies. Generally a
    * vertex, edge or face of the simplex
    * 
    * @param point the point for which the smallest simplex is needed
    * @return the smallest simplex for said point
    */
   Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point);
}
