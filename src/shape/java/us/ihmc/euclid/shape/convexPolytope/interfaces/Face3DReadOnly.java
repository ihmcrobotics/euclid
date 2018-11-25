package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.List;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Face3DReadOnly extends Clearable
{
   /**
    * Gets a list of all half edges that constitute the face
    * 
    * @return a list of read only references to the half edges
    */
   List<? extends HalfEdge3DReadOnly> getEdgeList();

   /**
    * Gets a particular half edge that is part of the face
    * 
    * @param index the index of the half edge that is required. Should be less than value returned by
    *           {@code getNumberOfEdges()}
    * @return
    */
   HalfEdge3DReadOnly getEdge(int index);

   /**
    * Returns the first edge that is visible from the specified point.
    * <p>
    * A face edge is considered visible if its origin and destination can be connected by a straight
    * line to the specified point without crossing any other point First is defined in the counter
    * clockwise sense w.r.t to the face normal.
    * </p>
    * 
    * @param point the point in the plane of the face w.r.t. which the visible edge is to be computed
    * @return a read only reference to the first visible half edge
    */
   HalfEdge3DReadOnly getFirstVisibleEdge(Point3DReadOnly point);

   /**
    * Checks if a point is strictly on the same side of the specified edge as the face centroid, i.e.
    * the left hand side w.r.t. to the edge
    * 
    * @param point the point to be checked
    * @param index index of the edge w.r.t. which the point is to be evaluated
    * @return
    */
   boolean isPointOnInteriorSideOfEdgeInternal(Point3DBasics point, int index);

   /**
    * Returns a dot product of the vector from a point on the face to the specified point and the face
    * normal A positive value indicates that the face is visible from the point specified
    * 
    * @param point the point from which visibility of the face is to be tested
    * @return dot product result from the computation. Represents the distance of the face plane from
    *         the point
    */
   double getFaceVisibilityProduct(Point3DReadOnly point);

   /**
    * Checks if a particular point is in the same spatial plane as this face
    * 
    * @param pointToCheck the point for which the check is to be performed
    * @param epsilon the error value that is acceptable (units should be distance)
    * @return {@code true} if the point is in the same plane as the face. {@code false} otherwise
    */
   boolean isPointInFacePlane(Point3DReadOnly pointToCheck, double epsilon);

   /**
    * Checks if a particular point is an interior point to the face. For a point to be an interior
    * point it must lie in the plane of the face and be strictly within the face edges
    * 
    * @param vertexToCheck the point on which the check is to be performed
    * @param epsilon the precision level for this check operation
    * @return {@code true} if the point is an interior point. {@code false} otherwise
    */
   boolean isInteriorPoint(Point3DReadOnly vertexToCheck, double epsilon);

   /**
    * Gets the centroid of the face
    * 
    * @return read only point object indicating the spatial location of the face centroid
    */
   Point3DReadOnly getFaceCentroid();

   /**
    * Gets a vector normal to the face. If the face is part of a polytope, the vector will point away
    * from the centroid of the polytope. If the face is not part of a polytope, the direction of the
    * vector is arbitrary
    * 
    * @return a read only vector normal to the face
    */
   Vector3DReadOnly getFaceNormal();

   /**
    * Gets the number of edges that the face is made up of
    * 
    * @return
    */
   int getNumberOfEdges();

   /**
    * Returns the dot product of the specified vector with the face normal
    * 
    * @param vector the vector for which the dot product is required
    * @return the resultant dot product
    */
   double dotFaceNormal(Vector3DReadOnly vector);

   /**
    * Check if the face is visible from the specified point
    * 
    * @param point the point from which the visibility of the face is to be established
    * @param epsilon the precision of this check
    * @return {@code true} if the face is visible from the specified point, {@code false} otherwise
    */
   boolean isFaceVisible(Point3DReadOnly point, double epsilon);

   /**
    * Returns the largest value of the face along the specified axis (0 - X axis, 1 - Y axis, 2 - Z
    * axis) The axes are in the reference frame in which the face exists
    * 
    * @param index the ordinal of the axis
    * @return the largest value of the coordinate along the specified axis
    */
   double getMaxElement(int index);

   /**
    * Returns the smallest value of the face along the specified axis (0 - X axis, 1 - Y axis, 2 - Z
    * axis) The axes are in the reference frame in which the face exists
    * 
    * @param index the ordinal of the axis
    * @return the smallest value of the coordinate along the specified axis
    */
   double getMinElement(int index);

   /**
    * Returns the largest X - coordinate value of all points on the face
    * 
    * @return the largest X coordinate value
    */
   double getMaxX();

   /**
    * Returns the largest Y - coordinate value of all points on the face
    * 
    * @return the largest Y coordinate value
    */
   double getMaxY();

   /**
    * Returns the largest Z - coordinate value of all points on the face
    * 
    * @return the largest Z coordinate value
    */
   double getMaxZ();

   /**
    * Returns the smallest X - coordinate value of all points on the face
    * 
    * @return the smallest X coordinate value
    */
   double getMinX();

   /**
    * Returns the smallest Y - coordinate value of all points on the face
    * 
    * @return the smallest Y coordinate value
    */
   double getMinY();

   /**
    * Returns the smallest Z - coordinate value of all points on the face
    * 
    * @return the smallest Z coordinate value
    */
   double getMinZ();

   /**
    * Return a reference to an adjacent face if this face is part of a polytope. If not can return a
    * {@code null} or throw a {@code NullPointerException}
    * 
    * @param index the index of the adjacent face that is required. Should be less than
    *           {@code getNumberOfEdges()}
    * @return a read only reference to the adjacent face
    */
   Face3DReadOnly getNeighbouringFace(int index);

   /**
    * Check if the face is marked
    * 
    * @return {@code true} if marked, else {@code false}
    */
   boolean isMarked();

   /**
    * Check if the face is not marked
    * 
    * @return {@code true} if not marked, else {@code false}
    */
   boolean isNotMarked();

   /**
    * Get a printable version of the face data
    * 
    * @return a string indicating face edges, vertices and their associations
    */
   String toString();

   /**
    * Returns the shortest distance to the point specified
    * 
    * @param point the point to which the distance is required
    * @return the shortest length from the specified point to the face
    */
   double distance(Point3DReadOnly point);

   /**
    * Returns the edge closed to the point specified
    * 
    * @param point the point to which the closed edge is required
    * @return read only reference to the half edge that is closed to the specified point
    */
   HalfEdge3DReadOnly getEdgeClosestTo(Point3DReadOnly point);

   default boolean equals(Face3DReadOnly other)
   {
      if (other == null)
         return false;
      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         if (!getEdge(edgeIndex).equals(other.getEdge(edgeIndex)))
            return false;
      }
      return true;
   }

   default boolean epsilonEquals(Face3DReadOnly other, double epsilon)
   {
      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         if (!getEdge(edgeIndex).epsilonEquals(other.getEdge(edgeIndex), epsilon))
            return false;
      }
      return true;
   }

   /**
    * Geometrically evaluates if the face is within an epsilon vicinity of the specified face. This
    * check is actually performed on the vertices that the face is composed of. Both spatial and and
    * relations between vertices are checked
    * 
    * @return {@code true} is the face is geometrically similar to the specified face, {@code false}
    *         otherwise
    */
   default boolean geometricallyEquals(Face3DReadOnly other, double epsilon)
   {
      if (getNumberOfEdges() != other.getNumberOfEdges())
         return false;

      HalfEdge3DReadOnly startEdge = null;
      HalfEdge3DReadOnly otherCurrentEdge = other.getEdge(0);
      HalfEdge3DReadOnly thisCurrentEdge = null;

      for (int edgeIndex = 0; edgeIndex < getNumberOfEdges(); edgeIndex++)
      {
         thisCurrentEdge = getEdge(edgeIndex);

         if (!thisCurrentEdge.geometricallyEquals(otherCurrentEdge, epsilon))
         {
            startEdge = thisCurrentEdge;
            break;
         }
      }

      if (startEdge == null)
         return false;

      do
      {
         otherCurrentEdge.getNextHalfEdge();
         thisCurrentEdge.getNextHalfEdge();
         if (!thisCurrentEdge.geometricallyEquals(otherCurrentEdge, epsilon))
            return false;
      }
      while (thisCurrentEdge != startEdge);

      return true;
   }
}