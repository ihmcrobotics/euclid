package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface HalfEdge3DReadOnly extends LineSegment3DReadOnly
{
   /**
    * Returns a reference to the origin vertex for this half edge
    * 
    * @return a read only reference of the origin vertex for the half edge
    */
   Vertex3DReadOnly getOrigin();

   /**
    * Returns a reference to the destination vertex for this half edge
    * 
    * @return a read only reference of the destination vertex for the half edge
    */
   Vertex3DReadOnly getDestination();

   @Override
   default Point3DReadOnly getFirstEndpoint()
   {
      return getOrigin();
   }

   @Override
   default Point3DReadOnly getSecondEndpoint()
   {
      return getDestination();
   }

   /**
    * Returns a reference to the twin edge of this half edge
    * 
    * @return a read only reference to the twin half edge
    */
   HalfEdge3DReadOnly getTwin();

   /**
    * Returns a reference to the {@code nextHalfEdge} in the same {@code face} as this half edge
    * 
    * @return a read only reference to the next half edge
    */
   HalfEdge3DReadOnly getNext();

   /**
    * Returns a reference to the {@code previousHalfEdge} in the same {@code face} as this half edge
    * 
    * @return a read only reference to the previous half edge
    */
   HalfEdge3DReadOnly getPrevious();

   /**
    * Returns the reference to the face that this half edge is a part of
    * 
    * @return a read only reference to the face
    */
   Face3DReadOnly getFace();

   @Override
   default double distance(Point3DReadOnly point)
   {
      return LineSegment3DReadOnly.super.distance(point);
   }

   default double distanceFromSupportLine(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, getOrigin(), getDestination());
   }

   /**
    * Geometrically checks if the specified half edge is the twin of the current edge.
    * 
    * @param twinEdge the half edge that is to be checked
    * @param the precision required for the check
    * @return {@code true} if the twin edge of the specified half edge is not null and the twin edge's
    *         origin and destination are in an {@code epsilon} vicinity of this half edges origin and
    *         destination respectively
    */
   default boolean isTwin(HalfEdge3DReadOnly twin, double epsilon)
   {
      return epsilonEquals(twin.getTwin(), epsilon);
   }

   default boolean equals(HalfEdge3DReadOnly other)
   {
      if (other == this)
         return true;
      if (getOrigin() == null || getDestination() == null)
         return false;
      else
         return LineSegment3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(HalfEdge3DReadOnly other, double epsilon)
   {
      return LineSegment3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(HalfEdge3DReadOnly other, double epsilon)
   {
      return LineSegment3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Returns a string that indicates the spatial location of the object
    * 
    * @return string containing details
    */
   String toString();
}