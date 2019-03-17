package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface HalfEdge3DReadOnly extends LineSegment3DReadOnly, ConvexPolytopeFeature3D
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
   default List<? extends Vertex3DReadOnly> getVertices()
   {
      return Arrays.asList(getOrigin(), getDestination());
   }

   @Override
   default double distance(Point3DReadOnly point)
   {
      return LineSegment3DReadOnly.super.distance(point);
   }

   default double distanceFromSupportLine(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, getOrigin(), getDestination());
   }

   @Override
   default ConvexPolytopeFeature3D getSmallestFeature(Point3DReadOnly point)
   {
      double percentage = percentageAlongLineSegment(point);
      if (percentage <= 0.0)
         return getOrigin();
      else if (percentage >= 1.0)
         return getDestination();
      else
         return this;
   }

   @Override
   default boolean getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      double alpha = percentageAlongLineSegment(point);

      if (alpha >= 1.0)
      {
         supportVectorToPack.sub(point, getDestination());
      }
      else if (alpha <= 0.0)
      {
         supportVectorToPack.sub(point, getOrigin());
      }
      else
      {
         supportVectorToPack.interpolate(getOrigin(), getDestination(), alpha);
         supportVectorToPack.sub(point, supportVectorToPack);
      }

      return true;
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