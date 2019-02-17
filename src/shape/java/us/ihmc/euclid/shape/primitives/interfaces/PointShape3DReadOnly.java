package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PointShape3DReadOnly extends Shape3DReadOnly, Point3DReadOnly
{
   @Override
   default double distance(Point3DReadOnly point)
   {
      return Point3DReadOnly.super.distance(point);
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return distance(point);
   }

   @Override
   default boolean containsNaN()
   {
      return Point3DReadOnly.super.containsNaN();
   }

   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      closestPointOnSurfaceToPack.set(this);
      normalAtClosestPointToPack.sub(pointToCheck, this);
      normalAtClosestPointToPack.normalize();
      return false;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(this);
      return true;
   }

   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      projectionToPack.set(this);
      return true;
   }

   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      return distanceSquared(query) <= epsilon * epsilon;
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.set(this, this);
   }

   default boolean epsilonEquals(PointShape3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(PointShape3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
