package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface PointShape3DReadOnly extends Shape3DReadOnly, Point3DReadOnly
{
   @Override
   default double getX()
   {
      return getPositionX();
   }

   @Override
   default double getY()
   {
      return getPositionY();
   }

   @Override
   default double getZ()
   {
      return getPositionZ();
   }

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
      return Shape3DReadOnly.super.containsNaN();
   }

   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      if (closestPointOnSurfaceToPack != null)
         closestPointOnSurfaceToPack.set(this);
      return false;
   }

   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      projectionToPack.set(this);
      return true;
   }

   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      return distanceSquared(query) <= epsilon * epsilon;
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
