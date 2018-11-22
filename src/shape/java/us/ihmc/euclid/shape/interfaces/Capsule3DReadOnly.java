package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Capsule3DReadOnly extends Shape3DReadOnly
{
   double getRadius();

   double getLength();

   Vector3DReadOnly getAxis();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || Double.isNaN(getLength()) || Double.isNaN(getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      return EuclidShapeTools.doPoint3DCapsule3DCollisionTest(getPosition(), getAxis(), getLength(), getRadius(), pointToCheck, closestPointOnSurfaceToPack,
                                                              normalAtClosestPointToPack) <= 0.0;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndCapsule3D(getPosition(), getAxis(), getLength(), getRadius(), point);
   }

   /** {@inheritDoc} */
   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideCapsule3D(getPosition(), getAxis(), getLength(), getRadius(), query, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidShapeTools.orthogonalProjectionOntoCapsule3D(getPosition(), getAxis(), getLength(), getRadius(), pointToProject, projectionToPack);
   }

   default boolean epsilonEquals(Capsule3DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getLength(), other.getLength(), epsilon) && EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon)
            && getPosition().epsilonEquals(other.getPosition(), epsilon) && other.getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   default boolean geometricallyEquals(Capsule3DReadOnly other, double epsilon)
   {
      if (Math.abs(getRadius() - other.getRadius()) > epsilon || Math.abs(getLength() - other.getLength()) > epsilon)
         return false;

      if (!getPosition().geometricallyEquals(other.getPosition(), epsilon))
         return false;

      return EuclidGeometryTools.areVector3DsParallel(getAxis(), other.getAxis(), epsilon);
   }
}
