package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Capsule3DReadOnly extends Shape3DReadOnly
{
   double getRadius();

   double getLength();

   default double getHalfLength()
   {
      return 0.5 * getLength();
   }

   default Vector3DReadOnly getAxis()
   {
      return getPose().getZAxis();
   }

   default Point3DReadOnly getTopCenter()
   {
      Point3D topCenter = new Point3D();
      topCenter.scaleAdd(getHalfLength(), getAxis(), getPosition());
      return topCenter;
   }

   default Point3DReadOnly getBottomCenter()
   {
      Point3D bottomCenter = new Point3D();
      bottomCenter.scaleAdd(-getHalfLength(), getAxis(), getPosition());
      return bottomCenter;
   }

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
      return EuclidShapeTools.doPoint3DCapsule3DCollisionTest(pointToCheck, getPosition(), getAxis(), getLength(), getRadius(), closestPointOnSurfaceToPack,
                                                              normalAtClosestPointToPack) <= 0.0;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      // TODO Should we consider the edge case where supportDirection is orthogonal to the cylinder's axis and return a vertex that is on the middle of the cylinder part?
      supportingVertexToPack.set(supportDirection);
      supportingVertexToPack.scale(getRadius() / supportDirection.length());

      if (supportDirection.dot(getAxis()) > 0.0)
         supportingVertexToPack.add(getTopCenter());
      else
         supportingVertexToPack.add(getBottomCenter());

      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndCapsule3D(point, getPosition(), getAxis(), getLength(), getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideCapsule3D(query, getPosition(), getAxis(), getLength(), getRadius(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidShapeTools.orthogonalProjectionOntoCapsule3D(pointToProject, getPosition(), getAxis(), getLength(), getRadius(), projectionToPack);
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
