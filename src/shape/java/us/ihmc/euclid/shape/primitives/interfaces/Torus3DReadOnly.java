package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Torus3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the radius from the torus center to the tube center.
    *
    * @return this torus main radius.
    */
   double getRadius();

   /**
    * Gets the radius of the tube of this torus.
    *
    * @return the radius of the tube.
    */
   double getTubeRadius();

   /**
    * Gets the read-only reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   Point3DReadOnly getPosition();

   Vector3DReadOnly getAxis();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getAxis().containsNaN() || Double.isNaN(getRadius()) || Double.isNaN(getTubeRadius());
   }

   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      return EuclidShapeTools.doPoint3DTorus3DCollisionTest(pointToCheck, getPosition(), getAxis(), getRadius(), getTubeRadius(), closestPointOnSurfaceToPack,
                                                            normalAtClosestPointToPack) <= 0.0;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      throw new UnsupportedOperationException("Torus3D being a concave shape cannot implement the supporting vertex feature.");
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndTorus3D(point, getPosition(), getAxis(), getRadius(), getTubeRadius());
   }

   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideTorus3D(query, getPosition(), getAxis(), getRadius(), getTubeRadius(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidShapeTools.orthogonalProjectionOntoTorus3D(pointToProject, getPosition(), getAxis(), getRadius(), getTubeRadius(), projectionToPack);
   }

   /**
    * Tests separately and on a per component basis if the pose and the radii of this torus and
    * {@code other}'s pose and radii are equal to an {@code epsilon}.
    *
    * @param other the other torus which pose and radii is to be compared against this torus pose and
    *           radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two tori are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Torus3DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon)
            && EuclidCoreTools.epsilonEquals(getTubeRadius(), other.getTubeRadius(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon)
            && getAxis().epsilonEquals(other.getAxis(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two tori are geometrically similar.
    * <p>
    * This method accounts for the multiple combinations of radii and rotations that generate identical
    * tori. For instance, two tori that are identical but one is rotated around its main axis are
    * considered geometrically equal.
    * </p>
    *
    * @param other the torus to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two tori represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Torus3DReadOnly other, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getTubeRadius(), other.getTubeRadius(), epsilon))
         return false;

      if (!getPosition().geometricallyEquals(getPosition(), epsilon))
         return false;

      return EuclidGeometryTools.areVector3DsParallel(getAxis(), other.getAxis(), epsilon);
   }
}
