package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

public interface Cylinder3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the radius of this cylinder.
    *
    * @return the value of the radius.
    */
   double getRadius();

   /**
    * Gets the height of this cylinder.
    *
    * @return the value of the height.
    */
   double getHeight();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || Double.isNaN(getHeight()) || Double.isNaN(getRadius());
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this cylinder and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other cylinder which pose and size is to be compared against this cylinder pose
    *           and size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two cylinders are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Cylinder3DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getHeight(), other.getHeight(), epsilon) && EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon)
            && getPosition().epsilonEquals(other.getPosition(), epsilon) && other.getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two cylinders are geometrically
    * similar.
    * <p>
    * This method accounts for the multiple combinations of radius/height and rotations that generate
    * identical cylinder. For instance, two cylinders that are identical but one is rotated around its
    * main axis are considered geometrically equal.
    * </p>
    *
    * @param other the cylinder to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the cylinders represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Cylinder3DReadOnly other, double epsilon)
   {
      if (Math.abs(getRadius() - other.getRadius()) > epsilon || Math.abs(getHeight() - other.getHeight()) > epsilon)
         return false;

      if (!getPosition().geometricallyEquals(getPosition(), epsilon))
         return false;

      /*
       * Here, we check that the axis the cylinder is aligned on (the Z axis, since the cylinder
       * inherently lies on the XY plane) is the same axis that the other cylinder is aligned on using
       * EuclidGeometryTools#areVector3DsParallel(). We could do this by transforming two (0, 0, 1)
       * vectors by each shapePose, but for each: / r00 r01 r02 \ / 0 \ / r02 \ | r10 r11 r12 | * | 0 | =
       * | r12 | \ r20 r21 r22 / \ 1 / \ r22 / So rather than perform this transform, just check that the
       * last column of the rotation matrix of each cylinder (M02, M12, and M22 in shapePose) are aligned
       * vectors.
       */

      return EuclidGeometryTools.areVector3DsParallel(getOrientation().getM02(), getOrientation().getM12(), getOrientation().getM22(),
                                                      other.getOrientation().getM02(), other.getOrientation().getM12(), other.getOrientation().getM22(),
                                                      epsilon);
   }
}
