package us.ihmc.euclid.tuple4D.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;

/**
 * Read-only interface for a 4 dimensional vector representing a generic quaternion.
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part
 * {@code s} and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 * <li>When transformed by a homogeneous transformation matrix, a quaternion is only pre-multiplied
 * by the rotation part of the transform, resulting in concatenating the orientations of the
 * transform and the quaternion.
 * <li>When transformed by a homogeneous transformation matrix, a 4D vector scalar part {@code s}
 * remains unchanged. The vector part ({@code x}, {@code y}, {@code z}) is scaled and rotated, and
 * translated by {@code s} times the translation part of the transform. Note that for {@code s = 0},
 * a 4D vector behaves as a 3D vector, and for {@code s = 1} it behaves as a 3D point.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Vector4DReadOnly extends Tuple4DReadOnly
{
   /**
    * Tests if {@code this} and {@code other} represent the same vector 4D to an {@code epsilon}.
    * <p>
    * Two vectors are considered geometrically equal if the length of their difference is less than or
    * equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other vector 4D to compare against this. Not modified.
    * @param epsilon the maximum length of the difference vector can be for the two vectors to be
    *                considered equal.
    * @return {@code true} if the two vectors represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Vector4DReadOnly other, double epsilon)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      double dz = getZ() - other.getZ();
      double ds = getS() - other.getS();
      return EuclidCoreTools.squareRoot(EuclidCoreTools.normSquared(dx, dy, dz, ds)) <= epsilon;
   }
}
