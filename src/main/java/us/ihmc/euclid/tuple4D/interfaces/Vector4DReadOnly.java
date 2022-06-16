package us.ihmc.euclid.tuple4D.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
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
    * <p>
    * Calculates the norm squared of the two vectors (this and other)
    * </p>
    * 
    * @param other the other vector to compare to.
    * @return |V<SUB>this</SUB> - V<SUB>other</SUB>|<SUP>2</SUP>
    */
   default double differenceLengthSquared(Vector4DReadOnly other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      double dz = getZ() - other.getZ();
      double ds = getS() - other.getS();
      return EuclidCoreTools.normSquared(dx, dy, dz, ds);
   }

   /**
    * <p>
    * Calculates the norm of the two vectors (this and other)
    * </p>
    * 
    * @param other the other vector to compare to.
    * @return |V<SUB>this</SUB> - V<SUB>other</SUB>|
    */
   default double differenceLength(Vector4DReadOnly other)
   {
      return EuclidCoreTools.squareRoot(differenceLengthSquared(other));
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof Vector4DReadOnly))
         return false;
      Vector4DReadOnly other = (Vector4DReadOnly) geometry;
      return differenceLength(other) <= epsilon;
   }
}
