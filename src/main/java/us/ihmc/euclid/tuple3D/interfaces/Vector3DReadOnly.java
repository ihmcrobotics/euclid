package us.ihmc.euclid.tuple3D.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.TupleTools;

/**
 * Read-only interface for a 3 dimensional vector.
 * <p>
 * A 3D vector represents a physical quantity with a magnitude and a direction. For instance, it can
 * be used to represent a 3D velocity, force, or translation from one 3D point to another.
 * </p>
 * <p>
 * Although a point and vector hold onto the same type of information, the distinction is made
 * between them as they represent different geometry objects and are typically not handled the same
 * way:
 * <ul>
 * <li>a point represents the coordinate of a location in space. A notable difference with a vector
 * is that the distance between two points has a physical meaning. When a point is transformed with
 * a homogeneous transformation matrix, a point's coordinates are susceptible to be scaled, rotated,
 * and translated.
 * <li>a vector is not constrained to a location in space. Instead, a vector represents some
 * physical quantity that has a direction and a magnitude such as: a velocity, a force, the
 * translation from one point to another, etc. When a vector is transformed with a homogeneous
 * transformation matrix, its components are susceptible to be scaled and rotated, but never to be
 * translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Vector3DReadOnly extends Tuple3DReadOnly
{
   /**
    * Calculates and returns the magnitude of this vector.
    * <p>
    * length = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
    * </p>
    *
    * @return the magnitude of this vector.
    * @deprecated Use {@link Tuple3DReadOnly#norm()}.
    */
   @Deprecated
   default double length()
   {
      return norm();
   }

   /**
    * Calculates and returns the square of the magnitude of this vector.
    * <p>
    * length<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #length()} when calculation speed matters and
    * knowledge of the actual magnitude does not, i.e. when comparing several vectors by theirs
    * magnitude.
    * </p>
    *
    * @return the square of the magnitude of this vector.
    * @deprecated Use {@link Tuple3DReadOnly#normSquared()}.
    */
   @Deprecated
   default double lengthSquared()
   {
      return normSquared();
   }

   /**
    * Calculates and returns the angle in radians from this vector to {@code other}.
    * <p>
    * The computed angle is in the range [0, <i>pi</i>].
    * </p>
    *
    * @param other the other vector used to compute the angle. Not modified.
    * @return the value of the angle from this vector to {@code other}.
    */
   default double angle(Vector3DReadOnly other)
   {
      return TupleTools.angle(this, other);
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Vector3DReadOnly))
         return false;
      Vector3DReadOnly other = (Vector3DReadOnly) geometry;
      return differenceNorm((Vector3DReadOnly) other) <= epsilon;
   }
}
