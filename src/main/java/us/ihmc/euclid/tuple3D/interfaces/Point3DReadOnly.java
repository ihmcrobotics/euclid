package us.ihmc.euclid.tuple3D.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Read-only interface for a 3 dimensional point.
 * <p>
 * A 3D point represents the 3D coordinates of a location in space.
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
public interface Point3DReadOnly extends Tuple3DReadOnly
{
   /**
    * Calculates and returns the distance between this point and {@code other}.
    *
    * @param other the other point used to measure the distance.
    * @return the distance between the two points.
    */
   default double distance(Point3DReadOnly other)
   {
      return differenceNorm(other);
   }

   /**
    * Calculates and returns the square of the distance between this point and {@code other}.
    * <p>
    * This method is usually preferred over {@link #distance(Point3DReadOnly)} when calculation speed
    * matters and knowledge of the actual distance does not, i.e. when comparing distances between
    * several pairs of points.
    * </p>
    *
    * @param other the other point used to measure the square of the distance.
    * @return the square of the distance between the two points.
    */
   default double distanceSquared(Point3DReadOnly other)
   {
      return differenceNormSquared(other);
   }

   /**
    * Calculates and returns the distance between this point and {@code other} in the XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - other.x)<sup>2</sup> + (this.y - other.y)<sup>2</sup>)
    * </p>
    *
    * @param other the other point used to measure the distance.
    * @return the distance between the two points in the XY-plane.
    */
   default double distanceXY(Point3DReadOnly other)
   {
      return EuclidCoreTools.squareRoot(distanceXYSquared(other));
   }

   /**
    * Calculates and returns the square of the distance between this point and {@code other} in the
    * XY-plane.
    * <p>
    * Effectively, this calculates the distance squared as follows:<br>
    * d<sub>xy</sub><sup>2</sup> = (this.x - other.x)<sup>2</sup> + (this.y - other.y)<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #distanceXY(Point3DReadOnly)} when calculation speed
    * matters and knowledge of the actual distance does not, i.e. when comparing distances between
    * several pairs of points.
    * </p>
    *
    * @param other the other point used to measure the square of the distance.
    * @return the square of the distance between the two points in the XY-plane.
    */
   default double distanceXYSquared(Point3DReadOnly other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      return EuclidCoreTools.normSquared(dx, dy);
   }

   /**
    * Calculates and returns the distance between this point and {@code point2DReadOnly} in the
    * XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - point2DReadOnly.x)<sup>2</sup> + (this.y -
    * point2DReadOnly.y)<sup>2</sup>)
    * </p>
    *
    * @param point2DReadOnly the other point used to measure the distance.
    * @return the distance between the two points in the XY-plane.
    */
   default double distanceXY(Point2DReadOnly point2DReadOnly)
   {
      return EuclidCoreTools.squareRoot(distanceXYSquared(point2DReadOnly));
   }

   /**
    * Calculates and returns the square of the distance between this point and {@code point2DReadOnly}
    * in the XY-plane.
    * <p>
    * Effectively, this calculates the distance squared as follows:<br>
    * d<sub>xy</sub><sup>2</sup> = (this.x - point2DReadOnly.x)<sup>2</sup> + (this.y -
    * point2DReadOnly.y)<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #distanceXY(Point2DReadOnly)} when calculation speed
    * matters and knowledge of the actual distance does not, i.e. when comparing distances between
    * several pairs of points.
    * </p>
    *
    * @param point2DReadOnly the other point used to measure the square of the distance.
    * @return the square of the distance between the two points in the XY-plane.
    */
   default double distanceXYSquared(Point2DReadOnly point2DReadOnly)
   {
      double dx = getX() - point2DReadOnly.getX();
      double dy = getY() - point2DReadOnly.getY();
      return EuclidCoreTools.normSquared(dx, dy);
   }

   /**
    * Calculates and returns the distance between this point and the origin (0, 0, 0).
    *
    * @return the distance between this point and the origin.
    */
   default double distanceFromOrigin()
   {
      return EuclidCoreTools.squareRoot(distanceFromOriginSquared());
   }

   /**
    * Calculates and returns the square of the distance between this point and the origin (0, 0, 0).
    * <p>
    * This method is usually preferred over {@link #distanceFromOrigin()} when calculation speed
    * matters and knowledge of the actual distance does not, i.e. when comparing distances of several
    * points with respect to the origin.
    * </p>
    *
    * @return the square of the distance between this point and the origin.
    */
   default double distanceFromOriginSquared()
   {
      return EuclidCoreTools.normSquared(getX(), getY(), getZ());
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Point3DReadOnly))
         return false;
      Point3DReadOnly other = (Point3DReadOnly) geometry;
      return distance(other) <= epsilon;
   }
}
