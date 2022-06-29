package us.ihmc.euclid.tuple2D.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read-only interface for a 2 dimensional point.
 * <p>
 * A 2D point represents the 2D coordinates of a location on the XY-plane.
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
public interface Point2DReadOnly extends Tuple2DReadOnly
{
   /**
    * Calculates and returns the distance between this point and {@code other}.
    *
    * @param other the other point used to measure the distance. Not modified.
    * @return the distance between the two points.
    */
   default double distance(Point2DReadOnly other)
   {
      return differenceNorm(other);
   }

   /**
    * Calculates and returns the square of the distance between this point and {@code other}.
    * <p>
    * This method is usually preferred over {@link #distance(Point2DReadOnly)} when calculation speed
    * matters and knowledge of the actual distance does not, i.e. when comparing distances between
    * several pairs of points.
    * </p>
    *
    * @param other the other point used to measure the square of the distance. Not modified.
    * @return the square of the distance between the two points.
    */
   default double distanceSquared(Point2DReadOnly other)
   {
      return differenceNormSquared(other);
   }

   /**
    * Calculates and returns the distance between this point and {@code point3DReadOnly} in the
    * XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - point3DReadOnly.x)<sup>2</sup> + (this.y -
    * point3DReadOnly.y)<sup>2</sup>)
    * </p>
    *
    * @param point3DReadOnly the other point used to measure the distance.
    * @return the distance between the two points in the XY-plane.
    */
   default double distanceXY(Point3DReadOnly point3DReadOnly)
   {
      return EuclidCoreTools.squareRoot(distanceXYSquared(point3DReadOnly));
   }

   /**
    * Calculates and returns the square of the distance between this point and {@code point3DReadOnly}
    * in the XY-plane.
    * <p>
    * Effectively, this calculates the distance squared as follows:<br>
    * d<sub>xy</sub><sup>2</sup> = (this.x - point3DReadOnly.x)<sup>2</sup> + (this.y -
    * point3DReadOnly.y)<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #distanceXY(Point3DReadOnly)} when calculation speed
    * matters and knowledge of the actual distance does not, i.e. when comparing distances between
    * several pairs of points.
    * </p>
    *
    * @param point3DReadOnly the other point used to measure the square of the distance.
    * @return the square of the distance between the two points in the XY-plane.
    */
   default double distanceXYSquared(Point3DReadOnly point3DReadOnly)
   {
      double dx = getX() - point3DReadOnly.getX();
      double dy = getY() - point3DReadOnly.getY();
      return EuclidCoreTools.normSquared(dx, dy);
   }

   /**
    * Calculates and returns the distance between this point and the origin (0, 0).
    *
    * @return the distance between this point and the origin.
    */
   default double distanceFromOrigin()
   {
      return EuclidCoreTools.squareRoot(distanceFromOriginSquared());
   }

   /**
    * Calculates and returns the square of the distance between this point and the origin (0, 0).
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
      return EuclidCoreTools.normSquared(getX(), getY());
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Point2DReadOnly))
         return false;
      Point2DReadOnly other = (Point2DReadOnly) geometry;
      return distance(other) <= epsilon;
   }
}
