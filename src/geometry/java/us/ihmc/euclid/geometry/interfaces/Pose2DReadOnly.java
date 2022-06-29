package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Read-only interface for a pose 2D.
 * <p>
 * A pose 2D represents a position and orientation in the XY-plane.
 * </p>
 */
public interface Pose2DReadOnly extends EuclidGeometry
{
   /**
    * Gets the x-coordinate of the position part of this pose 2D.
    *
    * @return the x-coordinate of this pose 2D.
    */
   default double getX()
   {
      return getPosition().getX();
   }

   /**
    * Gets the y-coordinate of the position part of this pose 2D.
    *
    * @return the y-coordinate of this pose 2D.
    */
   default double getY()
   {
      return getPosition().getY();
   }

   /**
    * Gets the yaw angle of the orientation part of this pose 2D.
    *
    * @return the yaw angle of this pose 2D.
    */
   default double getYaw()
   {
      return getOrientation().getYaw();
   }

   /**
    * Tests if this pose 2D contains a {@link Double#NaN}.
    *
    * @return {@code true} if either the position or orientation part of this pose 2D contains
    *         {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getOrientation().containsNaN();
   }

   /**
    * Gets the read-only reference of the position part of this pose 2D.
    *
    * @return the position part of this pose 2D.
    */
   Point2DReadOnly getPosition();

   /**
    * Gets the read-only reference of the orientation part of this pose 2D.
    *
    * @return the orientation part of this pose 2D.
    */
   Orientation2DReadOnly getOrientation();

   /**
    * Packs this pose 2D into the given {@code transformToPack}.
    *
    * @param transformToPack the rigid-body transform that is set to represent this pose 2D. Modified.
    */
   default void get(RigidBodyTransformBasics transformToPack)
   {
      transformToPack.getTranslation().set(getX(), getY(), 0.0);
      transformToPack.getRotation().setToYawOrientation(getYaw());
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Pose2DReadOnly))
         return false;
      Pose2DReadOnly other = (Pose2DReadOnly) geometry;
      return getPosition().equals(other.getPosition()) && getOrientation().equals(other.getOrientation());
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with separate tolerances
    * for the position {@code positionEpsilon} and the orientation {@code orientationEpsilon}.
    *
    * @param geometry the query. Not modified.
    * @param epsilon  the tolerance to use.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Pose2DReadOnly))
         return false;
      Pose2DReadOnly other = (Pose2DReadOnly) geometry;
      return getPosition().epsilonEquals(other.getPosition(), epsilon) && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Pose2DReadOnly))
         return false;
      Pose2DReadOnly other = (Pose2DReadOnly) geometry;
      return getPosition().geometricallyEquals(other.getPosition(), epsilon) && getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
   }

   /**
    * Gets a representative {@code String} of this pose 2D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Pose 2D: position = ( 0.174, -0.222 ), orientation = (-0.130 )
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidGeometryIOTools.getPose2DString(format, this);
   }
}
