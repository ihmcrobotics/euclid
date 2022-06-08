package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
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

   /**
    * Computes the distance between the position of this pose 2D and the given {@code point}.
    *
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    * @deprecated Use {@code this.getPosition().distance(point)} instead.
    */
   @Deprecated
   default double getPositionDistance(Point2DReadOnly point)
   {
      return getPosition().distance(point);
   }

   /**
    * Computes the distances between the position part of the two poses.
    *
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    * @deprecated Use {@code this.getPosition().distance(other.getPosition())} instead.
    */
   @Deprecated
   default double getPositionDistance(Pose2DReadOnly other)
   {
      return getPosition().distance(other.getPosition());
   }

   /**
    * Computes the absolute angle difference between the orientation part of this pose 2D and the give
    * {@code orientation}.
    *
    * @param other the orientation used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this} and {@code orientation}.
    * @deprecated Use {@code this.getOrientation().distance(other))} instead.
    */
   @Deprecated
   default double getOrientationDistance(Orientation2DReadOnly other)
   {
      return getOrientation().distance(other);
   }

   /**
    * Computes the absolute angle difference between this pose 2D and {@code other}.
    *
    * @param other the other pose 2D used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this.orientation} and
    *         {@code other.orientation}.
    * @deprecated Use {@code this.getOrientation().distance(other.getOrientation())} instead.
    */
   @Deprecated
   default double getOrientationDistance(Pose2DReadOnly other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   /**
    * Tests on a per component basis, if this pose 2D is exactly equal to {@code other}.
    *
    * @param other the other pose 2D to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Pose2DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getPosition().equals(other.getPosition()) && getOrientation().equals(other.getOrientation());
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with separate tolerances
    * for the position {@code positionEpsilon} and the orientation {@code orientationEpsilon}.
    *
    * @param object  the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(Object object, double epsilon)
   {
      if (!(object instanceof Pose2DReadOnly))
         return false;
      Pose2DReadOnly other = (Pose2DReadOnly) object;
      return getPosition().epsilonEquals(other.getPosition(), epsilon) && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically equal.
    * </p>
    *
    * @param object  the object to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      if (!(object instanceof Pose2DReadOnly))
         return false;
      Pose2DReadOnly other = (Pose2DReadOnly) object;
      return getPosition().geometricallyEquals(other.getPosition(), epsilon) && getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
   }

   /**
    * Gets a representative {@code String} of {@code pose2D} as follows:
    *
    * <pre>
    * Pose 2D: position = ( 0.174, -0.222 ), orientation = (-0.130 )
    * </pre>
    *
    * @param format the format to be used.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidGeometryIOTools.getPose2DString(format, this);
   }
}
