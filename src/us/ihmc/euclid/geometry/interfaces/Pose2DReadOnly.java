package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;

public interface Pose2DReadOnly
{
   /**
    * Gets the x-coordinate of the position part of this pose 2D.
    *
    * @return the x-coordinate of this pose 2D.
    */
   double getX();

   /**
    * Gets the y-coordinate of the position part of this pose 2D.
    *
    * @return the y-coordinate of this pose 2D.
    */
   double getY();

   /**
    * Gets the yaw angle of the orientation part of this pose 2D.
    *
    * @return the yaw angle of this pose 2D.
    */
   double getYaw();

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
    * Packs the position part of this pose 2D into the given {@code positionToPack}.
    *
    * @param positionToPack tuple used to store the position coordinates. Modified.
    */
   default void getPosition(Tuple2DBasics positionToPack)
   {
      positionToPack.set(getPosition());
   }

   /**
    * Packs the orientation part of this pose 2D into the given {@code orientationToPack}.
    *
    * @param orientationToPack used to store the orientation of this pose 2D. Modified.
    */
   default void getOrientation(Orientation2D orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   /**
    * Packs this pose 2D into the given {@code transformToPack}.
    *
    * @param transformToPack the rigid-body transform that is set to represent this pose 2D.
    *           Modified.
    */
   default void get(RigidBodyTransform transformToPack)
   {
      transformToPack.setTranslation(getX(), getY(), 0.0);
      transformToPack.setRotationYaw(getYaw());
   }

   /**
    * Computes the distance between the position of this pose 2D and the given {@code point}.
    *
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    */
   default double getPositionDistance(Point2DReadOnly point)
   {
      return getPosition().distance(point);
   }

   /**
    * Computes the distances between the position part of the two poses.
    *
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    */
   default double getPositionDistance(Pose2DReadOnly other)
   {
      return getPosition().distance(other.getPosition());
   }

   /**
    * Computes the absolute angle difference between the orientation part of this pose 2D and the
    * give {@code orientation}.
    *
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this} and {@code orientation}.
    */
   default double getOrientationDistance(Orientation2D other)
   {
      return getOrientation().distance(other);
   }

   /**
    * Computes the absolute angle difference between this pose 2D and {@code other}.
    *
    * @param other the other pose 2D used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this.orientation} and
    *         {@code other.orientation}.
    */
   default double getOrientationDistance(Pose2DReadOnly other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with separate tolerances
    * for the position {@code positionEpsilon} and the orientation {@code orientationEpsilon}.
    *
    * @param other the query. Not modified.
    * @param positionEpsilon the tolerance to use for comparing the position part.
    * @param orientationEpsilon the tolerance to use for comparing the orientation part.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Pose2DReadOnly other, double epsilon) {
      return epsilonEquals(other, epsilon, epsilon);
   }
   
   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with separate tolerances
    * for the position {@code positionEpsilon} and the orientation {@code orientationEpsilon}.
    *
    * @param other the query. Not modified.
    * @param positionEpsilon the tolerance to use for comparing the position part.
    * @param orientationEpsilon the tolerance to use for comparing the orientation part.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Pose2DReadOnly other, double positionEpsilon, double orientationEpsilon)
   {
      return getPosition().epsilonEquals(other.getPosition(), positionEpsilon) && getOrientation().epsilonEquals(other.getOrientation(), orientationEpsilon);
   }
   
   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically
    * equal.
    * </p>
    *
    * @param other the pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Pose2DReadOnly other, double epsilon)
   {
      return getPosition().geometricallyEquals(other.getPosition(), epsilon) && getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
   }
}
