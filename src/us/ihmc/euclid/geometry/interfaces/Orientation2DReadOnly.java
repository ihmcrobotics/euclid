package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;

public interface Orientation2DReadOnly
{
   /**
    * Returns the current yaw angle of this orientation 2D.
    *
    * @return the angle value in radians.
    */
   double getYaw();

   /**
    * Computes and returns the difference between {@code this} and {@code other}:<br>
    * {@code distance = this.yaw - other.yaw}
    *
    * @param other the other orientation 2D. Not modified.
    * @return the difference between {@code this} and {@code other} contained in [-<i>pi</i>,
    *         <i>pi</pi>].
    */
   default double difference(Orientation2DReadOnly other)
   {
      return EuclidCoreTools.angleDifferenceMinusPiToPi(getYaw(), other.getYaw());
   }

   /**
    * Computes the distance between {@code this} and {@code other} as the absolute difference in
    * angle:<br>
    * {@code distance = Math.abs(this.yaw - other.yaw)}
    *
    * @param other the other orientation 2D. Not modified.
    * @return the distance between {@code this} and {@code other} contained in [0, <i>pi</pi>].
    */
   default double distance(Orientation2DReadOnly other)
   {
      return Math.abs(difference(other));
   }

   /**
    * Tests if the yaw angle of this orientation is equal to an {@code epsilon} to the yaw of
    * {@code other}.
    * <p>
    * Note that this method performs number comparison and not an angle comparison, such that:
    * -<i>pi</i> &ne; <i>pi</i>.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two orientations are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Orientation2DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getYaw(), other.getYaw(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two orientations are geometrically
    * similar, i.e. the difference in yaw of {@code this} and {@code other} is less than or equal to
    * {@code epsilon}.
    *
    * @param other the orientation to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(Orientation2DReadOnly other, double epsilon)
   {
      return Math.abs(difference(other)) <= epsilon;
   }
}
