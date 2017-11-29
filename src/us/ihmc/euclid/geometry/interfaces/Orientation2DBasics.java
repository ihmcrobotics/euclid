package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;

public interface Orientation2DBasics extends Orientation2DReadOnly
{
   /**
    * Sets the yaw angle of this orientation 2D.
    * <p>
    * Note that the argument is trimmed to be contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param yaw the new yaw angle value in radians.
    */
   void setYaw(double yaw);
   
   /**
    * Sets this orientation 2D to the {@code other} orientation 2D.
    *
    * @param other the other orientation 2D. Not modified.
    */
   default void set(Orientation2DReadOnly other) {
      setYaw(other.getYaw());
   }

   /**
    * Adds the given {@code yaw} angle to this orientation 2D:<br>
    * {@code this.yaw += yaw}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param yaw the angle to add to this.
    */
   default void add(double yaw)
   {
      add(getYaw(), yaw);
   }

   /**
    * Sets this orientation 2D to the sum of the two given yaw angles:<br>
    * {@code this.yaw = yaw1 + yaw2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param yaw1 the first yaw angle.
    * @param yaw2 the second yaw angle.
    */
   default void add(double yaw1, double yaw2)
   {
      setYaw(EuclidCoreTools.trimAngleMinusPiToPi(yaw1 + yaw2));
   }

   /**
    * Adds the other orientation 2D to this:<br>
    * {@code this += other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param other the other orientation 2D to add to this. Not modified.
    */
   default void add(Orientation2DReadOnly other)
   {
      add(other.getYaw());
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   default void add(Orientation2DReadOnly orientation1, Orientation2DReadOnly orientation2)
   {
      add(orientation1.getYaw(), orientation2.getYaw());
   }

   /**
    * Subtracts the given {@code yaw} angle from this orientation 2D:<br>
    * {@code this.yaw -= yaw}
    *
    * @param yaw the angle to subtract.
    */
   default void sub(double yaw)
   {
      sub(getYaw(), yaw);
   }

   /**
    * Subtracts the other orientation 2D from this orientation 2D:<br>
    * {@code this -= other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param other the other orientation 2D to subtract. Not modified.
    */
   default void sub(Orientation2DReadOnly other)
   {
      sub(other.getYaw());
   }

   /**
    * Sets this orientation 2D to the difference of the two given yaw angles:<br>
    * {@code this.yaw = yaw1 - yaw2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param yaw1 the first yaw angle.
    * @param yaw2 the second yaw angle.
    */
   default void sub(double yaw1, double yaw2)
   {
      setYaw(EuclidCoreTools.angleDifferenceMinusPiToPi(yaw1, yaw2));
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   default void sub(Orientation2DReadOnly orientation1, Orientation2DReadOnly orientation2)
   {
      sub(orientation1.getYaw(), orientation2.getYaw());
   }
}
