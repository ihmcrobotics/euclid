package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * Write and read interface for a 2D orientation.
 * <p>
 * A 2D orientation is in the XY-plane, i.e. the yaw angle about the z-axis.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface Orientation2DBasics extends Orientation2DReadOnly, Clearable, Transformable
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
    * Sets the yaw angle of this orientation 2D to zero.
    */
   @Override
   default void setToZero()
   {
      setYaw(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      setYaw(Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Orientation2DReadOnly.super.containsNaN();
   }

   /**
    * Sets this orientation 2D to the {@code other} orientation 2D.
    *
    * @param other the other orientation 2D. Not modified.
    */
   default void set(Orientation2DReadOnly other)
   {
      setYaw(other.getYaw());
   }

   /**
    * Sets this orientation 2D to the yaw angle of the given {@code orientation3DReadOnly}.
    *
    * @param orientation3DReadOnly the orientation to get the yaw angle from. Not modified.
    */
   default void set(Orientation3DReadOnly orientation3DReadOnly)
   {
      setYaw(orientation3DReadOnly.getYaw());
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

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other orientation 2D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *           {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    */
   default void interpolate(Orientation2DReadOnly other, double alpha)
   {
      interpolate(this, other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code orientation1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code orientation2}.
    */
   default void interpolate(Orientation2DReadOnly orientation1, Orientation2DReadOnly orientation2, double alpha)
   {
      double deltaYaw = EuclidCoreTools.angleDifferenceMinusPiToPi(orientation2.getYaw(), orientation1.getYaw());
      add(orientation1.getYaw(), alpha * deltaYaw);
   }

   /**
    * Transforms this orientation 2D by the given {@code transform}.
    * <p>
    * This is equivalent to extracting the yaw rotation part from the given transform and adding it to
    * this.
    * </p>
    *
    * @param transform the geometric transform to apply on this orientation 2D. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY plane.
    */
   @Override
   void applyTransform(Transform transform);

   /**
    * Transforms this orientation 2D by the inverse of the given {@code transform}.
    * <p>
    * This is equivalent to extracting the yaw rotation part from the given transform and subtracting
    * it to this.
    * </p>
    *
    * @param transform the geometric transform to apply on this orientation 2D. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY plane.
    */
   @Override
   void applyInverseTransform(Transform transform);
}
