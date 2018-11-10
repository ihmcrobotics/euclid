package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Write and read interface for a pose 2D.
 * <p>
 * A pose 2D represents a position and orientation in the XY-plane.
 * </p>
 */
public interface Pose2DBasics extends Pose2DReadOnly, Clearable, Transformable
{
   /**
    * Sets the x-coordinate of the position.
    *
    * @param x the x-coordinate of the position.
    */
   default void setX(double x)
   {
      getPosition().setX(x);
   }

   /**
    * Sets the y-coordinate of the position.
    *
    * @param y the y-coordinate of the position.
    */
   default void setY(double y)
   {
      getPosition().setY(y);
   }

   /**
    * Sets the orientation yaw angle value.
    *
    * @param yaw the orientation angle value.
    */
   default void setYaw(double yaw)
   {
      getOrientation().setYaw(yaw);
   }

   /**
    * Gets the reference of the position part of this pose 2D.
    *
    * @return the position part of this pose 2D.
    */
   @Override
   Point2DBasics getPosition();

   /**
    * Gets the reference of the orientation part of this pose 2D.
    *
    * @return the orientation part of this pose 2D.
    */
   @Override
   Orientation2DBasics getOrientation();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Pose2DReadOnly.super.containsNaN();
   }

   /**
    * Sets all the components of this pose 2D to zero.
    */
   @Override
   default void setToZero()
   {
      getPosition().setToZero();
      getOrientation().setToZero();
   }

   /**
    * Sets all the components of this pose 2D to {@link Double#NaN}.
    */
   @Override
   default void setToNaN()
   {
      getPosition().setToNaN();
      getOrientation().setToNaN();
   }

   /**
    * Sets this pose 2D to the {@code other} pose 2D.
    *
    * @param other the other pose 2D. Not modified.
    */
   default void set(Pose2DReadOnly other)
   {
      setPosition(other.getPosition());
      setOrientation(other.getOrientation());
   }

   /**
    * Sets this pose 2D to the given {@code pose3DReadOnly}.
    *
    * @param pose3DReadOnly the pose 3D. Not modified.
    */
   default void set(Pose3DReadOnly pose3DReadOnly)
   {
      setPosition(pose3DReadOnly.getPosition());
      setOrientation(pose3DReadOnly.getOrientation());
   }

   /**
    * Sets the position coordinates.
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    */
   default void setPosition(double x, double y)
   {
      getPosition().set(x, y);
   }

   /**
    * Sets the position to the given tuple.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    */
   default void setPosition(Tuple2DReadOnly position)
   {
      getPosition().set(position);
   }

   /**
    * Sets the position to the given tuple.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    */
   default void setPosition(Tuple3DReadOnly position)
   {
      getPosition().set(position);
   }

   /**
    * Sets the orientation angle value.
    *
    * @param yaw the orientation angle value.
    */
   default void setOrientation(double yaw)
   {
      getOrientation().setYaw(yaw);
   }

   /**
    * Sets the orientation from the given orientation 2D.
    *
    * @param orientation the orientation with the new angle value for this. Not modified.
    */
   default void setOrientation(Orientation2DReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation from the yaw angle of the given quaternion.
    *
    * @param orientation the orientation with the new angle value for this. Not modified.
    */
   default void setOrientation(QuaternionReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets all the components of this pose 2D.
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param yaw the orientation angle value.
    */
   default void set(double x, double y, double yaw)
   {
      setPosition(x, y);
      setOrientation(yaw);
   }

   /**
    * Sets this pose 2D from the given {@code position} and {@code yaw} angle.
    *
    * @param position the tuple used to initialize this pose's position. Not modified.
    * @param yaw the angle used to initialize the pose's orientation.
    */
   default void set(Tuple2DReadOnly position, double yaw)
   {
      setPosition(position);
      setOrientation(yaw);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    */
   default void set(Tuple2DReadOnly position, Orientation2DReadOnly orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   /**
    * Sets this pose 2D to match the given rigid-body transform.
    * <p>
    * The given transform has to represent a 2D transformation.
    * </p>
    *
    * @param rigidBodyTransform the transform use to set this pose 2D. Not modified.
    * @throws NotAMatrix2DException if the rotation part of the transform does not represent a 2D
    *            transformation.
    */
   default void set(RigidBodyTransform rigidBodyTransform)
   {
      set(rigidBodyTransform, true);
   }

   /**
    * Sets this pose 2D to match the given rigid-body transform.
    *
    * @param rigidBodyTransform the transform use to set this pose 2D. Not modified.
    * @param checkIsTransform2D indicates whether or not the method should check that the rotation part
    *           of the given transform represents a 2D rotation in the XY-plane.
    * @throws NotAMatrix2DException if {@code checkIsTransform2D} is {@code true} and if the rotation
    *            part of the transform does not represent a 2D transformation.
    */
   default void set(RigidBodyTransform rigidBodyTransform, boolean checkIsTransform2D)
   {
      if (checkIsTransform2D)
         rigidBodyTransform.checkIfRotation2D();

      setPosition(rigidBodyTransform.getTranslationX(), rigidBodyTransform.getTranslationY());
      setOrientation(rigidBodyTransform.getRotationMatrix().getYaw());
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 2D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *           {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    */
   default void interpolate(Pose2DReadOnly other, double alpha)
   {
      getPosition().interpolate(other.getPosition(), alpha);
      getOrientation().interpolate(other.getOrientation(), alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 2D used in the interpolation. Not modified.
    * @param pose2 the second pose 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting {@code this}
    *           to {@code pose2}.
    */
   default void interpolate(Pose2DReadOnly pose1, Pose2DReadOnly pose2, double alpha)
   {
      getPosition().interpolate(pose1.getPosition(), pose2.getPosition(), alpha);
      getOrientation().interpolate(pose1.getOrientation(), pose2.getOrientation(), alpha);
   }

   /**
    * Adds the translation (x, y) to this pose 2D assuming it is expressed in the coordinates in which
    * this pose is expressed.
    * <p>
    * If the translation is expressed in the local coordinates described by this pose 2D, use
    * {@link #appendTranslation(double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    */
   default void prependTranslation(double x, double y)
   {
      getPosition().add(x, y);
   }

   /**
    * Adds the given {@code translation} to this pose 2D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 2D, use
    * {@link #appendTranslation(Tuple2DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 2D. Not modified.
    */
   default void prependTranslation(Tuple2DReadOnly translation)
   {
      prependTranslation(translation.getX(), translation.getY());
   }

   /**
    * Rotates the position part of this pose 2D by {@code yaw} and adds {@code yaw} to the orientation
    * part.
    * <p>
    * If the rotation should not affect this pose's position, use {@link #appendRotation(double)}.
    * </p>
    *
    * @param yaw the angle about the z-axis to prepend to this pose 2D.
    */
   default void prependRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, getPosition(), getPosition());
      getOrientation().add(yaw);
   }

   /**
    * Rotates the position part of this pose 2D by {@code orientation} and adds {@code orientation} to
    * the orientation part.
    * <p>
    * If the rotation should not affect this pose's position, use
    * {@link #appendRotation(Orientation2DReadOnly)}.
    * </p>
    *
    * @param orientation the orientation to prepend to this pose 2D. Not modified.
    */
   default void prependRotation(Orientation2DReadOnly orientation)
   {
      prependRotation(orientation.getYaw());
   }

   /**
    * Rotates, then adds the translation (x, y) to this pose 2D.
    * <p>
    * Use this method if the translation (x, y) is expressed in the local coordinates described by this
    * pose 2D. Otherwise, use {@link #prependTranslation(double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    */
   default void appendTranslation(double x, double y)
   {
      double thisX = getPosition().getX();
      double thisY = getPosition().getY();

      getPosition().set(x, y);
      getOrientation().transform(getPosition());
      getPosition().add(thisX, thisY);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 2D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 2D. Otherwise, use {@link #prependTranslation(Tuple2DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 2D. Not modified.
    */
   default void appendTranslation(Tuple2DReadOnly translation)
   {
      appendTranslation(translation.getX(), translation.getY());
   }

   /**
    * Adds the given {@code yaw} angle to the orientation of this pose 2D.
    * <p>
    * If the position part of this pose 2D is to be rotated by the given angle, use
    * {@link #prependRotation(double)}.
    * </p>
    *
    * @param yaw the angle about the z-axis to append to this pose 2D.
    */
   default void appendRotation(double yaw)
   {
      getOrientation().add(yaw);
   }

   /**
    * Adds the given {@code orientation} to the orientation of this pose 2D.
    * <p>
    * If the position part of this pose 2D is to be rotated by the given {@code orientation}, use
    * {@link #prependRotation(Orientation2DReadOnly)}.
    * </p>
    *
    * @param orientation the orientation to append to this pose 2D. Not modified.
    */
   default void appendRotation(Orientation2DReadOnly orientation)
   {
      appendRotation(orientation.getYaw());
   }

   /**
    * Transforms the position and orientation parts of this pose 2D by the given {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 2D. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY plane.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      getPosition().applyTransform(transform);
      getOrientation().applyTransform(transform);
   }

   /**
    * Transforms the position and orientation parts of this pose 2D by the inverse of the given
    * {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 2D. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY plane.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      getPosition().applyInverseTransform(transform);
      getOrientation().applyInverseTransform(transform);
   }
}
