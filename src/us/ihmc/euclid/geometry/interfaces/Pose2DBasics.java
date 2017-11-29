package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public interface Pose2DBasics extends Pose2DReadOnly, Clearable
{
   /**
    * Sets the x-coordinate of the position.
    *
    * @param x the x-coordinate of the position.
    */
   void setX(double x);

   /**
    * Sets the y-coordinate of the position.
    *
    * @param y the y-coordinate of the position.
    */
   void setY(double y);

   /**
    * Sets the orientation yaw angle value.
    *
    * @param yaw the orientation angle value.
    */
   void setYaw(double yaw);

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
    * Sets the position coordinates.
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    */
   default void setPosition(double x, double y)
   {
      setX(x);
      setY(y);
   }

   /**
    * Sets the position to the given tuple.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    */
   default void setPosition(Tuple2DReadOnly position)
   {
      setPosition(position.getX(), position.getY());
   }

   /**
    * Sets the orientation angle value.
    *
    * @param yaw the orientation angle value.
    */
   default void setOrientation(double yaw)
   {
      setYaw(yaw);
   }

   /**
    * Sets the orientation from the given orientation 2D.
    *
    * @param orientation the orientation with the new angle value for this. Not modified.
    */
   default void setOrientation(Orientation2DReadOnly orientation)
   {
      setYaw(orientation.getYaw());
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
    * @param checkIsTransform2D indicates whether or not the method should check that the rotation
    *           part of the given transform represents a 2D rotation in the XY-plane.
    * @throws NotAMatrix2DException if {@code checkIsTransform2D} is {@code true} and if the
    *            rotation part of the transform does not represent a 2D transformation.
    */
   default void set(RigidBodyTransform rigidBodyTransform, boolean checkIsTransform2D)
   {
      if (checkIsTransform2D)
         rigidBodyTransform.checkIfRotation2D();

      setPosition(rigidBodyTransform.getTranslationX(), rigidBodyTransform.getTranslationY());
      setOrientation(rigidBodyTransform.getRotationMatrix().getYaw());
   }
}
