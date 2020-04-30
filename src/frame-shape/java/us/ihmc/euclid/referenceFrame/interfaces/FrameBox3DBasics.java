package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a box 3D expressed in a changeable reference frame, i.e. the
 * reference frame in which this line is expressed can be changed.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameBox3DBasics extends FixedFrameBox3DBasics, FrameShape3DBasics
{
   /**
    * Copies the {@code other} box data into {@code this} and updates the reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other box to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Box3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} box data into {@code this} and updates the reference frame.
    *
    * @param other the other box to copy. Not modified.
    */
   default void setIncludingFrame(FrameBox3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY,
                                  double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param size           the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param position    the position of this box center. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX       the size along the x-axis.
    * @param sizeY       the size along the y-axis.
    * @param sizeZ       the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param position    the position of this box center. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param size        the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param position    the position of this box center. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param size        the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly size)
   {
      position.checkReferenceFrameMatch(orientation, size);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, Vector3DReadOnly size)
   {
      setReferenceFrame(referenceFrame);
      set(pose, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setIncludingFrame(FramePose3DReadOnly pose, Vector3DReadOnly size)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setIncludingFrame(FramePose3DReadOnly pose, FrameVector3DReadOnly size)
   {
      pose.checkReferenceFrameMatch(size);
      setIncludingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, Vector3DReadOnly size)
   {
      setReferenceFrame(referenceFrame);
      set(pose, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param pose  the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, Vector3DReadOnly size)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param pose  the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, FrameVector3DReadOnly size)
   {
      pose.checkReferenceFrameMatch(size);
      setIncludingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] size)
   {
      setReferenceFrame(referenceFrame);
      set(pose, size);
   }

   /**
    * Sets this box properties and its reference frame.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double[] size)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, size);
   }

   /** {@inheritDoc} */
   @Override
   FrameBox3DBasics copy();
}
