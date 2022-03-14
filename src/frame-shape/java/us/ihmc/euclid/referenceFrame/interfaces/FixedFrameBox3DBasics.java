package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a box 3D expressed in a constant reference frame, i.e. the reference
 * frame of this object cannot be changed via this interface.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameBox3DBasics extends Box3DBasics, FrameBox3DReadOnly, FixedFrameShape3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFrameVector3DBasics getSize();

   /** {@inheritDoc} */
   @Override
   FixedFrameShape3DPoseBasics getPose();

   /** {@inheritDoc} */
   @Override
   default FixedFrameRotationMatrixBasics getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /** {@inheritDoc} */
   @Override
   default FixedFramePoint3DBasics getPosition()
   {
      return getPose().getShapePosition();
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other box to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Box3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    *
    * @param other the other box to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameBox3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this box properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(position, orientation, size);
   }

   /**
    * Sets this box properties.
    *
    * @param position    the position of this box center. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX       the size along the x-axis.
    * @param sizeY       the size along the y-axis.
    * @param sizeZ       the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      position.checkReferenceFrameMatch(orientation);
      set(position.getReferenceFrame(), position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param position    the position of this box center. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param size        the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      position.checkReferenceFrameMatch(orientation);
      set(position.getReferenceFrame(), position, orientation, size);
   }

   /**
    * Sets this box properties.
    *
    * @param position    the position of this box center. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param size        the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly size)
   {
      position.checkReferenceFrameMatch(orientation, size);
      set(position.getReferenceFrame(), position, orientation, size);
   }

   /**
    * Sets this box properties.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose, Vector3DReadOnly size)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, size);
   }

   /**
    * Sets this box properties.
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePose3DReadOnly pose, Vector3DReadOnly size)
   {
      set(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FramePose3DReadOnly pose, FrameVector3DReadOnly size)
   {
      pose.checkReferenceFrameMatch(size);
      set(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, Vector3DReadOnly size)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, size);
   }

   /**
    * Sets this box properties.
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, Vector3DReadOnly size)
   {
      set(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, FrameVector3DReadOnly size)
   {
      pose.checkReferenceFrameMatch(size);
      set(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] size)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, size);
   }

   /**
    * Sets this box properties.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, double[] size)
   {
      set(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Box3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other box to copy. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Box3DReadOnly other)
   {
      set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameBox3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other box to copy. Not modified.
    */
   default void setMatchingFrame(FrameBox3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this box properties.
    * <p>
    * If arguments are expressed in the same frame as {@code this}, then this method is equivalent to
    * {@link #set(ReferenceFrame, Point3DReadOnly, Orientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame,
                                 Point3DReadOnly position,
                                 Orientation3DReadOnly orientation,
                                 double sizeX,
                                 double sizeY,
                                 double sizeZ)
   {
      set(position, orientation, sizeX, sizeY, sizeZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this box properties.
    * <p>
    * If arguments are expressed in the same frame as {@code this}, then this method is equivalent to
    * {@link #set(ReferenceFrame, Point3DReadOnly, Orientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      set(position, orientation, size);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Point3DReadOnly, Orientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Point3DReadOnly, Orientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      position.checkReferenceFrameMatch(orientation);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, size);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Point3DReadOnly, Orientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param position       the position of this box center. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly size)
   {
      position.checkReferenceFrameMatch(orientation, size);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, size);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Pose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Pose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, Vector3DReadOnly size)
   {
      set(pose, size);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Pose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setMatchingFrame(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Pose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setMatchingFrame(FramePose3DReadOnly pose, Vector3DReadOnly size)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Pose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setMatchingFrame(FramePose3DReadOnly pose, FrameVector3DReadOnly size)
   {
      pose.checkReferenceFrameMatch(size);
      setMatchingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param sizeX          the size along the x-axis.
    * @param sizeY          the size along the y-axis.
    * @param sizeZ          the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, Vector3DReadOnly size)
   {
      set(pose, size);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, Vector3DReadOnly size)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException        if any of the size components is negative.
    * @throws ReferenceFrameMismatchException if arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, FrameVector3DReadOnly size)
   {
      pose.checkReferenceFrameMatch(size);
      setMatchingFrame(pose.getReferenceFrame(), pose, size);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose of this box. Not modified.
    * @param size           the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] size)
   {
      set(pose, size);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this box properties.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, RigidBodyTransformReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double[] size)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, size);
   }
}
