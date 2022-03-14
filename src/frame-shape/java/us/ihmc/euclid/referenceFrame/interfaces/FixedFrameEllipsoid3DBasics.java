package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for ellipsoid 3D expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * A ellipsoid 3D is represented by its radii, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameEllipsoid3DBasics extends Ellipsoid3DBasics, FrameEllipsoid3DReadOnly, FixedFrameShape3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFrameVector3DBasics getRadii();

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
    * Copies the {@code other} ellipsoid data into {@code this}.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other ellipsoid to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameEllipsoid3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this ellipsoid properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this ellipsoid center. Not modified.
    * @param orientation    the orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets this ellipsoid properties.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this ellipsoid center. Not modified.
    * @param orientation    the orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(position, orientation, radii);
   }

   /**
    * Sets this ellipsoid properties.
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radiusX     radius of the ellipsoid along the x-axis.
    * @param radiusY     radius of the ellipsoid along the y-axis.
    * @param radiusZ     radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      position.checkReferenceFrameMatch(orientation);
      set(position.getReferenceFrame(), position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets this ellipsoid properties.
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radii       the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      position.checkReferenceFrameMatch(orientation);
      set(position.getReferenceFrame(), position, orientation, radii);
   }

   /**
    * Sets this ellipsoid properties.
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radii       the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly radii)
   {
      position.checkReferenceFrameMatch(orientation, radii);
      set(position.getReferenceFrame(), position, orientation, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose, Vector3DReadOnly radii)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePose3DReadOnly pose, Vector3DReadOnly radii)
   {
      set(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FramePose3DReadOnly pose, FrameVector3DReadOnly radii)
   {
      pose.checkReferenceFrameMatch(radii);
      set(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, Vector3DReadOnly radii)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, Vector3DReadOnly radii)
   {
      set(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, FrameVector3DReadOnly radii)
   {
      pose.checkReferenceFrameMatch(radii);
      set(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of this ellipsoid along the x, y, and z axes in order. Not
    *                       modified.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] radii)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of this ellipsoid along the x, y, and z axes in order. Not modified.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameShape3DPoseReadOnly pose, double[] radii)
   {
      set(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(ReferenceFrame, Ellipsoid3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other ellipsoid to copy. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    * <p>
    * If {@code other} is expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameEllipsoid3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   default void setMatchingFrame(FrameEllipsoid3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this ellipsoid properties.
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
    * @param position       the position of this ellipsoid center. Not modified.
    * @param orientation    the orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame,
                                 Point3DReadOnly position,
                                 Orientation3DReadOnly orientation,
                                 double radiusX,
                                 double radiusY,
                                 double radiusZ)
   {
      set(position, orientation, radiusX, radiusY, radiusZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this ellipsoid properties.
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
    * @param position       the position of this ellipsoid center. Not modified.
    * @param orientation    the orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      set(position, orientation, radii);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this ellipsoid properties.
    * <p>
    * If arguments are expressed in the same frame as {@code this}, then this method is equivalent to
    * {@link #set(FramePoint3DReadOnly, FrameOrientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radiusX     radius of the ellipsoid along the x-axis.
    * @param radiusY     radius of the ellipsoid along the y-axis.
    * @param radiusZ     radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets this ellipsoid properties.
    * <p>
    * If arguments are expressed in the same frame as {@code this}, then this method is equivalent to
    * {@link #set(FramePoint3DReadOnly, FrameOrientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radii       the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      position.checkReferenceFrameMatch(orientation);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, radii);
   }

   /**
    * Sets this ellipsoid properties.
    * <p>
    * If arguments are expressed in the same frame as {@code this}, then this method is equivalent to
    * {@link #set(FramePoint3DReadOnly, FrameOrientation3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radii       the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly radii)
   {
      position.checkReferenceFrameMatch(orientation, radii);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
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
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
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
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, Vector3DReadOnly radii)
   {
      set(pose, radii);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FramePose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setMatchingFrame(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FramePose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   default void setMatchingFrame(FramePose3DReadOnly pose, Vector3DReadOnly radii)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FramePose3DReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setMatchingFrame(FramePose3DReadOnly pose, FrameVector3DReadOnly radii)
   {
      pose.checkReferenceFrameMatch(radii);
      setMatchingFrame(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
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
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
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
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, Vector3DReadOnly radii)
   {
      set(pose, radii);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameShape3DPoseReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameShape3DPoseReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, Vector3DReadOnly radii)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameShape3DPoseReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, FrameVector3DReadOnly radii)
   {
      pose.checkReferenceFrameMatch(radii);
      setMatchingFrame(pose.getReferenceFrame(), pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
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
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of this ellipsoid along the x, y, and z axes in order. Not
    *                       modified.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] radii)
   {
      set(pose, radii);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * <p>
    * If the arguments are expressed in the same frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameShape3DPoseReadOnly, double, double, double)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * with the arguments and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of this ellipsoid along the x, y, and z axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double[] radii)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radii);
   }
}
