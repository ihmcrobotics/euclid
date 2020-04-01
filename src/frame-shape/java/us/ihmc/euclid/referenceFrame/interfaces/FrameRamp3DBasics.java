package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read and write interface for a ramp 3D expressed in a changeable reference frame, i.e. the
 * reference frame in which this line is expressed can be changed.
 * <p>
 * A ramp represents a 3D shape with a triangular section in the XZ-plane. Shape description:
 * <ul>
 * <li>The slope face starts from {@code x=0.0}, {@code z=0.0} to end at {@code x=size.getX()},
 * {@code z=size.getZ()}.
 * <li>The bottom face is horizontal (XY-plane) at {@code z=0.0}.
 * <li>The rear face is vertical (YZ-plane) at {@code x=size.getX()}.
 * <li>The left face is vertical (XZ-plane) at {@code y=-size.getY()/2.0}.
 * <li>The right face is vertical (XZ-plane) at {@code y=size.getY()/2.0}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameRamp3DBasics extends FixedFrameRamp3DBasics, FrameShape3DBasics
{
   /**
    * Copies the {@code other} ramp data into {@code this} and updates its reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other ramp to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Ramp3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} ramp data into {@code this} and updates its reference frame.
    *
    * @param other the other ramp to copy. Not modified.
    */
   default void setIncludingFrame(FrameRamp3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this ramp properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this ramp. Not modified.
    * @param orientation    the orientation of this ramp. Not modified.
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
    * Sets this ramp properties and its reference frame.
    *
    * @param position    the position of this ramp. Not modified.
    * @param orientation the orientation of this ramp. Not modified.
    * @param sizeX       the size along the x-axis.
    * @param sizeY       the size along the y-axis.
    * @param sizeZ       the size along the z-axis.
    * @throws IllegalArgumentException        if any of the three size arguments is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments are not expressed in the
    *                                         same reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this ramp properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this ramp. Not modified.
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
    * Sets this ramp properties and its reference frame.
    *
    * @param pose  the pose of this ramp. Not modified.
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
    * Sets this ramp properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this ramp. Not modified.
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
    * Sets this ramp properties and its reference frame.
    *
    * @param pose  the pose of this ramp. Not modified.
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
    * Sets this ramp properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the pose of this ramp. Not modified.
    * @param size           the size of this ramp along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] size)
   {
      setReferenceFrame(referenceFrame);
      set(pose, size);
   }

   /**
    * Sets this ramp properties and its reference frame.
    *
    * @param pose the pose of this ramp. Not modified.
    * @param size the size of this ramp along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double[] size)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, size);
   }

   /** {@inheritDoc} */
   @Override
   FrameRamp3DBasics copy();
}
