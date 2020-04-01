package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read and write interface for ellipsoid 3D expressed in a changeable reference frame, i.e. the
 * reference frame in which this line is expressed can be changed.
 * <p>
 * A ellipsoid 3D is represented by its radii, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameEllipsoid3DBasics extends FixedFrameEllipsoid3DBasics, FrameShape3DBasics
{
   /**
    * Copies the {@code other} ellipsoid data into {@code this} and updates its reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other ellipsoid to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this} and updates its reference frame.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   default void setIncludingFrame(FrameEllipsoid3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this ellipsoid properties and its reference frame.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param position       the position of this ellipsoid center. Not modified.
    * @param orientation    the orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY,
                                  double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets this ellipsoid properties and its reference frame.
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radiusX     radius of the ellipsoid along the x-axis.
    * @param radiusY     radius of the ellipsoid along the y-axis.
    * @param radiusZ     radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid and its reference frame.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid and its reference frame.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid and its reference frame.
    *
    * @param referenceFrame the reference frame in which the pose argument is expressed.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of this ellipsoid along the x, y, and z axes in order. Not
    *                       modified.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] radii)
   {
      setReferenceFrame(referenceFrame);
      set(pose, radii);
   }

   /**
    * Sets the pose and radii of this ellipsoid and its reference frame.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of this ellipsoid along the x, y, and z axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double[] radii)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, radii);
   }

   /** {@inheritDoc} */
   @Override
   FrameEllipsoid3DBasics copy();
}
