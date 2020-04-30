package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of an ellipsoid 3D expressed in a given reference frame.
 * <p>
 * A ellipsoid 3D is represented by its radii, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameEllipsoid3D implements FrameEllipsoid3DBasics, GeometryObject<FrameEllipsoid3D>
{
   /** The reference frame in which this shape is expressed. */
   private ReferenceFrame referenceFrame;
   /** Pose of this ellipsoid. */
   private final FixedFrameShape3DPose pose = new FixedFrameShape3DPose(this);
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();
   /**
    * Represents the radiusX, radiusY, and radiusZ of this ellipsoid.
    */
   private final FixedFrameVector3DBasics radii = EuclidFrameFactories.newObservableFixedFrameVector3DBasics(this,
                                                                                                             (axis, newValue) -> checkRadiusPositive(axis),
                                                                                                             null);

   /**
    * Creates a new ellipsoid 3D with its 3 radii initialized to {@code 1} and initializes its
    * reference frame to {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameEllipsoid3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new ellipsoid 3D with its 3 radii initialized to {@code 1} and initializes its
    * reference frame.
    *
    * @param referenceFrame this shape initial reference frame.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 1.0, 1.0);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its radii.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, double radiusX, double radiusY, double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its radii.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Vector3DReadOnly radii)
   {
      setReferenceFrame(referenceFrame);
      getRadii().set(radii);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param position       the position of this ellipsoid. Not modified.
    * @param orientation    the orientation of this ellipsoid. Not modified.
    * @param radiusX        the size of the ellipsoid along the x-axis.
    * @param radiusY        the size of the ellipsoid along the y-axis.
    * @param radiusZ        the size of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY,
                           double radiusZ)
   {
      setIncludingFrame(referenceFrame, position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param position       the position of this ellipsoid. Not modified.
    * @param orientation    the orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      setIncludingFrame(referenceFrame, position, orientation, radii);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param position    the position of this ellipsoid. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radiusX     the size of the ellipsoid along the x-axis.
    * @param radiusY     the size of the ellipsoid along the y-axis.
    * @param radiusZ     the size of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException        if any of the three radii is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   public FrameEllipsoid3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param position    the position of this ellipsoid. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radii       the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   public FrameEllipsoid3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      setIncludingFrame(position, orientation, radii);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param position    the position of this ellipsoid. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radii       the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   public FrameEllipsoid3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, FrameVector3DReadOnly radii)
   {
      setIncludingFrame(position, orientation, radii);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(referenceFrame, pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose, Vector3DReadOnly radii)
   {
      setIncludingFrame(referenceFrame, pose, radii);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public FrameEllipsoid3D(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public FrameEllipsoid3D(FramePose3DReadOnly pose, Vector3DReadOnly radii)
   {
      setIncludingFrame(pose, radii);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   public FrameEllipsoid3D(FramePose3DReadOnly pose, FrameVector3DReadOnly radii)
   {
      setIncludingFrame(pose, radii);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radiusX        radius of the ellipsoid along the x-axis.
    * @param radiusY        radius of the ellipsoid along the y-axis.
    * @param radiusZ        radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(referenceFrame, pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation of this ellipsoid. Not modified.
    * @param radii          the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, Vector3DReadOnly radii)
   {
      setIncludingFrame(referenceFrame, pose, radii);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public FrameEllipsoid3D(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public FrameEllipsoid3D(FrameShape3DPoseReadOnly pose, Vector3DReadOnly radii)
   {
      setIncludingFrame(pose, radii);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException        if any of the radii components is negative.
    * @throws ReferenceFrameMismatchException if arguments are not expressed in the same reference
    *                                         frame.
    */
   public FrameEllipsoid3D(FrameShape3DPoseReadOnly pose, FrameVector3DReadOnly radii)
   {
      setIncludingFrame(pose, radii);
   }

   /**
    * Creates a new ellipsoid 3D identical to {@code other}.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param other          the other ellipsoid to copy. Not modified.
    */
   public FrameEllipsoid3D(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   /**
    * Creates a new ellipsoid 3D identical to {@code other}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   public FrameEllipsoid3D(FrameEllipsoid3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameShape3DPoseBasics getPose()
   {
      return pose;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getRadii()
   {
      return radii;
   }

   /** {@inheritDoc} */
   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   /** {@inheritDoc} */
   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      supplier = newSupplier;
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameEllipsoid3D other)
   {
      FrameEllipsoid3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public FrameEllipsoid3D copy()
   {
      return new FrameEllipsoid3D(this);
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    * <p>
    * If the two ellipsoids have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other ellipsoid to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ellipsoids are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameEllipsoid3D other, double epsilon)
   {
      return FrameEllipsoid3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ellipsoids are geometrically
    * similar.
    *
    * @param other   the ellipsoid to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ellipsoids represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameEllipsoid3D other, double epsilon)
   {
      return FrameEllipsoid3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameEllipsoid3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two ellipsoids have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameEllipsoid3DReadOnly)
         return FrameEllipsoid3DBasics.super.equals((FrameEllipsoid3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this ellipsoid 3D.
    *
    * @return the hash code value for this ellipsoid 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(pose, radii);
   }

   /**
    * Provides a {@code String} representation of this ellipsoid 3D as follows:
    *
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this ellipsoid 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameEllipsoid3DString(this);
   }
}
