package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Implementation of a ramp 3D expressed in a given reference frame.
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
public class FrameRamp3D implements FrameRamp3DBasics, GeometryObject<FrameRamp3D>
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   /** The reference frame in which this shape is expressed. */
   private ReferenceFrame referenceFrame;
   /** Pose of this ramp. */
   private final FixedFrameShape3DPose pose = new FixedFrameShape3DPose(this);
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();
   /** Represents the sizeX, sizeY, and sizeZ of this ramp. */
   private final FixedFrameVector3DBasics size = EuclidFrameFactories.newObservableFixedFrameVector3DBasics(this, (axis, newValue) ->
   {
      checkSizePositive(axis);
      notifyChangeListeners();
   }, null);

   private boolean rampFeaturesDirty = true;

   /** Length of the slope face of this ramp. */
   private double rampLength;
   /**
    * Positive angle in [0, <i>pi</i>] representing the angle formed by the bottom face and the slope
    * face.
    */
   private double angleOfRampIncline;

   private boolean centroidDirty = true;

   private final FixedFramePoint3DBasics centroid = EuclidFrameFactories.newObservableFixedFramePoint3DBasics(this, null, axis -> updateCentroid());

   /**
    * Creates a new ramp 3D and initializes its length, width, and height to {@code 1.0} and
    * initializes its reference frame to {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameRamp3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new ramp 3D and initializes its length, width, and height to {@code 1.0} and
    * initializes its reference frame.
    *
    * @param referenceFrame this shape initial reference frame.
    */
   public FrameRamp3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 1.0, 1.0);
   }

   /**
    * Creates a new ramp 3D and initializes its size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param sizeX          the size of this ramp along the x-axis.
    * @param sizeY          the size of this ramp along the y-axis.
    * @param sizeZ          the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameRamp3D(ReferenceFrame referenceFrame, double sizeX, double sizeY, double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      getSize().set(sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param position       the position of this ramp. Not modified.
    * @param orientation    the orientation of this ramp. Not modified.
    * @param sizeX          the size of this ramp along the x-axis.
    * @param sizeY          the size of this ramp along the y-axis.
    * @param sizeZ          the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameRamp3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, position, orientation, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param position    the position of this ramp. Not modified.
    * @param orientation the orientation of this ramp. Not modified.
    * @param sizeX       the size of this ramp along the x-axis.
    * @param sizeY       the size of this ramp along the y-axis.
    * @param sizeZ       the size of this ramp along the z-axis.
    * @throws IllegalArgumentException        if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ}
    *                                         is negative.
    * @throws ReferenceFrameMismatchException if any of the frame arguments are not expressed in the
    *                                         same reference frame.
    */
   public FrameRamp3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(position, orientation, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation for this ramp. Not modified.
    * @param sizeX          the size of this ramp along the x-axis.
    * @param sizeY          the size of this ramp along the y-axis.
    * @param sizeZ          the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameRamp3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation for this ramp. Not modified.
    * @param sizeX the size of this ramp along the x-axis.
    * @param sizeY the size of this ramp along the y-axis.
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameRamp3D(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation for this ramp. Not modified.
    * @param sizeX          the size of this ramp along the x-axis.
    * @param sizeY          the size of this ramp along the y-axis.
    * @param sizeZ          the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameRamp3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation for this ramp. Not modified.
    * @param sizeX the size of this ramp along the x-axis.
    * @param sizeY the size of this ramp along the y-axis.
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameRamp3D(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D identical to {@code other}.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param other          the other ramp to copy. Not modified.
    */
   public FrameRamp3D(ReferenceFrame referenceFrame, Ramp3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D identical to {@code other}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   public FrameRamp3D(FrameRamp3DReadOnly other)
   {
      setIncludingFrame(other);
      setupListeners();
   }

   private void setupListeners()
   {
      changeListeners.add(() -> rampFeaturesDirty = true);
      changeListeners.add(() -> centroidDirty = true);
      pose.addChangeListeners(changeListeners);
   }

   private void updateRamp()
   {
      if (!rampFeaturesDirty)
         return;

      rampLength = EuclidShapeTools.computeRamp3DLength(size.getX(), size.getZ());
      angleOfRampIncline = EuclidShapeTools.computeRamp3DIncline(size.getX(), size.getZ());
      rampFeaturesDirty = false;
   }

   private void updateCentroid()
   {
      if (!centroidDirty)
         return;

      EuclidShapeTools.computeRamp3DCentroid(pose, size, centroid);
      centroidDirty = false;
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
   public FixedFrameVector3DBasics getSize()
   {
      return size;
   }

   @Override
   public FramePoint3DReadOnly getCentroid()
   {
      return centroid;
   }

   /**
    * Notifies the internal listeners that this shape has changed.
    */
   public void notifyChangeListeners()
   {
      for (int i = 0; i < changeListeners.size(); i++)
      {
         changeListeners.get(i).changed();
      }
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
   public void set(FrameRamp3D other)
   {
      FrameRamp3DBasics.super.set(other);
   }

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   @Override
   public double getRampLength()
   {
      updateRamp();
      return rampLength;
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   @Override
   public double getRampIncline()
   {
      updateRamp();
      return angleOfRampIncline;
   }

   /** {@inheritDoc} */
   @Override
   public FrameRamp3D copy()
   {
      return new FrameRamp3D(this);
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    * <p>
    * If the two ramp have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other ramp to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameRamp3D other, double epsilon)
   {
      return FrameRamp3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other   the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameRamp3D other, double epsilon)
   {
      return FrameRamp3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameRamp3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two ramp have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameRamp3DReadOnly)
         return FrameRamp3DBasics.super.equals((FrameRamp3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this ramp 3D.
    *
    * @return the hash code value for this ramp 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(pose, size);
   }

   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameRamp3DString(this);
   }
}
