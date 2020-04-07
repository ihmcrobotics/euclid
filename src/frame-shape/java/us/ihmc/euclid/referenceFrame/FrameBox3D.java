package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoxPolytope3DView;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Implementation of a box 3D expressed in a given reference frame.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameBox3D implements FrameBox3DBasics, GeometryObject<FrameBox3D>
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   /** The reference frame in which this shape is expressed. */
   private ReferenceFrame referenceFrame;
   /** Pose of this box. */
   private final FixedFrameShape3DPose pose = new FixedFrameShape3DPose(this);
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();
   /**
    * Represents the sizeX, sizeY, and sizeZ of this box.
    */
   private final FixedFrameVector3DBasics size = EuclidFrameFactories.newObservableFixedFrameVector3DBasics(this, (axis, newValue) ->
   {
      checkSizePositive(axis);
      notifyChangeListeners();
   }, null);

   private FrameBoxPolytope3D polytopeView = null;

   /**
    * Creates a 1-by-1-by-1 box 3D and initializes its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameBox3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a 1-by-1-by-1 box 3D and initializes its reference frame.
    *
    * @param referenceFrame this shape initial reference frame.
    */
   public FrameBox3D(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 1.0, 1.0, 1.0);
   }

   /**
    * Creates a new box 3D and initializes its size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param sizeX          the size of this box along the x-axis.
    * @param sizeY          the size of this box along the y-axis.
    * @param sizeZ          the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameBox3D(ReferenceFrame referenceFrame, double sizeX, double sizeY, double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      getSize().set(sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param position       the position of this box. Not modified.
    * @param orientation    the orientation of this box. Not modified.
    * @param sizeX          the size of this box along the x-axis.
    * @param sizeY          the size of this box along the y-axis.
    * @param sizeZ          the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameBox3D(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param position    the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX       the size of this box along the x-axis.
    * @param sizeY       the size of this box along the y-axis.
    * @param sizeZ       the size of this box along the z-axis.
    * @throws IllegalArgumentException        if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ}
    *                                         is negative.
    * @throws ReferenceFrameMismatchException if the frame arguments are not expressed in the same
    *                                         reference frame.
    */
   public FrameBox3D(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation of this box. Not modified.
    * @param sizeX          the size of this box along the x-axis.
    * @param sizeY          the size of this box along the y-axis.
    * @param sizeZ          the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameBox3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameBox3D(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the position and orientation of this box. Not modified.
    * @param sizeX          the size of this box along the x-axis.
    * @param sizeY          the size of this box along the y-axis.
    * @param sizeZ          the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameBox3D(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(referenceFrame, pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public FrameBox3D(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param other          the other box to copy. Not modified.
    */
   public FrameBox3D(ReferenceFrame referenceFrame, Box3DReadOnly other)
   {
      setIncludingFrame(referenceFrame, other);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    *
    * @param other the other box to copy. Not modified.
    */
   public FrameBox3D(FrameBox3DReadOnly other)
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
   public FixedFrameVector3DBasics getSize()
   {
      return size;
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
   public void set(FrameBox3D other)
   {
      FrameBox3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public FrameBox3D copy()
   {
      return new FrameBox3D(this);
   }

   @Override
   public FrameBoxPolytope3DView asConvexPolytope()
   {
      if (polytopeView == null)
         polytopeView = new FrameBoxPolytope3D(this);
      return polytopeView;
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

   /**
    * Registers a list of listeners to be notified when this shape changes.
    *
    * @param listeners the listeners to register.
    */
   public void addChangeListeners(List<? extends Shape3DChangeListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         addChangeListener(listeners.get(i));
      }
   }

   /**
    * Registers a listener to be notified when this shape changes.
    *
    * @param listener the listener to register.
    */
   public void addChangeListener(Shape3DChangeListener listener)
   {
      changeListeners.add(listener);
      pose.addChangeListener(listener);
   }

   /**
    * Removes a previously registered listener.
    * <p>
    * This listener will no longer be notified of changes from this pose.
    * </p>
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed successful, {@code false} if the listener could
    *         not be found.
    */
   public boolean removeChangeListener(Shape3DChangeListener listener)
   {
      boolean hasBeenRemoved = changeListeners.remove(listener);
      hasBeenRemoved |= pose.removeChangeListener(listener);
      return hasBeenRemoved;
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    * <p>
    * If the two faces have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other box to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two boxes are equal component-wise and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameBox3D other, double epsilon)
   {
      return FrameBox3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two boxes are geometrically similar.
    *
    * @param other   the box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameBox3D other, double epsilon)
   {
      return FrameBox3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameBox3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two faces have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameBox3DReadOnly)
         return FrameBox3DBasics.super.equals((FrameBox3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this box 3D.
    *
    * @return the hash code value for this box 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(pose, size);
   }

   /**
    * Provides a {@code String} representation of this box 3D as follows:
    *
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameBox3DString(this);
   }
}
