package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Implementation of a shape pose 3D expressed in a given reference frame.
 *
 * @author Sylvain Bertrand
 */
public class FrameShape3DPose implements FrameShape3DPoseBasics, GeometryObject<FrameShape3DPose>
{
   /** The listeners to be notified when this pose changes. */
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   /** The reference frame in which this pose is expressed. */
   private ReferenceFrame referenceFrame;
   /** The orientation part. */
   private final FixedFrameRotationMatrixBasics shapeOrientation = EuclidFrameFactories.newObservableFixedFrameRotationMatrixBasics(this,
                                                                                                                                    this::notifyChangeListeners,
                                                                                                                                    null);
   /** The position part. */
   private final FixedFramePoint3DBasics shapePosition = EuclidFrameFactories.newObservableFixedFramePoint3DBasics(this,
                                                                                                                   (axis, newValue) -> notifyChangeListeners(),
                                                                                                                   null);

   /** Vector linked to the components of the x-axis unit-vector. */
   private final FrameVector3DReadOnly xAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this,
                                                                                                   shapeOrientation::getM00,
                                                                                                   shapeOrientation::getM10,
                                                                                                   shapeOrientation::getM20);
   /** Vector linked to the components of the y-axis unit-vector. */
   private final FrameVector3DReadOnly yAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this,
                                                                                                   shapeOrientation::getM01,
                                                                                                   shapeOrientation::getM11,
                                                                                                   shapeOrientation::getM21);
   /** Vector linked to the components of the z-axis unit-vector. */
   private final FrameVector3DReadOnly zAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this,
                                                                                                   shapeOrientation::getM02,
                                                                                                   shapeOrientation::getM12,
                                                                                                   shapeOrientation::getM22);

   /**
    * Creates a new shape pose which both position and orientation are initialized to zero and
    * initializes its reference frame to {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameShape3DPose()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new shape pose which both position and orientation are initialized to zero and
    * initializes its reference frame.
    *
    * @param referenceFrame this shape initial reference frame.
    */
   public FrameShape3DPose(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new shape pose and initializes it to the given pose.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the pose to initialize this shape pose. Not modified.
    */
   public FrameShape3DPose(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      setIncludingFrame(referenceFrame, pose);
   }

   /**
    * Creates a new shape pose and initializes it to the given pose.
    *
    * @param pose the pose to initialize this shape pose. Not modified.
    */
   public FrameShape3DPose(FramePose3DReadOnly pose)
   {
      setIncludingFrame(pose);
   }

   /**
    * Creates a new shape pose and initializes it to the given transform.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param pose           the transform to initialize this shape pose. Not modified.
    */
   public FrameShape3DPose(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      setIncludingFrame(referenceFrame, pose);
   }

   /**
    * Creates a new shape pose and initializes it to the given pose.
    *
    * @param pose the pose to initialize this shape pose. Not modified.
    */
   public FrameShape3DPose(FrameShape3DPoseReadOnly pose)
   {
      setIncludingFrame(pose);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FrameShape3DPose other)
   {
      FrameShape3DPoseBasics.super.set(other);
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
   public FixedFrameRotationMatrixBasics getShapeOrientation()
   {
      return shapeOrientation;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getShapePosition()
   {
      return shapePosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getXAxis()
   {
      return xAxis;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getYAxis()
   {
      return yAxis;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getZAxis()
   {
      return zAxis;
   }

   /**
    * Notifies the listeners registered that this pose has changed.
    */
   public void notifyChangeListeners()
   {
      for (int i = 0; i < changeListeners.size(); i++)
         changeListeners.get(i).changed();
   }

   /**
    * Registers a list of listeners to be notified when this pose changes.
    *
    * @param listeners the listeners to register.
    */
   public void addChangeListeners(List<Shape3DChangeListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         addChangeListener(listeners.get(i));
      }
   }

   /**
    * Registers a listener to be notified when this pose changes.
    *
    * @param listener the listener to register.
    */
   public void addChangeListener(Shape3DChangeListener listener)
   {
      changeListeners.add(listener);
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
      return changeListeners.remove(listener);
   }

   /**
    * Tests on a per-component basis if this shape pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two shape poses are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameShape3DPose other, double epsilon)
   {
      return FrameShape3DPoseBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two shape poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically equal.
    * </p>
    *
    * @param other   the shape pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two shape poses represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FrameShape3DPose other, double epsilon)
   {
      return FrameShape3DPoseBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameShape3DPoseReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameShape3DPoseReadOnly)
         return FrameShape3DPoseBasics.super.equals((FrameShape3DPoseReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this pose 3D.
    *
    * @return the hash code value for this pose 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(shapePosition, shapeOrientation);
   }

   /**
    * Provides a {@code String} representation of this pose 3D as follows:
    *
    * <pre>
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)] - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this pose 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameShape3DPoseString(this);
   }
}
