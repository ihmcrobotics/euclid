package us.ihmc.euclid.referenceFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DPoseBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Implementation of a shape pose 3D expressed in a given reference frame.
 * <p>
 * This object does not manage its reference frame.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FixedFrameShape3DPose implements FixedFrameShape3DPoseBasics, Settable<FixedFrameShape3DPose>
{
   /** The listeners to be notified when this pose changes. */
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   /**
    * This object does not manage its reference frame, this field is the owner of this pose and manages
    * the current reference frame.
    */
   private final ReferenceFrameHolder referenceFrameHolder;
   /** The orientation part. */
   private final FixedFrameRotationMatrixBasics shapeOrientation = EuclidFrameFactories.newObservableFixedFrameRotationMatrixBasics(this,
                                                                                                                                    this::notifyChangeListeners,
                                                                                                                                    null);
   /** The position part. */
   private final FixedFramePoint3DBasics shapePosition = EuclidFrameFactories.newObservableFixedFramePoint3DBasics(this,
                                                                                                                   (axis, newValue) -> notifyChangeListeners(),
                                                                                                                   null);

   /** Vector linked to the components of the x-axis unit-vector. */
   private final FrameVector3DReadOnly xAxis;
   /** Vector linked to the components of the y-axis unit-vector. */
   private final FrameVector3DReadOnly yAxis;
   /** Vector linked to the components of the z-axis unit-vector. */
   private final FrameVector3DReadOnly zAxis;

   /**
    * Creates a new shape pose which both position and orientation are initialized to zero.
    *
    * @param referenceFrameHolder the owner of this pose which manages its reference frame. Reference
    *                             saved.
    */
   public FixedFrameShape3DPose(ReferenceFrameHolder referenceFrameHolder)
   {
      this.referenceFrameHolder = referenceFrameHolder;

      xAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this, shapeOrientation::getM00, shapeOrientation::getM10, shapeOrientation::getM20);
      yAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this, shapeOrientation::getM01, shapeOrientation::getM11, shapeOrientation::getM21);
      zAxis = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(this, shapeOrientation::getM02, shapeOrientation::getM12, shapeOrientation::getM22);
   }

   /**
    * Creates a new shape pose and initializes it to the given transform.
    *
    * @param referenceFrameHolder the owner of this pose which manages its reference frame. Reference
    *                             saved.
    * @param rigidBodyTransform   the transform to initialize this shape pose. Not modified.
    */
   public FixedFrameShape3DPose(ReferenceFrameHolder referenceFrameHolder, RigidBodyTransformReadOnly rigidBodyTransform)
   {
      this(referenceFrameHolder);
      set(rigidBodyTransform);
   }

   /**
    * Creates a new shape pose and initializes it to the given pose.
    *
    * @param referenceFrameHolder the owner of this pose which manages its reference frame. Reference
    *                             saved.
    * @param pose                 the pose to initialize this shape pose. Not modified.
    */
   public FixedFrameShape3DPose(ReferenceFrameHolder referenceFrameHolder, Pose3DReadOnly pose)
   {
      this(referenceFrameHolder);
      set(pose);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FixedFrameShape3DPose other)
   {
      FixedFrameShape3DPoseBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrameHolder.getReferenceFrame();
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
   public void addChangeListeners(List<? extends Shape3DChangeListener> listeners)
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
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidFrameGeometry)}, it returns {@code false} otherwise.
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
         return equals((EuclidFrameGeometry) object);
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
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
