package us.ihmc.euclid.shape.primitives;

import static us.ihmc.euclid.tools.EuclidCoreFactories.newLinkedVector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a shape pose 3D.
 *
 * @author Sylvain Bertrand
 */
public class Shape3DPose implements Shape3DPoseBasics, Settable<Shape3DPose>
{
   /** The listeners to be notified when this pose changes. */
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   /** The orientation part. */
   private final RotationMatrixBasics shapeOrientation = EuclidCoreFactories.newObservableRotationMatrixBasics(this::notifyChangeListeners, null);
   /** The position part. */
   private final Point3DBasics shapePosition = EuclidCoreFactories.newObservablePoint3DBasics((axis, newValue) -> notifyChangeListeners(), null);

   /** Vector linked to the components of the x-axis unit-vector. */
   private final Vector3DReadOnly xAxis = newLinkedVector3DReadOnly(shapeOrientation::getM00, shapeOrientation::getM10, shapeOrientation::getM20);
   /** Vector linked to the components of the y-axis unit-vector. */
   private final Vector3DReadOnly yAxis = newLinkedVector3DReadOnly(shapeOrientation::getM01, shapeOrientation::getM11, shapeOrientation::getM21);
   /** Vector linked to the components of the z-axis unit-vector. */
   private final Vector3DReadOnly zAxis = newLinkedVector3DReadOnly(shapeOrientation::getM02, shapeOrientation::getM12, shapeOrientation::getM22);

   /**
    * Creates a new shape pose which both position and orientation are initialized to zero.
    */
   public Shape3DPose()
   {
      setToZero();
   }

   /**
    * Creates a new shape pose and initializes it to the given transform.
    *
    * @param rigidBodyTransform the transform to initialize this shape pose. Not modified.
    */
   public Shape3DPose(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new shape pose and initializes it to the given pose.
    *
    * @param pose the pose to initialize this shape pose. Not modified.
    */
   public Shape3DPose(Pose3DReadOnly pose)
   {
      set(pose);
   }

   /** {@inheritDoc} */
   @Override
   public void set(Shape3DPose other)
   {
      Shape3DPoseBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public RotationMatrixBasics getShapeOrientation()
   {
      return shapeOrientation;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getShapePosition()
   {
      return shapePosition;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getXAxis()
   {
      return xAxis;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getYAxis()
   {
      return yAxis;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getZAxis()
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
    * {@link #equals(Shape3DPoseReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Shape3DPoseReadOnly)
         return Shape3DPoseBasics.super.equals((Shape3DPoseReadOnly) object);
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
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)]
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
