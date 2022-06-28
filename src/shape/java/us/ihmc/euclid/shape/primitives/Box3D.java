package us.ihmc.euclid.shape.primitives;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a box 3D.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Box3D implements Box3DBasics
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();
   /** Pose of this box. */
   private final Shape3DPose pose = new Shape3DPose();
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /**
    * Represents the sizeX, sizeY, and sizeZ of this box.
    */
   private final Vector3DBasics size = EuclidCoreFactories.newObservableVector3DBasics((axis, newValue) ->
   {
      checkSizePositive(axis);
      notifyChangeListeners();
   }, null);

   private BoxPolytope3D polytopeView = null;

   /**
    * Creates a 1-by-1-by-1 box 3D.
    */
   public Box3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new box 3D and initializes its size.
    *
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Box3D(double sizeX, double sizeY, double sizeZ)
   {
      getSize().set(sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its size.
    *
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public Box3D(Vector3DReadOnly size)
   {
      getSize().set(size);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param position    the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX       the size of this box along the x-axis.
    * @param sizeY       the size of this box along the y-axis.
    * @param sizeZ       the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Box3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param position    the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param size        the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public Box3D(Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      set(position, orientation, size);
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
   public Box3D(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose the position and orientation of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public Box3D(Pose3DReadOnly pose, Vector3DReadOnly size)
   {
      set(pose, size);
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
   public Box3D(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose the position and orientation of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public Box3D(RigidBodyTransformReadOnly pose, Vector3DReadOnly size)
   {
      set(pose, size);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    *
    * @param other the other box to copy. Not modified.
    */
   public Box3D(Box3DReadOnly other)
   {
      set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Shape3DPose getPose()
   {
      return pose;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DBasics getSize()
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

   @Override
   public Box3D copy()
   {
      return new Box3D(this);
   }

   @Override
   public BoxPolytope3DView asConvexPolytope()
   {
      if (polytopeView == null)
         polytopeView = new BoxPolytope3D(this);
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
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Box3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Box3DReadOnly)
         return Box3DBasics.super.equals((Box3DReadOnly) object);
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
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
