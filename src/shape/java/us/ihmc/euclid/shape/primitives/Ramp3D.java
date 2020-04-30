package us.ihmc.euclid.shape.primitives;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.RampPolytope3DView;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a ramp 3D.
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
public class Ramp3D implements Ramp3DBasics, GeometryObject<Ramp3D>
{
   private final List<Shape3DChangeListener> changeListeners = new ArrayList<>();

   /** Pose of this ramp. */
   private final Shape3DPose pose = new Shape3DPose();
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /** Size of this ramp's bounding box. */
   private final Vector3DBasics size = EuclidCoreFactories.newObservableVector3DBasics((axis, newValue) ->
   {
      checkSizePositive(axis);
      notifyChangeListeners();
   }, null);

   private boolean rampSurfaceNormalDirty = true;
   private final Vector3DBasics rampSurfaceNormal = EuclidCoreFactories.newObservableVector3DBasics(null, axis -> updateRampSurfaceNormal());

   private boolean rampFeaturesDirty = true;
   /** Length of the slope face of this ramp. */
   private double rampLength;
   /**
    * Positive angle in [0, <i>pi</i>] representing the angle formed by the bottom face and the slope
    * face.
    */
   private double angleOfRampIncline;

   private boolean centroidDirty = true;

   private final Point3DBasics centroid = EuclidCoreFactories.newObservablePoint3DBasics(null, axis -> updateCentroid());

   private RampPolytope3D polytopeView = null;

   /**
    * Creates a new ramp 3D and initializes its length, width, and height to {@code 1.0}.
    */
   public Ramp3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new ramp 3D and initializes its size.
    *
    * @param sizeX the size of this ramp along the x-axis.
    * @param sizeY the size of this ramp along the y-axis.
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Ramp3D(double sizeX, double sizeY, double sizeZ)
   {
      getSize().set(sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its size.
    *
    * @param size the size of this ramp. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   public Ramp3D(Vector3DReadOnly size)
   {
      getSize().set(size);
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
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public Ramp3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      set(position, orientation, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param position    the position of this ramp. Not modified.
    * @param orientation the orientation of this ramp. Not modified.
    * @param size        the size of this ramp. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   public Ramp3D(Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      set(position, orientation, size);
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
   public Ramp3D(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose the position and orientation for this ramp. Not modified.
    * @param size the size of this ramp. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   public Ramp3D(RigidBodyTransformReadOnly pose, Vector3DReadOnly size)
   {
      set(pose, size);
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
   public Ramp3D(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D and initializes its pose and size.
    *
    * @param pose the position and orientation for this ramp. Not modified.
    * @param size the size of this ramp. Not modified.
    * @throws IllegalArgumentException if any of the size components is negative.
    */
   public Ramp3D(Pose3DReadOnly pose, Vector3DReadOnly size)
   {
      set(pose, size);
      setupListeners();
   }

   /**
    * Creates a new ramp 3D identical to {@code other}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   public Ramp3D(Ramp3DReadOnly other)
   {
      set(other);
      setupListeners();
   }

   private void setupListeners()
   {
      changeListeners.add(() ->
      {
         rampSurfaceNormalDirty = true;
         rampFeaturesDirty = true;
         centroidDirty = true;
      });
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

   private void updateRampSurfaceNormal()
   {
      if (!rampSurfaceNormalDirty)
         return;

      rampSurfaceNormal.set(-getSizeZ() / getRampLength(), 0.0, getSizeX() / getRampLength());
      transformToWorld(rampSurfaceNormal);
      rampSurfaceNormalDirty = false;
   }

   private void updateCentroid()
   {
      if (!centroidDirty)
         return;

      EuclidShapeTools.computeRamp3DCentroid(pose, size, centroid);
      centroidDirty = false;
   }

   /**
    * Copies the {@code other} ramp data into {@code this}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   @Override
   public void set(Ramp3D other)
   {
      Ramp3DBasics.super.set(other);
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
   public Point3DReadOnly getCentroid()
   {
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getRampSurfaceNormal()
   {
      return rampSurfaceNormal;
   }

   /** {@inheritDoc} */
   @Override
   public void getRampSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(rampSurfaceNormal);
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

   @Override
   public Ramp3D copy()
   {
      return new Ramp3D(this);
   }

   @Override
   public RampPolytope3DView asConvexPolytope()
   {
      if (polytopeView == null)
         polytopeView = new RampPolytope3D(this);
      return polytopeView;
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other ramp to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Ramp3D other, double epsilon)
   {
      return Ramp3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other   the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Ramp3D other, double epsilon)
   {
      return Ramp3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Ramp3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Ramp3DReadOnly)
         return Ramp3DBasics.super.equals((Ramp3DReadOnly) object);
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

   /**
    * Provides a {@code String} representation of this ramp 3D as follows:<br>
    * Ramp 3D: [position: ( 0.540, 0.110, 0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: (
    * 0.191, 0.719, 0.479 )]
    *
    * @return the {@code String} representing this ramp 3D.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getRamp3DString(this);
   }
}
