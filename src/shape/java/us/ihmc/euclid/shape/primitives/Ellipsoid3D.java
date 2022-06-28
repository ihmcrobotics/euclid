package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of an ellipsoid 3D.
 * <p>
 * A ellipsoid 3D is represented by its radii, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Ellipsoid3D implements Ellipsoid3DBasics
{
   /** Pose of this ellipsoid. */
   private final Shape3DPose pose = new Shape3DPose();
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /** The three radii of this ellipsoid. */
   private final Vector3DBasics radii = EuclidCoreFactories.newObservableVector3DBasics((axis, newValue) -> checkRadiusPositive(axis), null);

   /**
    * Creates a new ellipsoid 3D with its 3 radii initialized to {@code 1}.
    */
   public Ellipsoid3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its radii.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(double radiusX, double radiusY, double radiusZ)
   {
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its radii.
    *
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public Ellipsoid3D(Vector3DReadOnly radii)
   {
      getRadii().set(radii);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param position    the position of this ellipsoid. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radiusX     the size of the ellipsoid along the x-axis.
    * @param radiusY     the size of the ellipsoid along the y-axis.
    * @param radiusZ     the size of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      set(position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param position    the position of this ellipsoid. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radii       the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public Ellipsoid3D(Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      set(position, orientation, radii);
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
   public Ellipsoid3D(Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public Ellipsoid3D(Pose3DReadOnly pose, Vector3DReadOnly radii)
   {
      set(pose, radii);
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
   public Ellipsoid3D(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of the ellipsoid. Not modified.
    * @throws IllegalArgumentException if any of the radii components is negative.
    */
   public Ellipsoid3D(RigidBodyTransformReadOnly pose, Vector3DReadOnly radii)
   {
      set(pose, radii);
   }

   /**
    * Creates a new ellipsoid 3D identical to {@code other}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   public Ellipsoid3D(Ellipsoid3DReadOnly other)
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
   public Vector3DBasics getRadii()
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
   public Ellipsoid3D copy()
   {
      return new Ellipsoid3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Ellipsoid3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Ellipsoid3DReadOnly)
         return Ellipsoid3DBasics.super.equals((Ellipsoid3DReadOnly) object);
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
      long hash = EuclidHashCodeTools.combineHashCode(pose.hashCode(), radii.hashCode());
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this ellipsoid 3D as follows:
    *
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )]
    * </pre>
    *
    * @return the {@code String} representing this ellipsoid 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
