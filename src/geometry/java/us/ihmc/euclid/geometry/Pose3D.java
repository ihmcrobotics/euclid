package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

/**
 * A {@code Pose3D} represents a position and orientation in 3 dimensions.
 */
public class Pose3D implements Pose3DBasics, GeometryObject<Pose3D>
{
   /** The position part of this pose 3D. */
   private final Point3D position = new Point3D();
   /** The orientation part of this pose 3D. */
   private final Quaternion orientation = new Quaternion();

   /**
    * Creates a new pose 3D initialized with its position at (0, 0, 0) and orientation set to the
    * neutral quaternion, i.e. zero rotation.
    */
   public Pose3D()
   {
   }

   /**
    * Creates a new pose 3D and initializes it with the given parameters.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param x     the x-coordinate of the position.
    * @param y     the y-coordinate of the position.
    * @param z     the z-coordinate of the position.
    * @param yaw   the first angle from the yaw-pitch-roll representation, represents a rotation about
    *              the z-axis.
    * @param pitch the second angle from the yaw-pitch-roll representation, represents a rotation about
    *              the y-axis.
    * @param roll  the third angle from the yaw-pitch-roll representation, represents a rotation about
    *              the x-axis.
    */
   public Pose3D(double x, double y, double z, double yaw, double pitch, double roll)
   {
      set(x, y, z, yaw, pitch, roll);
   }

   /**
    * Creates a new pose 3D and initializes to represent the same pose as the given
    * {@code pose2DReadOnly}.
    *
    * @param pose2DReadOnly the pose 2D used to initialize this pose 3D. Not modified.
    */
   public Pose3D(Pose2DReadOnly pose2DReadOnly)
   {
      set(pose2DReadOnly);
   }

   /**
    * Creates a new pose 3D and initializes it to {@code other}.
    *
    * @param other the other pose 3D used to initialize this. Not modified.
    */
   public Pose3D(Pose3DReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new pose 3D and initializes it with the given rigid-body transform.
    *
    * @param rigidBodyTransform the transform used to initialize this. Not modified.
    */
   public Pose3D(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new pose 3D and initializes it with the given parameters.
    *
    * @param position    tuple used to initialize the position part of this pose. Not modified.
    * @param orientation used to initialize the orientation part of this pose. Not modified.
    */
   public Pose3D(Tuple3DReadOnly position, Orientation3DReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   /** {@inheritDoc} */
   @Override
   public Point3D getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public Quaternion getOrientation()
   {
      return orientation;
   }

   /**
    * Sets this pose 3D to the {@code other} pose 3D.
    *
    * @param other the other pose 3D. Not modified.
    */
   @Override
   public void set(Pose3D other)
   {
      Pose3DBasics.super.set(other);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Pose3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Pose3DReadOnly)
         return equals((Pose3DReadOnly) object);
      else
         return false;
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Pose3D other, double epsilon)
   {
      return Pose3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically equal.
    * </p>
    *
    * @param other   the pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Pose3D other, double epsilon)
   {
      return Pose3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this pose 3D as follows:<br>
    * Pose 3D: position = (x, y, z), orientation = (qx, qy, qz, qs)
    *
    * @return the {@code String} representing this pose 3D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getPose3DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this pose 3D.
    *
    * @return the hash code value for this pose 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(position, orientation);
   }
}
