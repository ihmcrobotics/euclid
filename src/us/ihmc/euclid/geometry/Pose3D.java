package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

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
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    * @param yaw the first angle from the yaw-pitch-roll representation, represents a rotation about
    *           the z-axis.
    * @param pitch the second angle from the yaw-pitch-roll representation, represents a rotation
    *           about the y-axis.
    * @param roll the third angle from the yaw-pitch-roll representation, represents a rotation
    *           about the x-axis.
    */
   public Pose3D(double x, double y, double z, double yaw, double pitch, double roll)
   {
      set(x, y, z, yaw, pitch, roll);
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
   public Pose3D(RigidBodyTransform rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new pose 3D and initializes it with the given parameters.
    *
    * @param position tuple used to initialize the position part of this pose. Not modified.
    * @param orientation used to initialize the orientation part of this pose. Not modified.
    */
   public Pose3D(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.orientation.set(orientation);
      this.position.set(position);
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return position.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return position.getY();
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return position.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   /** {@inheritDoc} */
   @Override
   public double getPitch()
   {
      return orientation.getPitch();
   }

   /** {@inheritDoc} */
   @Override
   public double getRoll()
   {
      return orientation.getRoll();
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

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      position.setX(x);
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      position.setY(y);
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      position.setZ(z);
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
    * Tests on a per component basis, if this pose 3D is exactly equal to {@code other}.
    *
    * @param other the other pose 3D to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Pose3D other)
   {
      if (other == null)
         return false;
      else
         return position.equals(other.position) && orientation.equals(other.orientation);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Pose3D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Pose3D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Pose3D other, double epsilon)
   {
      return Pose3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically
    * equal.
    * </p>
    *
    * @param other the pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Pose3D other, double epsilon)
   {
      return Pose3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of the position part of this pose 3D as follows:<br>
    * (x, y, z)
    *
    * @return the {@code String} representing the position part of this pose 3D.
    */
   public String printOutPosition()
   {
      return position.toString();
   }

   /**
    * Provides a {@code String} representation of the orientation part of this pose 3D as
    * follows:<br>
    * (qx, qy, qz, qs)
    *
    * @return the {@code String} representing the orientation part of this pose 3D.
    */
   public String printOutOrientation()
   {
      return orientation.toString();
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
}
