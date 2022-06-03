package us.ihmc.euclid.tuple4D;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Class used to represent unit-quaternions which are used to represent 3D orientations.
 * <p>
 * This version of quaternion uses double precision fields to save the value of each component. It
 * is meant for garbage free usage.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Quaternion implements QuaternionBasics
{
   /** The x-component. */
   private double x;
   /** The y-component. */
   private double y;
   /** The z-component. */
   private double z;
   /** The s-component. */
   private double s;

   /**
    * Creates a new quaternion and initializes it to the neutral quaternion which represents a 'zero'
    * rotation.
    */
   public Quaternion()
   {
      setToZero();
   }

   /**
    * Creates a new quaternion and initializes it with the given components.
    * <p>
    * The quaternion is immediately normalized.
    * </p>
    *
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    * @param s the s-component.
    */
   public Quaternion(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   /**
    * Creates a new quaternion and initializes its component {@code x}, {@code y}, {@code z}, {@code s}
    * in order from the given array.
    * <p>
    * The quaternion is immediately normalized.
    * </p>
    *
    * @param quaternionArray the array containing this quaternion's components. Not modified.
    */
   public Quaternion(double[] quaternionArray)
   {
      set(quaternionArray);
   }

   /**
    * Creates a new quaternion and initializes its component {@code x}, {@code y}, {@code z}, {@code s}
    * in order from the given matrix.
    * <p>
    * The quaternion is immediately normalized.
    * </p>
    *
    * @param matrix the matrix containing this quaternion's components. Not modified.
    */
   public Quaternion(DMatrix matrix)
   {
      set(matrix);
   }

   /**
    * Creates a new quaternion and initializes such that it represents the same orientation as the
    * given one.
    *
    * @param orientation the orientation to initialize this quaternion. Not modified.
    */
   public Quaternion(Orientation3DReadOnly orientation)
   {
      set(orientation);
   }

   /**
    * Creates a new quaternion and initializes it to {@code other}.
    *
    * @param other the quaternion to copy the components from. Not modified.
    */
   public Quaternion(QuaternionReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new quaternion and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components from. Not modified.
    */
   public Quaternion(Tuple4DReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new quaternion and initializes such that it represents the same orientation as the
    * given {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector to initialize this quaternion. Not modified.
    */
   public Quaternion(Vector3DReadOnly rotationVector)
   {
      setRotationVector(rotationVector);
   }

   /**
    * Creates and new quaternion and initializes such that it represents the same orientation as the
    * given yaw-pitch-roll {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   public Quaternion(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll);
   }

   /** {@inheritDoc} */
   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      x = qx;
      y = qy;
      z = qz;
      s = qs;
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y;
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z;
   }

   /** {@inheritDoc} */
   @Override
   public double getS()
   {
      return s;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Tuple4DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Tuple4DReadOnly)
         return equals((Tuple4DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this quaternion as follows: (x, y, z, s).
    *
    * @return the {@code String} representing this quaternion.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this quaternion.
    *
    * @return the hash code value for this quaternion.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(x, y, z, s);
   }
}
