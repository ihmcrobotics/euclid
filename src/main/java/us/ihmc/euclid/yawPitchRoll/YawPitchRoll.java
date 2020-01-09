package us.ihmc.euclid.yawPitchRoll;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * A yaw-pitch-roll is used to represent a 3D orientation by three successive rotations: rotation
 * around the z-axis (yaw), then around the y-axis (pitch), and then around the x-axis (roll). The
 * three components yaw, pitch, and roll represents the angle for rotation expressed in radians.
 * <p>
 * In general, yaw-pitch-roll representation is considered one of the most intuitive way of
 * interpreting an orientation and is thus commonly used as an interface between human and machine.
 * However, there is no algebra directly accessible for manipulating orientations represented as
 * yaw-pitch-roll making it highly computationally expensive when compared to rotation matrices or
 * quaternions. In addition, yaw-pitch-roll representation is sensitive to gimbal lock which happens
 * when the pitch angle is in the neighborhood of either <i>pi/2</i> or -<i>pi/2</i>. When close to
 * such configuration, converting orientation to yaw-pitch-roll becomes inaccurate and can sometimes
 * lead to unexpected results.
 * </p>
 * <p>
 * Equivalent representation of yaw-pitch-roll as 3-by-3 rotation matrix:
 *
 * <pre>
 *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
 * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
 *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
 * </pre>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class YawPitchRoll implements YawPitchRollBasics, GeometryObject<YawPitchRoll>
{
   /** The yaw angle representing the first rotation around the z-axis. */
   private double yaw;
   /** The pitch angle representing the second rotation around the y-axis. */
   private double pitch;
   /** The roll angle representing the third rotation around the x-axis. */
   private double roll;

   /**
    * Creates a yaw-pitch-roll that represents a "zero" rotation. The three angles are initialized to
    * zero.
    */
   public YawPitchRoll()
   {
      setToZero();
   }

   /**
    * Creates a yaw-pitch-roll that represents the same orientation as the given one.
    *
    * @param orientation the orientation used to initialized this. Not modified.
    */
   public YawPitchRoll(Orientation3DReadOnly orientation)
   {
      set(orientation);
   }

   /**
    * Creates a yaw-pitch-roll that represents the same orientation as the given rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to initialized this. Not modified.
    */
   public YawPitchRoll(Vector3DReadOnly rotationVector)
   {
      setRotationVector(rotationVector);
   }

   /**
    * Creates a yaw-pitch-roll with the given angles.
    *
    * @param yaw   the angle representing the first rotation around the z-axis.
    * @param pitch the angle representing the second rotation around the y-axis.
    * @param roll  the angle representing the third rotation around the x-axis.
    */
   public YawPitchRoll(double yaw, double pitch, double roll)
   {
      set(yaw, pitch, roll);
   }

   /**
    * Creates a yaw-pitch-roll initialized with the values contained in the given array:
    * <ul>
    * <li>{@code this.setYaw(yawPitchRollArray[0]);}
    * <li>{@code this.setPitch(yawPitchRollArray[1]);}
    * <li>{@code this.setRoll(yawPitchRollArray[2]);}
    * </ul>
    *
    * @param yawPitchRollArray the array containing the values for this yaw-pitch-roll. Not modified.
    */
   public YawPitchRoll(double[] yawPitchRollArray)
   {
      set(yawPitchRollArray);
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    *
    * @param other the other yaw-pitch-roll. Not modified.
    */
   @Override
   public void set(YawPitchRoll other)
   {
      YawPitchRollBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setYaw(double yaw)
   {
      this.yaw = yaw;
   }

   /** {@inheritDoc} */
   @Override
   public void setPitch(double pitch)
   {
      this.pitch = pitch;
   }

   /** {@inheritDoc} */
   @Override
   public void setRoll(double roll)
   {
      this.roll = roll;
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return yaw;
   }

   /** {@inheritDoc} */
   @Override
   public double getPitch()
   {
      return pitch;
   }

   /** {@inheritDoc} */
   @Override
   public double getRoll()
   {
      return roll;
   }

   /**
    * Tests on a per component basis, if this yaw-pitch-roll is exactly equal to {@code other}. A
    * failing test does not necessarily mean that the two yaw-pitch-rolls represent two different
    * orientations.
    *
    * @param object the other yaw-pitch-roll to compare against this. Not modified.
    * @return {@code true} if the two yaw-pitch-rolls are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof YawPitchRollReadOnly)
         return YawPitchRollBasics.super.equals((YawPitchRollReadOnly) object);
      else
         return false;
   }

   /**
    * Tests on a per component basis, if this yaw-pitch-roll is equal to {@code other} to an
    * {@code epsilon}. A failing test does not necessarily mean that the two yaw-pitch-rolls represent
    * two different orientations.
    *
    * @param other   the other yaw-pitch-roll to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two yaw-pitch-rolls are equal component-wise, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(YawPitchRoll other, double epsilon)
   {
      return YawPitchRollBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two axis-angle are considered geometrically equal if the magnitude of their difference is less
    * than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other axis-angle to compare against this. Not modified.
    * @param epsilon the maximum angle for the two quaternions to be considered equal.
    * @return {@code true} if the two axis-angle represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(YawPitchRoll other, double epsilon)
   {
      return YawPitchRollBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this yaw-pitch-roll.
    *
    * @return the hash code value for this yaww-pitch-roll.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(yaw);
      bits = 31L * bits + Double.doubleToLongBits(pitch);
      bits = 31L * bits + Double.doubleToLongBits(roll);
      return (int) (bits ^ bits >> 32);
   }

   /**
    * Provides a {@code String} representation of this yaw-pitch-roll as follows: yaw-pitch-roll: (yaw,
    * pitch, roll).
    *
    * @return the {@code String} representing this yaw-pitch-roll.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getYawPitchRollString(this);
   }
}
