package us.ihmc.euclid.yawPitchRoll.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tools.YawPitchRollTools;

/**
 * Write and read interface for a yaw-pitch-roll object.
 * <p>
 * A yaw-pitch-roll is used to represent a 3D orientation by three successive rotations: rotation
 * around the z-axis (yaw), then around the y-axis (pitch), and then around the x-axis (roll). The
 * three components yaw, pitch, and roll represents the angle for rotation expressed in radians.
 * </p>
 * <p>
 * In general, yaw-pitch-roll representation is considered one of the most intuitive way of
 * interpreting an orientation and is thus commonly used as an interface between human and machine.
 * However, there is no algebra directly accessible for manipulating orientations represented as
 * yaw-pitch-roll making it highly computationally expensive when compared to rotation matrices or
 * quaternions. In addition, yaw-pitch-roll representation is sensitive to gimbal lock which happens
 * when the pitch angle is in the neighborhood of either <i>pi/2</i> or -<i>pi/2</i>. When close to such
 * configuration, converting orientation to yaw-pitch-roll becomes inaccurate and can sometimes lead
 * to unexpected results.
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
public interface YawPitchRollBasics extends YawPitchRollReadOnly, Orientation3DBasics
{
   /**
    * Sets the yaw angle.
    * 
    * @param yaw the new yaw angle.
    */
   void setYaw(double yaw);

   /**
    * Sets the pitch angle.
    * 
    * @param pitch the new pitch angle.
    */
   void setPitch(double pitch);

   /**
    * Sets the roll angle.
    * 
    * @param roll the new roll angle.
    */
   void setRoll(double roll);

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      set(0.0, 0.0, 0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return YawPitchRollReadOnly.super.containsNaN();
   }

   /**
    * Negates each angle of this yaw-pitch-roll.
    */
   default void negate()
   {
      set(-getYaw(), -getPitch(), -getRoll());
   }

   /**
    * Sets each component of this yaw-pitch-roll to its absolute value.
    */
   default void absolute()
   {
      set(Math.abs(getYaw()), Math.abs(getPitch()), Math.abs(getRoll()));
   }

   /** {@inheritDoc} */
   @Override
   default void invert()
   {
      YawPitchRollTools.invert(getYaw(), getPitch(), getRoll(), this);
   }

   /**
    * This does nothing for yaw-pitch-roll.
    */
   @Override
   default void normalize()
   {
      // Yaw-pitch-roll does not have to be normalized.
   }

   /**
    * Sets this yaw-pitch-roll angles.
    * 
    * @param yaw the new yaw angle.
    * @param pitch the new pitch angle.
    * @param roll the new roll angle.
    */
   default void set(double yaw, double pitch, double roll)
   {
      setYaw(yaw);
      setPitch(pitch);
      setRoll(roll);
   }

   /** {@inheritDoc} */
   @Override
   default void set(Orientation3DReadOnly orientation3dReadOnly)
   {
      orientation3dReadOnly.get(this);
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param other the other yaw-pitch-roll. Not modified.
    */
   default void set(YawPitchRollReadOnly other)
   {
      set(other.getYaw(), other.getPitch(), other.getRoll());
   }

   /**
    * Sets this yaw-pitch-roll to {@code other} and calls {@link #negate()}.
    * 
    * @param other the other yaw-pitch-roll to copy the values from. Not modified.
    */
   default void setAndNegate(YawPitchRollReadOnly other)
   {
      set(other);
      negate();
   }

   /**
    * Copies the values in the given array into this yaw-pitch-roll as follows:
    * <ul>
    * <li>{@code this.setYaw(yawPitchRollArray[0]);}
    * <li>{@code this.setPitch(yawPitchRollArray[1]);}
    * <li>{@code this.setRoll(yawPitchRollArray[2]);}
    * </ul>
    *
    * @param yawPitchRollArray the array containing the new values for this yaw-pitch-roll. Not
    *           modified.
    */
   default void set(double[] yawPitchRollArray)
   {
      set(0, yawPitchRollArray);
   }

   /**
    * Copies the values in the given array into this yaw-pitch-roll as follows:
    * <ul>
    * <li>{@code this.setYaw(yawPitchRollArray[startIndex + 0]);}
    * <li>{@code this.setPitch(yawPitchRollArray[startIndex + 1]);}
    * <li>{@code this.setRoll(yawPitchRollArray[startIndex + 2]);}
    * </ul>
    *
    * @param startIndex the first index to start reading from in the array.
    * @param yawPitchRollArray the array containing the new values for this yaw-pitch-roll. Not
    *           modified.
    */
   default void set(int startIndex, double[] yawPitchRollArray)
   {
      setYaw(yawPitchRollArray[startIndex++]);
      setPitch(yawPitchRollArray[startIndex++]);
      setRoll(yawPitchRollArray[startIndex]);
   }

   /**
    * Copies the values in the given array into this yaw-pitch-roll as follows:
    * <ul>
    * <li>{@code this.setYaw(yawPitchRollArray[0]);}
    * <li>{@code this.setPitch(yawPitchRollArray[1]);}
    * <li>{@code this.setRoll(yawPitchRollArray[2]);}
    * </ul>
    *
    * @param yawPitchRollArray the array containing the new values for this yaw-pitch-roll. Not
    *           modified.
    */
   default void set(float[] yawPitchRollArray)
   {
      set(0, yawPitchRollArray);
   }

   /**
    * Copies the values in the given array into this yaw-pitch-roll as follows:
    * <ul>
    * <li>{@code this.setYaw(yawPitchRollArray[startIndex + 0]);}
    * <li>{@code this.setPitch(yawPitchRollArray[startIndex + 1]);}
    * <li>{@code this.setRoll(yawPitchRollArray[startIndex + 2]);}
    * </ul>
    *
    * @param startIndex the first index to start reading from in the array.
    * @param yawPitchRollArray the array containing the new values for this yaw-pitch-roll. Not
    *           modified.
    */
   default void set(int startIndex, float[] yawPitchRollArray)
   {
      setYaw(yawPitchRollArray[startIndex++]);
      setPitch(yawPitchRollArray[startIndex++]);
      setRoll(yawPitchRollArray[startIndex]);
   }

   /** {@inheritDoc} */
   @Override
   default void setAxisAngle(double x, double y, double z, double angle)
   {
      YawPitchRollConversion.convertAxisAngleToYawPitchRoll(x, y, z, angle, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setQuaternion(double x, double y, double z, double s)
   {
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(x, y, z, s, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setRotationVector(double x, double y, double z)
   {
      YawPitchRollConversion.convertRotationVectorToYawPitchRoll(x, y, z, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      set(yaw, pitch, roll);
   }

   /** {@inheritDoc} */
   @Override
   default void setRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      YawPitchRollConversion.convertMatrixToYawPitchRoll(m00, m01, m02, m10, m11, m12, m20, m21, m22, this);
   }

   /**
    * Selects a component of this yaw-pitch-roll based on {@code index} and sets it to {@code value}.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are yaw, pitch, and roll,
    * respectively.
    * </p>
    *
    * @param index the index of the component to set.
    * @param value the new value of the selected component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 2].
    */
   default void setElement(int index, double value)
   {
      switch (index)
      {
      case 0:
         setYaw(value);
         break;
      case 1:
         setPitch(value);
         break;
      case 2:
         setRoll(value);
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /** {@inheritDoc} */
   @Override
   default void append(Orientation3DReadOnly other)
   {
      YawPitchRollTools.multiply(this, false, other, false, this);
   }

   /** {@inheritDoc} */
   @Override
   default void appendInvertOther(Orientation3DReadOnly orientation)
   {
      YawPitchRollTools.multiply(this, false, orientation, true, this);
   }

   /** {@inheritDoc} */
   @Override
   default void appendYawRotation(double yaw)
   {
      YawPitchRollTools.appendYawRotation(this, yaw, this);
   }

   /** {@inheritDoc} */
   @Override
   default void appendPitchRotation(double pitch)
   {
      YawPitchRollTools.appendPitchRotation(this, pitch, this);
   }

   /** {@inheritDoc} */
   @Override
   default void appendRollRotation(double roll)
   {
      YawPitchRollTools.appendRollRotation(this, roll, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prepend(Orientation3DReadOnly orientation)
   {
      YawPitchRollTools.multiply(orientation, false, this, false, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prependInvertOther(Orientation3DReadOnly orientation)
   {
      YawPitchRollTools.multiply(orientation, true, this, false, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prependYawRotation(double yaw)
   {
      YawPitchRollTools.prependYawRotation(this, yaw, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prependPitchRotation(double pitch)
   {
      YawPitchRollTools.prependPitchRotation(this, pitch, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prependRollRotation(double roll)
   {
      YawPitchRollTools.prependRollRotation(this, roll, this);
   }
}
