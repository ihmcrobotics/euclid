package us.ihmc.euclid.axisAngle;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.UnitVector3D32;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * An {@code AxisAngle} is used to represent a 3D orientation by a unitary axis of components (x, y,
 * z) and an angle of rotation usually expressed in radians. This version of axis-angle uses single
 * precision fields to save the value of each component. It is meant for garbage free usage and for
 * situations where heap memory is limited. When memory is not a constraint, the use of
 * {@link AxisAngle} is preferable.
 *
 * @author Sylvain Bertrand
 */
public class AxisAngle32 implements AxisAngleBasics, Settable<AxisAngle32>
{
   /** The axis part of this axis-angle. */
   private final UnitVector3D32 axis = new UnitVector3D32(Axis3D.X);
   /** The angle component of this axis-angle. */
   private float angle;

   /**
    * Creates an axis-angle that represents a "zero" rotation. The axis is equal to {@link Axis3D#X}
    * and the angle to 0.
    */
   public AxisAngle32()
   {
      setToZero();
   }

   /**
    * Creates an axis-angle that represents the same orientation as the given one.
    *
    * @param orientation the orientation to initialize this axis-angle. Not modified.
    */
   public AxisAngle32(Orientation3DReadOnly orientation)
   {
      set(orientation);
   }

   /**
    * Creates an axis-angle with the given values of the axis ({@code x}, {@code y}, {@code z}) and of
    * the angle {@code angle}.
    *
    * @param x     x-component of the axis.
    * @param y     y-component of the axis.
    * @param z     z-component of the axis.
    * @param angle the angle value.
    */
   public AxisAngle32(float x, float y, float z, float angle)
   {
      set(x, y, z, angle);
   }

   /**
    * Creates an axis-angle initialized with the values contained in the given array:
    * <ul>
    * <li>{@code this.setX(axisAngleArray[0]);}
    * <li>{@code this.setY(axisAngleArray[1]);}
    * <li>{@code this.setZ(axisAngleArray[2]);}
    * <li>{@code this.setAngle(axisAngleArray[3]);}
    * </ul>
    *
    * @param axisAngleArray the array containing the values for this axis-angle. Not modified.
    */
   public AxisAngle32(float[] axisAngleArray)
   {
      set(axisAngleArray);
   }

   /**
    * Create an axis-angle from the given axis and angle.
    *
    * @param axis  the axis. Not modified
    * @param angle the angle value.
    */
   public AxisAngle32(Vector3DReadOnly axis, float angle)
   {
      set(axis, angle);
   }

   /**
    * Creates an axis-angle such that it represents the same orientation the rotation vector
    * represents. See
    * {@link AxisAngleConversion#convertRotationVectorToAxisAngle(Vector3DReadOnly, AxisAngleBasics)}.
    *
    * @param rotationVector the rotation vector used to create this axis-angle. Not modified.
    */
   public AxisAngle32(Vector3DReadOnly rotationVector)
   {
      setRotationVector(rotationVector);
   }

   /**
    * Creates an axis-angle such that it represents the same orientation the yaw-pitch-roll angles
    * represents. See
    * {@link AxisAngleConversion#convertYawPitchRollToAxisAngle(double, double, double, AxisAngleBasics)}.
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   public AxisAngle32(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this axis-angle to the same value as the given axis-angle {@code other}.
    *
    * @param other the other axis-angle. Not modified.
    */
   @Override
   public void set(AxisAngle32 other)
   {
      AxisAngleBasics.super.set(other);
   }

   /**
    * Sets this axis-angle to represent a new rotation of axis ({@code x}, {@code y}, {@code z}) and
    * angle of {@code angle}.
    *
    * @param x     x-component of the new axis.
    * @param y     y-component of the new axis.
    * @param z     z-component of the new axis.
    * @param angle the new angle.
    */
   public final void set(float x, float y, float z, float angle)
   {
      axis.set(x, y, z);
      this.angle = angle;
   }

   /** {@inheritDoc} */
   @Override
   public UnitVector3DBasics getAxis()
   {
      return axis;
   }

   /** {@inheritDoc} */
   @Override
   public final void setAngle(double angle)
   {
      this.angle = (float) angle;
   }

   /** {@inheritDoc} */
   @Override
   public final double getAngle()
   {
      return angle;
   }

   /** {@inheritDoc} */
   @Override
   public final float getAngle32()
   {
      return angle;
   }

   /** {@inheritDoc} */
   @Override
   public final float getX32()
   {
      return axis.getX32();
   }

   /** {@inheritDoc} */
   @Override
   public final float getY32()
   {
      return axis.getY32();
   }

   /** {@inheritDoc} */
   @Override
   public final float getZ32()
   {
      return axis.getZ32();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(AxisAngleReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof AxisAngleReadOnly)
         return AxisAngleBasics.super.equals((AxisAngleReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this axis-angle as follows: (x, y, z, angle).
    *
    * @return the {@code String} representing this axis-angle.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this axis-angle.
    *
    * @return the hash code value for this axis-angle.
    */
   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(1L, axis);
      bits = EuclidHashCodeTools.addToHashCode(bits, angle);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
