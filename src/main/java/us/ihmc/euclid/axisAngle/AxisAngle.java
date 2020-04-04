package us.ihmc.euclid.axisAngle;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * An {@code AxisAngle} is used to represent a 3D orientation by a unitary axis of components (x, y,
 * z) and an angle of rotation usually expressed in radians.
 * <p>
 * This version of axis-angle uses double precision fields to save the value of each component. It
 * is meant for garbage free usage.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class AxisAngle implements AxisAngleBasics, EpsilonComparable<AxisAngle>, Settable<AxisAngle>, GeometricallyComparable<AxisAngle>
{
   /** The axis part of this axis-angle. */
   private final UnitVector3D axis = new UnitVector3D();
   /** The angle component of this axis-angle. */
   private double angle;

   /**
    * Creates an axis-angle that represents a "zero" rotation. The axis is equal to (1, 0, 0) and the
    * angle to 0.
    */
   public AxisAngle()
   {
      setToZero();
   }

   /**
    * Creates an axis-angle that represents the same orientation as the given one.
    *
    * @param orientation the orientation to initialize this axis-angle. Not modified.
    */
   public AxisAngle(Orientation3DReadOnly orientation)
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
   public AxisAngle(double x, double y, double z, double angle)
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
   public AxisAngle(double[] axisAngleArray)
   {
      set(axisAngleArray);
   }

   /**
    * Create an axis-angle from the given axis and angle.
    *
    * @param axis  the axis. Not modified
    * @param angle the angle value.
    */
   public AxisAngle(Vector3DReadOnly axis, double angle)
   {
      set(axis, angle);
   }

   /**
    * Creates an axis-angle such that it represents the same orientation the rotation vector
    * represents. See
    * {@link AxisAngleConversion#convertRotationVectorToAxisAngle(Vector3DReadOnly, AxisAngleBasics)}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to create this axis-angle. Not modified.
    */
   public AxisAngle(Vector3DReadOnly rotationVector)
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
   public AxisAngle(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this axis-angle to the same value as the given axis-angle {@code other}.
    *
    * @param other the other axis-angle. Not modified.
    */
   @Override
   public void set(AxisAngle other)
   {
      AxisAngleBasics.super.set(other);
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
      this.angle = angle;
   }

   /** {@inheritDoc} */
   @Override
   public final double getAngle()
   {
      return angle;
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
         return equals((AxisAngleReadOnly) object);
      else
         return false;
   }

   /**
    * Tests on a per component basis, if this axis-angle is equal to {@code other} to an
    * {@code epsilon}. A failing test does not necessarily mean that the two axis-angles represent two
    * different orientations.
    *
    * @param other   the other axis-angle to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two axis-angle are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(AxisAngle other, double epsilon)
   {
      return AxisAngleBasics.super.epsilonEquals(other, epsilon);
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
   public boolean geometricallyEquals(AxisAngle other, double epsilon)
   {
      return AxisAngleBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this axis-angle as follows: (x, y, z, angle).
    *
    * @return the {@code String} representing this axis-angle.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getAxisAngleString(this);
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
