package us.ihmc.euclid.axisAngle.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.tools.AxisAngleTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for an axis-angle object.
 * <p>
 * An axis-angle is used to represent a 3D orientation by a unitary axis of components (x, y, z) and
 * an angle of rotation usually expressed in radians.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface AxisAngleBasics extends AxisAngleReadOnly, Orientation3DBasics, Clearable
{
   /**
    * Sets a new angle to this axis-angle.
    *
    * @param angle the new angle.
    */
   void setAngle(double angle);

   /**
    * Sets a new x-component for the axis of this axis-angle.
    *
    * @param x the new axis x-component.
    */
   void setX(double x);

   /**
    * Sets a new y-component for the axis of this axis-angle.
    *
    * @param y the new axis y-component.
    */
   void setY(double y);

   /**
    * Sets a new z-component for the axis of this axis-angle.
    *
    * @param z the new axis z-component.
    */
   void setZ(double z);

   /**
    * Sets the components of this axis-angle to represent a "zero" rotation. After calling the axis
    * is equal to (1, 0, 0) and the angle to 0.
    */
   @Override
   default void setToZero()
   {
      set(1.0, 0.0, 0.0, 0.0);
   }

   /**
    * Sets the components of this axis-angle to {@link Double#NaN}.
    */
   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   /**
    * Tests if this axis-angle contains a {@link Double#NaN}.
    *
    * @return {@code true} if this axis-angle contains a {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return AxisAngleReadOnly.super.containsNaN();
   }

   /**
    * Sets each component of this axis-angle to its absolute value.
    */
   default void absolute()
   {
      set(Math.abs(getX()), Math.abs(getY()), Math.abs(getZ()), Math.abs(getAngle()));
   }

   /**
    * Negates each component of this axis-angle.
    */
   default void negate()
   {
      set(-getX(), -getY(), -getZ(), -getAngle());
   }

   /** {@inheritDoc} */
   @Override
   default void invert()
   {
      setAngle(-getAngle());
   }

   /**
    * Normalizes the axis of this axis-angle such that its norm is equal to 1 after calling this
    * method and its direction remains unchanged.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if this axis-angle contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   @Override
   default void normalize()
   {
      if (containsNaN())
         return;

      double invNorm = axisNorm();

      if (invNorm == 0.0)
      {
         setToZero();
         return;
      }

      invNorm = 1.0 / invNorm;
      double ux = getX() * invNorm;
      double uy = getY() * invNorm;
      double uz = getZ() * invNorm;
      double angle = getAngle();
      set(ux, uy, uz, angle);
   }

   /**
    * Multiplies the angle of this axis-angle by the given {@code scale}.
    *
    * @param scale the scaling factor to apply to the angle of this axis-angle.
    */
   default void scaleAngle(double scale)
   {
      setAngle(scale * getAngle());
   }

   /**
    * Sets this axis-angle to represent a new rotation of axis ({@code x}, {@code y}, {@code z}) and
    * angle of {@code angle}.
    *
    * @param x x-component of the new axis.
    * @param y y-component of the new axis.
    * @param z z-component of the new axis.
    * @param angle the new angle.
    */
   default void set(double x, double y, double z, double angle)
   {
      setAngle(angle);
      setX(x);
      setY(y);
      setZ(z);
   }

   /** {@inheritDoc} */
   @Override
   default void set(Orientation3DReadOnly orientation3DReadOnly)
   {
      orientation3DReadOnly.get(this);
   }

   /**
    * Sets the axis and the angle of this axis-angle.
    *
    * @param axis the new axis. Not modified.
    * @param angle the new angle.
    */
   default void set(Vector3DReadOnly axis, double angle)
   {
      set(axis.getX(), axis.getY(), axis.getZ(), angle);
   }

   /**
    * Sets this axis-angle to the same value as the given axis-angle {@code other}.
    *
    * @param other the other axis-angle. Not modified.
    */
   default void set(AxisAngleReadOnly other)
   {
      set(other.getX(), other.getY(), other.getZ(), other.getAngle());
   }

   /**
    * Sets this axis-angle to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other axis-angle to copy the values from. Not modified.
    */
   default void setAndNegate(AxisAngleReadOnly other)
   {
      set(other);
      negate();
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    * <li>{@code this.setX(axisAngleArray[0]);}
    * <li>{@code this.setY(axisAngleArray[1]);}
    * <li>{@code this.setZ(axisAngleArray[2]);}
    * <li>{@code this.setAngle(axisAngleArray[3]);}
    * </ul>
    *
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(double[] axisAngleArray)
   {
      set(0, axisAngleArray);
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    * <li>{@code this.setX(axisAngleArray[startIndex + 0]);}
    * <li>{@code this.setY(axisAngleArray[startIndex + 1]);}
    * <li>{@code this.setZ(axisAngleArray[startIndex + 2]);}
    * <li>{@code this.setAngle(axisAngleArray[startIndex + 3]);}
    * </ul>
    *
    * @param startIndex the first index to start reading from in the array.
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(int startIndex, double[] axisAngleArray)
   {
      setX(axisAngleArray[startIndex++]);
      setY(axisAngleArray[startIndex++]);
      setZ(axisAngleArray[startIndex++]);
      setAngle(axisAngleArray[startIndex]);
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    * <li>{@code this.setX(axisAngleArray[0]);}
    * <li>{@code this.setY(axisAngleArray[1]);}
    * <li>{@code this.setZ(axisAngleArray[2]);}
    * <li>{@code this.setAngle(axisAngleArray[3]);}
    * </ul>
    *
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(float[] axisAngleArray)
   {
      set(0, axisAngleArray);
   }

   /**
    * Copies the values in the given array into this axis-angle as follows:
    * <ul>
    * <li>{@code this.setX(axisAngleArray[startIndex + 0]);}
    * <li>{@code this.setY(axisAngleArray[startIndex + 1]);}
    * <li>{@code this.setZ(axisAngleArray[startIndex + 2]);}
    * <li>{@code this.setAngle(axisAngleArray[startIndex + 3]);}
    * </ul>
    *
    * @param startIndex the first index to start reading from in the array.
    * @param axisAngleArray the array containing the new values for this axis-angle. Not modified.
    */
   default void set(int startIndex, float[] axisAngleArray)
   {
      setX(axisAngleArray[startIndex++]);
      setY(axisAngleArray[startIndex++]);
      setZ(axisAngleArray[startIndex++]);
      setAngle(axisAngleArray[startIndex]);
   }

   /** {@inheritDoc} */
   @Override
   default void setAxisAngle(double x, double y, double z, double angle)
   {
      set(x, y, z, angle);
   }

   /** {@inheritDoc} */
   @Override
   default void setQuaternion(double x, double y, double z, double s)
   {
      AxisAngleConversion.convertQuaternionToAxisAngle(x, y, z, s, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setRotationVector(double x, double y, double z)
   {
      AxisAngleConversion.convertRotationVectorToAxisAngle(x, y, z, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      AxisAngleConversion.convertYawPitchRollToAxisAngle(yaw, pitch, roll, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      AxisAngleConversion.convertMatrixToAxisAngle(m00, m01, m02, m10, m11, m12, m20, m21, m22, this);
   }

   /**
    * Selects a component of this axis-angle based on {@code index} and sets it to {@code value}.
    * <p>
    * For {@code index} values of 0, 1, and 2, the corresponding components are x, y, and z,
    * respectively, while 3 corresponds to the angle.
    * </p>
    *
    * @param index the index of the component to set.
    * @param value the new value of the selected component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default void setElement(int index, double value)
   {
      switch (index)
      {
      case 0:
         setX(value);
         break;
      case 1:
         setY(value);
         break;
      case 2:
         setZ(value);
         break;
      case 3:
         setAngle(value);
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Multiplies this axis-angle by {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void multiply(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiply(this, other, this);
   }

   /** {@inheritDoc} */
   @Override
   default void append(Orientation3DReadOnly other)
   {
      AxisAngleTools.multiply(this, false, other, false, this);
   }

   /**
    * Sets this axis-angle to the multiplication of {@code aa1} and {@code aa2}.
    * <p>
    * this = aa1 * aa2
    * </p>
    *
    * @param aa1 the first axis-angle in the multiplication. Not modified.
    * @param aa2 the second axis-angle in the multiplication. Not modified.
    */
   default void multiply(AxisAngleReadOnly aa1, AxisAngleReadOnly aa2)
   {
      AxisAngleTools.multiply(aa1, aa2, this);
   }

   /**
    * Multiplies this axis-angle by the inverse of {@code other}.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void multiplyInvertOther(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiplyInvertRight(this, other, this);
   }

   /** {@inheritDoc} */
   @Override
   default void appendInvertOther(Orientation3DReadOnly orientation)
   {
      AxisAngleTools.multiply(this, false, orientation, true, this);
   }

   /**
    * Sets this axis-angle to the multiplication of the inverse of {@code this} and {@code other}.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void multiplyInvertThis(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiplyInvertLeft(this, other, this);
   }

   /**
    * Sets this axis-angle to the multiplication of the inverse of {@code this} and the inverse of
    * {@code other}.
    * <p>
    * this = this<sup>-1</sup> * other<sup>-1</sup>
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void multiplyInvertBoth(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiplyInvertBoth(this, other, this);
   }

   /**
    * Append a rotation about the z-axis to this axis-angle.
    *
    * <pre>
    *               / ux    =  0  \
    * this = this * | uy    =  0  |
    *               | uz    =  1  |
    *               \ angle = yaw /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void appendYawRotation(double yaw)
   {
      AxisAngleTools.appendYawRotation(this, yaw, this);
   }

   /**
    * Append a rotation about the y-axis to this axis-angle.
    *
    * <pre>
    *               / ux    =  0    \
    * this = this * | uy    =  1    |
    *               | uz    =  0    |
    *               \ angle = pitch /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void appendPitchRotation(double pitch)
   {
      AxisAngleTools.appendPitchRotation(this, pitch, this);
   }

   /**
    * Append a rotation about the x-axis to this axis-angle.
    *
    * <pre>
    *               / ux    =  1   \
    * this = this * | uy    =  0   |
    *               | uz    =  0   |
    *               \ angle = roll /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void appendRollRotation(double roll)
   {
      AxisAngleTools.appendRollRotation(this, roll, this);
   }

   /**
    * Pre-multiplies this axis-angle by {@code other}.
    * <p>
    * this = other * other
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void preMultiply(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiply(other, this, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prepend(Orientation3DReadOnly orientation)
   {
      AxisAngleTools.multiply(orientation, false, this, false, this);
   }

   /**
    * Sets this axis-angle to the multiplication of the inverse of {@code other} and {@code this}.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void preMultiplyInvertOther(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiplyInvertLeft(other, this, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prependInvertOther(Orientation3DReadOnly orientation)
   {
      AxisAngleTools.multiply(orientation, true, this, false, this);
   }

   /**
    * Sets this axis-angle to the multiplication of {@code other} and the inverse of {@code this}.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void preMultiplyInvertThis(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiplyInvertRight(other, this, this);
   }

   /**
    * Sets this axis-angle to the multiplication of the inverse of {@code other} and the inverse of
    * {@code this}.
    * <p>
    * this = other<sup>-1</sup> * this<sup>-1</sup>
    * </p>
    *
    * @param other the other axis-angle to multiply this. Not modified.
    */
   default void preMultiplyInvertBoth(AxisAngleReadOnly other)
   {
      AxisAngleTools.multiplyInvertBoth(other, this, this);
   }

   /**
    * Prepend a rotation about the z-axis to this axis-angle.
    *
    * <pre>
    *        / ux    =  0  \
    * this = | uy    =  0  | * this
    *        | uz    =  1  |
    *        \ angle = yaw /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void prependYawRotation(double yaw)
   {
      AxisAngleTools.prependYawRotation(yaw, this, this);
   }

   /**
    * Prepend a rotation about the y-axis to this axis-angle.
    *
    * <pre>
    *        / ux    =  0    \
    * this = | uy    =  1    | * this
    *        | uz    =  0    |
    *        \ angle = pitch /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void prependPitchRotation(double pitch)
   {
      AxisAngleTools.prependPitchRotation(pitch, this, this);
   }

   /**
    * Prepend a rotation about the x-axis to this axis-angle.
    *
    * <pre>
    *        / ux    =  1   \
    * this = | uy    =  0   | * this
    *        | uz    =  0   |
    *        \ angle = roll /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void prependRollRotation(double roll)
   {
      AxisAngleTools.prependRollRotation(roll, this, this);
   }
}