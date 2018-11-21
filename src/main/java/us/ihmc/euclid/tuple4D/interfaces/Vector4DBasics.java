package us.ihmc.euclid.tuple4D.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a 4 dimensional vector representing a generic quaternion.
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part
 * {@code s} and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 * <li>When transformed by a homogeneous transformation matrix, a quaternion is only pre-multiplied
 * by the rotation part of the transform, resulting in concatenating the orientations of the
 * transform and the quaternion.
 * <li>When transformed by a homogeneous transformation matrix, a 4D vector scalar part {@code s}
 * remains unchanged. The vector part ({@code x}, {@code y}, {@code z}) is scaled and rotated, and
 * translated by {@code s} times the translation part of the transform. Note that for {@code s = 0},
 * a 4D vector behaves as a 3D vector, and for {@code s = 1} it behaves as a 3D point.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Vector4DBasics extends Vector4DReadOnly, Tuple4DBasics
{
   /**
    * Sets the x-component of this vector.
    *
    * @param x the new x-component's value.
    */
   void setX(double x);

   /**
    * Sets the y-component of this vector.
    *
    * @param y the new y-component's value.
    */
   void setY(double y);

   /**
    * Sets the z-component of this vector.
    *
    * @param z the new z-component's value.
    */
   void setZ(double z);

   /**
    * Sets the s-component of this vector.
    *
    * @param s the new s-component's value.
    */
   void setS(double s);

   /**
    * Sets all the components of this vector to zero.
    */
   @Override
   default void setToZero()
   {
      set(0.0, 0.0, 0.0, 0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / norm());
   }

   /**
    * Clips each component of this vector to a maximum value {@code max}.
    *
    * @param max the maximum value for each component.
    */
   default void clipToMax(double max)
   {
      set(Math.min(max, getX()), Math.min(max, getY()), Math.min(max, getZ()), Math.min(max, getS()));
   }

   /**
    * Clips each component of this vector to a minimum value {@code min}.
    *
    * @param min the minimum value for each component.
    */
   default void clipToMin(double min)
   {
      set(Math.max(min, getX()), Math.max(min, getY()), Math.max(min, getZ()), Math.max(min, getS()));
   }

   /**
    * Clips each component of this vector to a minimum value {@code min} and a maximum value
    * {@code max}.
    *
    * @param min the minimum value for each component.
    * @param max the maximum value for each component.
    */
   default void clipToMinMax(double min, double max)
   {
      clipToMax(max);
      clipToMin(min);
   }

   /**
    * Selects a component of this vector based on {@code index} and sets it to {@code value}.
    * <p>
    * For an {@code index} value going from 0 up to 3, the corresponding components are {@code x},
    * {@code y}, {@code z}, and {@code s}, respectively.
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
         setS(value);
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /** {@inheritDoc} */
   @Override
   default void set(double x, double y, double z, double s)
   {
      setX(x);
      setY(y);
      setZ(z);
      setS(s);
   }

   /**
    * Sets this 4D vector to represent the given 3D vector
    * <p>
    * this.xyz = vector3D<br>
    * this.s = 0.0
    * </p>
    *
    * @param vector3D the 3D vector used to set this 4D vector. Not modified.
    */
   default void set(Vector3DReadOnly vector3D)
   {
      set(vector3D.getX(), vector3D.getY(), vector3D.getZ(), 0.0);
   }

   /**
    * Sets this 4D vector to represent the given 3D point
    * <p>
    * this.xyz = point3D<br>
    * this.s = 1.0
    * </p>
    *
    * @param point3D the 3D point used to set this 4D vector. Not modified.
    */
   default void set(Point3DReadOnly point3D)
   {
      set(point3D.getX(), point3D.getY(), point3D.getZ(), 1.0);
   }

   /**
    * Sets this vector to {@code tupleReadOnly} and then scales it {@link #scale(double)}.
    *
    * @param scalar the scale factor to use on this tuple.
    * @param tupleReadOnly the tuple to copy the values from. Not modified.
    */
   default void setAndScale(double scalar, Tuple4DReadOnly tupleReadOnly)
   {
      set(scalar * tupleReadOnly.getX(), scalar * tupleReadOnly.getY(), scalar * tupleReadOnly.getZ(), scalar * tupleReadOnly.getS());
   }

   /**
    * Sets this vector to {@code tupleReadOnly} and then calls {@link #clipToMax(double)}.
    *
    * @param max the maximum value for each component of this tuple.
    * @param tupleReadOnly the tuple to copy the values from. Not modified.
    */
   default void setAndClipToMax(double max, Tuple4DReadOnly tupleReadOnly)
   {
      set(Math.min(max, tupleReadOnly.getX()), Math.min(max, tupleReadOnly.getY()), Math.min(max, tupleReadOnly.getZ()), Math.min(max, tupleReadOnly.getS()));
   }

   /**
    * Sets this vector to {@code tupleReadOnly} and then calls {@link #clipToMin(double)}.
    *
    * @param min the minimum value for each component of this tuple.
    * @param tupleReadOnly the tuple to copy the values from. Not modified.
    */
   default void setAndClipToMin(double min, Tuple4DReadOnly tupleReadOnly)
   {
      set(Math.max(min, tupleReadOnly.getX()), Math.max(min, tupleReadOnly.getY()), Math.max(min, tupleReadOnly.getZ()), Math.max(min, tupleReadOnly.getS()));
   }

   /**
    * Sets this vector to {@code tupleReadOnly} and then calls
    * {@link #clipToMinMax(double, double)}.
    *
    * @param min the minimum value for each component of this tuple.
    * @param max the maximum value for each component of this tuple.
    * @param tupleReadOnly the tuple to copy the values from. Not modified.
    */
   default void setAndClipToMinMax(double min, double max, Tuple4DReadOnly tupleReadOnly)
   {
      set(tupleReadOnly);
      clipToMinMax(min, max);
   }

   /**
    * Adds the given {@code x} to this vector's x-component.
    *
    * @param x the value to add.
    */
   default void addX(double x)
   {
      setX(getX() + x);
   }

   /**
    * Adds the given {@code y} to this vector's y-component.
    *
    * @param y the value to add.
    */
   default void addY(double y)
   {
      setY(getY() + y);
   }

   /**
    * Adds the given {@code z} to this vector's z-component.
    *
    * @param z the value to add.
    */
   default void addZ(double z)
   {
      setZ(getZ() + z);
   }

   /**
    * Adds the given {@code s} to this vector's s-component.
    *
    * @param s the value to add.
    */
   default void addS(double s)
   {
      setS(getS() + s);
   }

   /**
    * Adds the given ({@code x}, {@code y}, {@code z}, {@code s})-tuple to this vector.
    * <p>
    * this = this + (x, y, z, s)
    * </p>
    *
    * @param x the value to add to the x-component of this vector.
    * @param y the value to add to the y-component of this vector.
    * @param z the value to add to the z-component of this vector.
    * @param s the value to add to the s-component of this vector.
    */
   default void add(double x, double y, double z, double s)
   {
      set(getX() + x, getY() + y, getZ() + z, getS() + s);
   }

   /**
    * Adds the given tuple to this vector.
    * <p>
    * this = this + tupleReadOnly
    * </p>
    *
    * @param tupleReadOnly the tuple to add to this vector.
    */
   default void add(Tuple4DReadOnly tupleReadOnly)
   {
      add(tupleReadOnly.getX(), tupleReadOnly.getY(), tupleReadOnly.getZ(), tupleReadOnly.getS());
   }

   /**
    * Sets this vector to the sum of the two given tuples.
    * <p>
    * this = tuple1 + tuple2
    * </p>
    *
    * @param tuple1 the first tuple to sum. Not modified.
    * @param tuple2 the second tuple to sum. Not modified.
    */
   default void add(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2)
   {
      set(tuple1.getX() + tuple2.getX(), tuple1.getY() + tuple2.getY(), tuple1.getZ() + tuple2.getZ(), tuple1.getS() + tuple2.getS());
   }

   /**
    * Subtracts the given {@code x} to this vector's x-component.
    *
    * @param x the value to add.
    */
   default void subX(double x)
   {
      setX(getX() - x);
   }

   /**
    * Subtracts the given {@code y} to this vector's y-component.
    *
    * @param y the value to add.
    */
   default void subY(double y)
   {
      setY(getY() - y);
   }

   /**
    * Subtracts the given {@code z} to this vector's z-component.
    *
    * @param z the value to add.
    */
   default void subZ(double z)
   {
      setZ(getZ() - z);
   }

   /**
    * Subtracts the given {@code s} to this vector's s-component.
    *
    * @param s the value to add.
    */
   default void subS(double s)
   {
      setS(getS() - s);
   }

   /**
    * Subtracts the given ({@code x}, {@code y}, {@code z}, {@code s})-tuple to this vector.
    * <p>
    * this = this - (x, y, z, s)
    * </p>
    *
    * @param x the value to add to the x-component of this vector.
    * @param y the value to add to the y-component of this vector.
    * @param z the value to add to the z-component of this vector.
    * @param s the value to add to the s-component of this vector.
    */
   default void sub(double x, double y, double z, double s)
   {
      set(getX() - x, getY() - y, getZ() - z, getS() - s);
   }

   /**
    * Subtracts the given tuple to this vector.
    * <p>
    * this = this - tupleReadOnly
    * </p>
    *
    * @param tupleReadOnly the tuple to subtract to this vector.
    */
   default void sub(Tuple4DReadOnly tupleReadOnly)
   {
      sub(tupleReadOnly.getX(), tupleReadOnly.getY(), tupleReadOnly.getZ(), tupleReadOnly.getS());
   }

   /**
    * Sets this vector to the difference of the two given tuples.
    * <p>
    * this = tuple1 - tuple2
    * </p>
    *
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple to subtract to {@code tuple1}. Not modified.
    */
   default void sub(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2)
   {
      set(tuple1.getX() - tuple2.getX(), tuple1.getY() - tuple2.getY(), tuple1.getZ() - tuple2.getZ(), tuple1.getS() - tuple2.getS());
   }

   /**
    * Scales the components of this vector by the given {@code scalar}.
    * <p>
    * this = scalar * this
    * </p>
    *
    * @param scalar the scale factor to use.
    */
   default void scale(double scalar)
   {
      scale(scalar, scalar, scalar, scalar);
   }

   /**
    * Scales independently each component of this vector.
    *
    * <pre>
    * / this.x \   / scalarX * this.x \
    * | this.y | = | scalarY * this.y |
    * | this.z |   | scalarZ * this.z |
    * \ this.s /   \ scalarS * this.s /
    * </pre>
    *
    * @param scalarX the scalar factor to use on the x-component of this vector.
    * @param scalarY the scalar factor to use on the y-component of this vector.
    * @param scalarZ the scalar factor to use on the z-component of this vector.
    * @param scalarS the scalar factor to use on the s-component of this vector.
    */
   default void scale(double scalarX, double scalarY, double scalarZ, double scalarS)
   {
      set(scalarX * getX(), scalarY * getY(), scalarZ * getZ(), scalarS * getS());
   }

   /**
    * Scales this vector and adds {@code tupleReadOnly}.
    * <p>
    * this = scalar * this + tupleReadOnly
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param tupleReadOnly the tuple to add to this. Not modified.
    */
   default void scaleAdd(double scalar, Tuple4DReadOnly tupleReadOnly)
   {
      scale(scalar);
      add(tupleReadOnly);
   }

   /**
    * Sets this vector to the sum of {@code tuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * tuple1 + tuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code tuple1}.
    * @param tuple1 the first tuple of the sum. Not modified.
    * @param tuple2 the second tuple of the sum. Not modified.
    */
   default void scaleAdd(double scalar, Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2)
   {
      double x = scalar * tuple1.getX() + tuple2.getX();
      double y = scalar * tuple1.getY() + tuple2.getY();
      double z = scalar * tuple1.getZ() + tuple2.getZ();
      double s = scalar * tuple1.getS() + tuple2.getS();
      set(x, y, z, s);
   }

   /**
    * Scales this vector and subtracts {@code tupleReadOnly}.
    * <p>
    * this = scalar * this - tupleReadOnly
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param tupleReadOnly the tuple to subtract to this. Not modified.
    */
   default void scaleSub(double scalar, Tuple4DReadOnly tupleReadOnly)
   {
      scale(scalar);
      sub(tupleReadOnly);
   }

   /**
    * Sets this vector to the difference of {@code tuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * tuple1 - tuple2
    * </p>
    *
    * @param scalar the scale factor to use on {@code tuple1}.
    * @param tuple1 the first tuple of the difference. Not modified.
    * @param tuple2 the second tuple of the difference. Not modified.
    */
   default void scaleSub(double scalar, Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2)
   {
      double x = scalar * tuple1.getX() - tuple2.getX();
      double y = scalar * tuple1.getY() - tuple2.getY();
      double z = scalar * tuple1.getZ() - tuple2.getZ();
      double s = scalar * tuple1.getS() - tuple2.getS();
      set(x, y, z, s);
   }

   /**
    * Performs a linear interpolation from this vector to {@code tupleReadOnly} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * tupleReadOnly
    * </p>
    *
    * @param tupleReadOnly the tuple used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying this vector, while a value of 1 is equivalent to setting this vector to
    *           {@code tupleReadOnly}.
    */
   default void interpolate(Tuple4DReadOnly tupleReadOnly, double alpha)
   {
      interpolate(this, tupleReadOnly, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code tuple2} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * tuple2
    * </p>
    *
    * @param tuple1 the first tuple used in the interpolation. Not modified.
    * @param tuple2 the second tuple used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this vector to {@code tuple1}, while a value of 1 is equivalent to setting this
    *           vector to {@code tuple2}.
    */
   default void interpolate(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2, double alpha)
   {
      double x = EuclidCoreTools.interpolate(tuple1.getX(), tuple2.getX(), alpha);
      double y = EuclidCoreTools.interpolate(tuple1.getY(), tuple2.getY(), alpha);
      double z = EuclidCoreTools.interpolate(tuple1.getZ(), tuple2.getZ(), alpha);
      double s = EuclidCoreTools.interpolate(tuple1.getS(), tuple2.getS(), alpha);
      set(x, y, z, s);
   }

   /**
    * Transforms the vector part (x, y, z) of this vector as a 3D vector and translates it by
    * {@code s} times the translation part of the transform. The scalar part (s) remains unchanged.
    * <p>
    * Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1} it
    * behaves as a 3D point.
    * </p>
    * <p>
    * <li>{@link RigidBodyTransform} rotates then translates a vector.
    * <li>{@link QuaternionBasedTransform} rotates then translates a vector.
    * <li>{@link AffineTransform} scales, rotates, then translates a vector.
    * </p>
    *
    * @param transform the geometric transform to apply on this vector. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * Performs the inverse the transform on the vector part (x, y, z) of this vector as a 3D vector
    * and translates it by {@code s} times the translation part of the transform. The scalar part
    * (s) remains unchanged.
    * <p>
    * Note that for {@code s = 0}, a 4D vector behaves as a 3D vector, and for {@code s = 1} it
    * behaves as a 3D point.
    * </p>
    * <p>
    * <li>{@link RigidBodyTransform} rotates then translates a vector.
    * <li>{@link QuaternionBasedTransform} rotates then translates a vector.
    * <li>{@link AffineTransform} scales, rotates, then translates a vector.
    * </p>
    *
    * @param transform the geometric transform to apply on this vector. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }
}
