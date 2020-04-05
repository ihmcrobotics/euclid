package us.ihmc.euclid;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

/**
 * {@code Axis3D} can be used to provide a simple and readable way to refer the three main axes of a
 * coordinate system.
 */
public enum Axis3D implements UnitVector3DReadOnly
{
   /** The x-axis is usually associated with the forward direction. */
   X(1.0, 0.0, 0.0),
   /**
    * The y-axis is usually associated with the direction pointing to the left of the forward direction
    * and is horizontal.
    */
   Y(0.0, 1.0, 0.0),
   /**
    * The z-axis is usually associated with the vertical direction, parallel to the gravity vector but
    * pointing the opposite direction.
    */
   Z(0.0, 0.0, 1.0);

   /**
    * Static final field holding the return from {@link #values()}. This field should be used in place
    * of calling values() for garbage-free operations.
    */
   public static final Axis3D[] values = values();

   private final double x, y, z;

   Axis3D(double x, double y, double z)
   {
      this.x = x;
      this.y = y;
      this.z = z;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDirty()
   {
      return false;
   }

   /**
    * Returns the x-component of this axis.
    *
    * @return the x-component.
    */
   @Override
   public double getX()
   {
      return x;
   }

   /**
    * Returns the y-component of this axis.
    *
    * @return the y-component.
    */
   @Override
   public double getY()
   {
      return y;
   }

   /**
    * Returns the z-component of this axis.
    *
    * @return the z-component.
    */
   @Override
   public double getZ()
   {
      return z;
   }

   /**
    * Returns the x-component of this axis.
    *
    * @return the x-component.
    */
   @Override
   public double getRawX()
   {
      return x;
   }

   /**
    * Returns the y-component of this axis.
    *
    * @return the y-component.
    */
   @Override
   public double getRawY()
   {
      return y;
   }

   /**
    * Returns the z-component of this axis.
    *
    * @return the z-component.
    */
   @Override
   public double getRawZ()
   {
      return z;
   }

   /**
    * Gets the value of the tuple for the given axis.
    *
    * @param tuple the tuple to get value from. Not modified.
    * @param axis  the {@link Axis3D} to get value for. Not modified.
    * @return the double value of {@code tuple} for {@code axis}.
    */
   public static double get(Tuple3DBasics tuple, Axis3D axis)
   {
      switch (axis)
      {
         case X:
            return tuple.getX();
         case Y:
            return tuple.getY();
         case Z:
            return tuple.getZ();
         default:
            throw new IndexOutOfBoundsException();
      }
   }

   /**
    * Sets the value of the given tuple for the given axis to the given value.
    *
    * @param tupleToModify the tuple to set value of. Modified.
    * @param axis          the {@link Axis3D} to set value for. Not modified.
    * @param value         the double value to set {@code axis} of {@code tupleToModify} to.
    */
   public static void set(Tuple3DBasics tupleToModify, Axis3D axis, double value)
   {
      switch (axis)
      {
         case X:
            tupleToModify.setX(value);
            break;
         case Y:
            tupleToModify.setY(value);
            break;
         case Z:
            tupleToModify.setZ(value);
            break;
         default:
            throw new IndexOutOfBoundsException();
      }
   }

   /**
    * Obtains the next axis, in a clockwise fashion.
    *
    * @return next clockwise axis
    */
   public Axis3D getNextClockwiseAxis()
   {
      switch (this)
      {
         case X:
            return Z;
         case Y:
            return X;
         default:
            return Y;
      }
   }

   /**
    * Obtains the next axis, in a counterclockwise fashion.
    *
    * @return next counterclockwise axis
    */
   public Axis3D getNextCounterClockwiseAxis()
   {
      switch (this)
      {
         case X:
            return Y;
         case Y:
            return Z;
         default:
            return X;
      }
   }
}
