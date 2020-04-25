package us.ihmc.euclid;

import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * {@code Axis2D} can be used to provide a simple and readable way to refer the three main axes of a
 * coordinate system.
 */
public enum Axis2D implements UnitVector2DReadOnly
{
   /** The x-axis is usually associated with the forward direction. */
   X(1.0, 0.0),
   /**
    * The y-axis is usually associated with the direction pointing to the left of the forward direction
    * and is horizontal.
    */
   Y(0.0, 1.0);

   /**
    * Static final field holding the return from {@link #values()}. This field should be used in place
    * of calling values() for garbage-free operations.
    */
   public static final Axis2D[] values = values();

   private final double x, y;
   private final UnitVector2DReadOnly negated = EuclidCoreFactories.newNegativeLinkedUnitVector2D(this);

   Axis2D(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   /**
    * Returns a view of this axis negated.
    * 
    * @return this axis negated.
    */
   public UnitVector2DReadOnly negated()
   {
      return negated;
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
    * Gets the value of the tuple for the given axis.
    *
    * @param tuple the tuple to get value from. Not modified.
    * @param axis  the {@link Axis2D} to get value for. Not modified.
    * @return the double value of {@code tuple} for {@code axis}.
    */
   public static double get(Tuple2DBasics tuple, Axis2D axis)
   {
      switch (axis)
      {
         case X:
            return tuple.getX();
         case Y:
            return tuple.getY();
         default:
            throw new IndexOutOfBoundsException();
      }
   }

   /**
    * Sets the value of the given tuple for the given axis to the given value.
    *
    * @param tupleToModify the tuple to set value of. Modified.
    * @param axis          the {@link Axis2D} to set value for. Not modified.
    * @param value         the double value to set {@code axis} of {@code tupleToModify} to.
    */
   public static void set(Tuple2DBasics tupleToModify, Axis2D axis, double value)
   {
      switch (axis)
      {
         case X:
            tupleToModify.setX(value);
            break;
         case Y:
            tupleToModify.setY(value);
            break;
         default:
            throw new IndexOutOfBoundsException();
      }
   }

   /**
    * Gets the value of the tuple for the given axis.
    *
    * @param tuple the tuple to get value from. Not modified.
    * @param axis  the {@link Axis2D} to get value for. Not modified.
    * @return the double value of {@code tuple} for {@code axis}.
    */
   public static double get(Tuple3DBasics tuple, Axis2D axis)
   {
      switch (axis)
      {
         case X:
            return tuple.getX();
         case Y:
            return tuple.getY();
         default:
            throw new IndexOutOfBoundsException();
      }
   }

   /**
    * Sets the value of the given tuple for the given axis to the given value.
    *
    * @param tupleToModify the tuple to set value of. Modified.
    * @param axis          the {@link Axis2D} to set value for. Not modified.
    * @param value         the double value to set {@code axis} of {@code tupleToModify} to.
    */
   public static void set(Tuple3DBasics tupleToModify, Axis2D axis, double value)
   {
      switch (axis)
      {
         case X:
            tupleToModify.setX(value);
            break;
         case Y:
            tupleToModify.setY(value);
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
   public Axis2D getNextClockwiseAxis()
   {
      switch (this)
      {
         case X:
            return Y;
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
   public Axis2D getNextCounterClockwiseAxis()
   {
      switch (this)
      {
         case X:
            return Y;
         case Y:
            return X;
         default:
            return X;
      }
   }
}
