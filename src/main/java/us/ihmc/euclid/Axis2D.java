package us.ihmc.euclid;

import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code Axis2D} can be used to provide a simple and readable way to refer the three main axes of a
 * coordinate system.
 */
public enum Axis2D implements UnitVector2DReadOnly
{
   /** The x-axis is usually associated with the forward direction. */
   X(1.0, 0.0)
   {
      @Override
      public double extract(Tuple2DReadOnly tuple)
      {
         return tuple.getX();
      }

      @Override
      public void insert(Tuple2DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setX(value);
      }

      @Override
      public Axis2D other()
      {
         return Y;
      }

      @Override
      public Axis3D asAxis3D()
      {
         return Axis3D.X;
      }
   },
   /**
    * The y-axis is usually associated with the direction pointing to the left of the forward direction
    * and is horizontal.
    */
   Y(0.0, 1.0)
   {
      @Override
      public double extract(Tuple2DReadOnly tuple)
      {
         return tuple.getY();
      }

      @Override
      public void insert(Tuple2DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setY(value);
      }

      @Override
      public Axis2D other()
      {
         return X;
      }

      @Override
      public Axis3D asAxis3D()
      {
         return Axis3D.Y;
      }
   };

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

   @Override
   public double dot(Tuple2DReadOnly other)
   {
      return extract(other);
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
    * Sets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param tupleToUpdate the tuple to update. Modified.
    * @param value         the new value for the tuple's component.
    */
   public abstract void insert(Tuple2DBasics tupleToUpdate, double value);

   /**
    * Sets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param tupleToUpdate the tuple to update. Modified.
    * @param value         the new value for the tuple's component.
    */
   public void insert(Tuple3DBasics tupleToUpdate, double value)
   {
      asAxis3D().insert(tupleToUpdate, value);
   }

   /**
    * Gets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param tuple the tuple to get the component's value of. Not modified.
    * @return the compoent's value.
    */
   public abstract double extract(Tuple2DReadOnly tuple);

   /**
    * Gets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param tuple the tuple to get the component's value of. Not modified.
    * @return the compoent's value.
    */
   public double extract(Tuple3DReadOnly tuple)
   {
      return asAxis3D().extract(tuple);
   }

   /**
    * Obtains the other axis, i.e. {@link #X}'s other is {@link #Y} and {@link #Y}'s other is
    * {@link #X}
    *
    * @return the other axis.
    */
   public abstract Axis2D other();

   /**
    * Obtains the {@link Axis3D}'s constant that corresponds to this axis 2D.
    * <p>
    * For {@link #X} this returns {@link Axis3D#X} and for {@link #Y} this returns {@link Axis3D#Y}
    * </p>
    *
    * @return the corresponding 3D axis.
    */
   public abstract Axis3D asAxis3D();

   /**
    * Gets the value of the tuple for the given axis.
    *
    * @param tuple the tuple to get value from. Not modified.
    * @param axis  the {@link Axis2D} to get value for. Not modified.
    * @return the double value of {@code tuple} for {@code axis}.
    * @deprecated Use {@code tuple.getElement(axis)} instead.
    */
   @Deprecated
   public static double get(Tuple2DReadOnly tuple, Axis2D axis)
   {
      return axis.extract(tuple);
   }

   /**
    * Sets the value of the given tuple for the given axis to the given value.
    *
    * @param tupleToModify the tuple to set value of. Modified.
    * @param axis          the {@link Axis2D} to set value for. Not modified.
    * @param value         the double value to set {@code axis} of {@code tupleToModify} to.
    * @deprecated Use {@code tupleToModify.setElement(axis, value)} instead.
    */
   @Deprecated
   public static void set(Tuple2DBasics tupleToModify, Axis2D axis, double value)
   {
      axis.insert(tupleToModify, value);
   }

   /**
    * Gets the value of the tuple for the given axis.
    *
    * @param tuple the tuple to get value from. Not modified.
    * @param axis  the {@link Axis2D} to get value for. Not modified.
    * @return the double value of {@code tuple} for {@code axis}.
    * @deprecated Use {@code tuple.getElement(axis)} instead.
    */
   @Deprecated
   public static double get(Tuple3DReadOnly tuple, Axis2D axis)
   {
      return axis.extract(tuple);
   }

   /**
    * Sets the value of the given tuple for the given axis to the given value.
    *
    * @param tupleToModify the tuple to set value of. Modified.
    * @param axis          the {@link Axis2D} to set value for. Not modified.
    * @param value         the double value to set {@code axis} of {@code tupleToModify} to.
    * @deprecated Use {@code tupleToModify.setElement(axis, value)} instead.
    */
   @Deprecated
   public static void set(Tuple3DBasics tupleToModify, Axis2D axis, double value)
   {
      axis.insert(tupleToModify, value);
   }

   /**
    * Obtains the next axis, in a clockwise fashion.
    *
    * @return next clockwise axis
    * @deprecated Use {@link #other()} instead.
    */
   @Deprecated
   public Axis2D getNextClockwiseAxis()
   {
      return other();
   }

   /**
    * Obtains the next axis, in a counterclockwise fashion.
    *
    * @return next counterclockwise axis
    * @deprecated Use {@link #other()} instead.
    */
   @Deprecated
   public Axis2D getNextCounterClockwiseAxis()
   {
      return other();
   }
}
