package us.ihmc.euclid;

import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

/**
 * {@code Axis3D} can be used to provide a simple and readable way to refer the three main axes of a
 * coordinate system.
 */
public enum Axis3D implements UnitVector3DReadOnly
{
   /** The x-axis is usually associated with the forward direction. */
   X(1.0, 0.0, 0.0)
   {
      @Override
      public double extract(Tuple3DReadOnly tuple)
      {
         return tuple.getX();
      }

      @Override
      public void insert(Tuple3DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setX(value);
      }

      @Override
      public Axis3D next()
      {
         return Y;
      }

      @Override
      public Axis3D previous()
      {
         return Z;
      }
   },
   /**
    * The y-axis is usually associated with the direction pointing to the left of the forward direction
    * and is horizontal.
    */
   Y(0.0, 1.0, 0.0)
   {
      @Override
      public double extract(Tuple3DReadOnly tuple)
      {
         return tuple.getY();
      }

      @Override
      public void insert(Tuple3DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setY(value);
      }

      @Override
      public Axis3D next()
      {
         return Z;
      }

      @Override
      public Axis3D previous()
      {
         return X;
      }
   },
   /**
    * The z-axis is usually associated with the vertical direction, parallel to the gravity vector but
    * pointing the opposite direction.
    */
   Z(0.0, 0.0, 1.0)
   {
      @Override
      public double extract(Tuple3DReadOnly tuple)
      {
         return tuple.getZ();
      }

      @Override
      public void insert(Tuple3DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setZ(value);
      }

      @Override
      public Axis3D next()
      {
         return X;
      }

      @Override
      public Axis3D previous()
      {
         return Y;
      }
   };

   /**
    * Static final field holding the return from {@link #values()}. This field should be used in place
    * of calling values() for garbage-free operations.
    */
   public static final Axis3D[] values = values();

   private final double x, y, z;
   private final UnitVector3DReadOnly negated = EuclidCoreFactories.newNegativeLinkedUnitVector3D(this);

   Axis3D(double x, double y, double z)
   {
      this.x = x;
      this.y = y;
      this.z = z;
   }

   /**
    * Returns a view of this axis negated.
    *
    * @return this axis negated.
    */
   public UnitVector3DReadOnly negated()
   {
      return negated;
   }

   @Override
   public double dot(Tuple3DReadOnly other)
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
    * Sets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param tupleToUpdate the tuple to update. Modified.
    * @param value         the new value for the tuple's component.
    */
   public abstract void insert(Tuple3DBasics tupleToUpdate, double value);

   /**
    * Gets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param tuple the tuple to get the component's value of. Not modified.
    * @return the compoent's value.
    */
   public abstract double extract(Tuple3DReadOnly tuple);

   /**
    * Obtains the next axis.
    * <p>
    * Here's the correspondence from each axis to its next:
    * <ul>
    * <li>{@link #X}'s next: {@link #Y}.
    * <li>{@link #Y}'s next: {@link #Z}.
    * <li>{@link #Z}'s next: {@link #X}.
    * </ul>
    * </p>
    *
    * @return the next axis.
    */
   public abstract Axis3D next();

   /**
    * Obtains the previous axis.
    * <p>
    * Here's the correspondence from each axis to its previous:
    * <ul>
    * <li>{@link #X}'s previous: {@link #Z}.
    * <li>{@link #Y}'s previous: {@link #X}.
    * <li>{@link #Z}'s previous: {@link #Y}.
    * </ul>
    * </p>
    *
    * @return the previous axis.
    */
   public abstract Axis3D previous();
}
