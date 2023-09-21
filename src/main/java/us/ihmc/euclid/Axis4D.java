package us.ihmc.euclid;

import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.UnitVector4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

/**
 * {@code Axis4D} can be used to provide a simple and readable way to refer the three main axes of a
 * coordinate system.
 */
public enum Axis4D implements UnitVector4DReadOnly
{
   /** The x-axis is usually associated with the forward direction. */
   X(1.0, 0.0, 0.0, 0.0)
   {
      @Override
      public double extract(Tuple4DReadOnly tuple)
      {
         return tuple.getX();
      }

      @Override
      public void insert(Vector4DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setX(value);
      }

      @Override
      public Axis4D next()
      {
         return Y;
      }

      @Override
      public Axis4D previous()
      {
         return S;
      }
   },
   /**
    * The y-axis is usually associated with the direction pointing to the left of the forward direction
    * and is horizontal.
    */
   Y(0.0, 1.0, 0.0, 0.0)
   {
      @Override
      public double extract(Tuple4DReadOnly tuple)
      {
         return tuple.getY();
      }

      @Override
      public void insert(Vector4DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setY(value);
      }

      @Override
      public Axis4D next()
      {
         return Z;
      }

      @Override
      public Axis4D previous()
      {
         return X;
      }
   },
   /**
    * The z-axis is usually associated with the vertical direction, parallel to the gravity vector but
    * pointing the opposite direction.
    */
   Z(0.0, 0.0, 1.0, 0.0)
   {
      @Override
      public double extract(Tuple4DReadOnly tuple)
      {
         return tuple.getZ();
      }

      @Override
      public void insert(Vector4DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setZ(value);
      }

      @Override
      public Axis4D next()
      {
         return S;
      }

      @Override
      public Axis4D previous()
      {
         return Y;
      }
   },
   /**
    * The s-axis is usually associated with the scalar value of a 4D vector.
    */
   S(0.0, 0.0, 0.0, 1.0)
   {
      @Override
      public double extract(Tuple4DReadOnly tuple)
      {
         return tuple.getS();
      }

      @Override
      public void insert(Vector4DBasics tupleToUpdate, double value)
      {
         tupleToUpdate.setS(value);
      }

      @Override
      public Axis4D next()
      {
         return X;
      }

      @Override
      public Axis4D previous()
      {
         return Z;
      }
   };

   /**
    * Static final field holding the return from {@link #values()}. This field should be used in place
    * of calling values() for garbage-free operations.
    */
   public static final Axis4D[] values = values();

   private final double x, y, z, s;
   private final UnitVector4DReadOnly negated = EuclidCoreFactories.newNegativeLinkedUnitVector4D(this);

   Axis4D(double x, double y, double z, double s)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.s = s;
   }

   /**
    * Returns a view of this axis negated.
    *
    * @return this axis negated.
    */
   public UnitVector4DReadOnly negated()
   {
      return negated;
   }

   @Override
   public double dot(Tuple4DReadOnly other)
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
    * Returns the s-component of this axis.
    *
    * @return the s-component.
    */
   @Override
   public double getS()
   {
      return s;
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
    * Returns the s-component of this axis.
    *
    * @return the s-component.
    */
   @Override
   public double getRawS()
   {
      return s;
   }

   /**
    * Sets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param vectorToUpdate the vector to update. Modified.
    * @param value          the new value for the vector's component.
    */
   public abstract void insert(Vector4DBasics vectorToUpdate, double value);

   /**
    * Gets the value of {@code tupleToUpdate} for the component along this axis.
    *
    * @param tuple the tuple to get the component's value of. Not modified.
    * @return the compoent's value.
    */
   public abstract double extract(Tuple4DReadOnly tuple);

   /**
    * Obtains the next axis.
    * <p>
    * Here's the correspondence from each axis to its next:
    * <ul>
    * <li>{@link #X}'s next: {@link #Y}.
    * <li>{@link #Y}'s next: {@link #Z}.
    * <li>{@link #Z}'s next: {@link #S}.
    * <li>{@link #S}'s next: {@link #X}.
    * </ul>
    * </p>
    *
    * @return the next axis.
    */
   public abstract Axis4D next();

   /**
    * Obtains the previous axis.
    * <p>
    * Here's the correspondence from each axis to its previous:
    * <ul>
    * <li>{@link #X}'s previous: {@link #S}.
    * <li>{@link #Y}'s previous: {@link #X}.
    * <li>{@link #Z}'s previous: {@link #Y}.
    * <li>{@link #S}'s previous: {@link #Z}.
    * </ul>
    * </p>
    *
    * @return the previous axis.
    */
   public abstract Axis4D previous();
}
